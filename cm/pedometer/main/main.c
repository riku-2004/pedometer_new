#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lp_core_i2c.h"
#include <stdbool.h>
#include "ulp_lp_core.h"
#include <string.h>
#include "esp_spiffs.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "soc/lp_i2c_reg.h"
#include "hal/i2c_ll.h"
#include "esp_rom_sys.h"

#define LP_I2C_FIFO_LEN     SOC_LP_I2C_FIFO_LEN
#define LP_I2C_READ_MODE    I2C_MASTER_READ
#define LP_I2C_WRITE_MODE   I2C_MASTER_WRITE
#define LP_I2C_ACK          I2C_MASTER_ACK
#define LP_I2C_NACK         I2C_MASTER_NACK
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
i2c_dev_t *dev = I2C_LL_GET_HW(LP_I2C_NUM_0);
static bool s_ack_check_en = true;

static void lp_core_i2c_format_cmd(uint32_t cmd_idx, uint8_t op_code, uint8_t ack_val,
                                   uint8_t ack_expected, uint8_t ack_check_en, uint8_t byte_num)
{
    if (cmd_idx >= sizeof(dev->command)) {
        /* We only have limited HW command registers.
         * Although unlikely, make sure that we do not write to an out of bounds index.
         */
        return;
    }

    /* Form new command */
    i2c_ll_hw_cmd_t hw_cmd = {
        .done = 0,                // CMD Done
        .op_code = op_code,       // Opcode
        .ack_val = ack_val,       // ACK bit sent by I2C controller during READ.
        // Ignored during RSTART, STOP, END and WRITE cmds.
        .ack_exp = ack_expected,  // ACK bit expected by I2C controller during WRITE.
        // Ignored during RSTART, STOP, END and READ cmds.
        .ack_en = ack_check_en,   // I2C controller verifies that the ACK bit sent by the
        // slave device matches the ACK expected bit during WRITE.
        // Ignored during RSTART, STOP, END and READ cmds.
        .byte_num = byte_num,     // Byte Num
    };

    /* Write new command to cmd register */
    i2c_ll_master_write_cmd_reg(dev, hw_cmd, cmd_idx);
}

static inline esp_err_t lp_core_i2c_wait_for_interrupt(uint32_t intr_mask, int32_t ticks_to_wait)
{
    uint32_t intr_status = 0;
    uint32_t to = 0;

    while (1) {
        i2c_ll_get_intr_raw_mask(dev, &intr_status);
        if (intr_status & intr_mask) {
            if (intr_status & LP_I2C_NACK_INT_ST) {
                /* The ACK/NACK received during a WRITE operation does not match the expected ACK/NACK level
                 * Abort and return an error.
                 */
                i2c_ll_clear_intr_mask(dev, intr_mask);
                return ESP_ERR_INVALID_RESPONSE;
            } else if (intr_status & LP_I2C_TRANS_COMPLETE_INT_ST_M) {
                /* Transaction complete.
                 * Clear interrupt bits and break
                 */
                i2c_ll_clear_intr_mask(dev, intr_mask);
                break;
            } else {
                /* We received an I2C_END_DETECT_INT.
                 * This means we are not yet done with the transaction.
                 * Simply clear the interrupt bit and break.
                 */
                i2c_ll_clear_intr_mask(dev, intr_mask);
                break;
            }
            break;
        }

        if (ticks_to_wait > -1) {
            /* If the ticks_to_wait value is not -1, keep track of ticks and
             * break from the loop once the timeout is reached.
             */
            // ulp_lp_core_delay_cycles(1);
            esp_rom_delay_us(1);
            to++;
            if (to >= ticks_to_wait) {
                /* Timeout. Clear interrupt bits and return an error */
                i2c_ll_clear_intr_mask(dev, intr_mask);
                return ESP_ERR_TIMEOUT;
            }
        }
    }

    /* We reach here only if we are in a good state */
    return ESP_OK;
}

static inline void lp_core_i2c_config_device_addr(uint32_t cmd_idx, uint16_t device_addr,  uint32_t rw_mode, uint8_t *addr_len)
{
    uint8_t data_byte = 0;
    uint8_t data_len = 0;

    /* 7-bit addressing mode. We do not support 10-bit addressing mode yet (IDF-7364) */

    // Write the device address + R/W mode in the first Tx FIFO slot
    data_byte = (uint8_t)(((device_addr & 0xFF) << 1) | (rw_mode << 0));
    i2c_ll_write_txfifo(dev, &data_byte, 1);
    data_len++;

    /* Update the HW command register. Expect an ACK from the device */
    lp_core_i2c_format_cmd(cmd_idx, I2C_LL_CMD_WRITE, 0, LP_I2C_ACK, s_ack_check_en, data_len);

    /* Return the address length in bytes */
    *addr_len = data_len;
}

esp_err_t hp_lp_core_i2c_master_write_read_device(i2c_port_t lp_i2c_num, uint16_t device_addr,
                                               const uint8_t *data_wr, size_t write_size,
                                               uint8_t *data_rd, size_t read_size,
                                               int32_t ticks_to_wait)
{
    (void)lp_i2c_num;

    esp_err_t ret = ESP_OK;
    uint32_t cmd_idx = 0;

    if ((write_size == 0) || (read_size == 0)) {
        // Quietly return
        return ESP_OK;
    } else if ((write_size > UINT8_MAX) || (read_size > UINT8_MAX)) {
        // HW register only has an 8-bit byte-num field
        return ESP_ERR_INVALID_SIZE;
    }

    /* If SCL is busy, reset the Master FSM */
    if (i2c_ll_is_bus_busy(dev)) {
        i2c_ll_master_fsm_rst(dev);
    }

    /* Reset the Tx and Rx FIFOs */
    i2c_ll_txfifo_rst(dev);
    i2c_ll_rxfifo_rst(dev);

    /* Enable trans complete interrupt and end detect interrupt for read/write operation */
    uint32_t intr_mask = (1 << LP_I2C_TRANS_COMPLETE_INT_ST_S) | (1 << LP_I2C_END_DETECT_INT_ST_S);
    if (s_ack_check_en) {
        /* Enable LP_I2C_NACK_INT to check for ACK errors */
        intr_mask |= (1 << LP_I2C_NACK_INT_ST_S);
    }
    i2c_ll_clear_intr_mask(dev, intr_mask);

    /* Execute RSTART command to send the START bit */
    lp_core_i2c_format_cmd(cmd_idx++, I2C_LL_CMD_RESTART, 0, 0, 0, 0);

    /* Write device addr and update the HW command register */
    uint8_t addr_len = 0;
    lp_core_i2c_config_device_addr(cmd_idx++, device_addr, LP_I2C_WRITE_MODE, &addr_len);

    /* Write data */
    uint32_t fifo_available = LP_I2C_FIFO_LEN - addr_len; // Initially, 1 or 2 fifo slots are taken by the device address
    uint32_t fifo_size = 0;
    uint32_t data_idx = 0;
    int32_t remaining_bytes = write_size;

    /* The data to be sent must occupy sequential slots of the Tx FIFO.
     * We must account for FIFO wraparound in case the length of data being sent is greater than LP_I2C_FIFO_LEN.
     */
    while (remaining_bytes > 0) {
        /* Select the amount of data that fits in the Tx FIFO */
        fifo_size = MIN(remaining_bytes, fifo_available);

        /* Update the number of bytes remaining to be sent */
        remaining_bytes -= fifo_size;

        /* Write data to the Tx FIFO and update the HW command register. Expect ACKs from the device */
        i2c_ll_write_txfifo(dev, &data_wr[data_idx], fifo_size);
        lp_core_i2c_format_cmd(cmd_idx++, I2C_LL_CMD_WRITE, 0, LP_I2C_ACK, s_ack_check_en, fifo_size);

        /* Insert an End command to signal the end of the write transaction to the HW */
        lp_core_i2c_format_cmd(cmd_idx++, I2C_LL_CMD_END, 0, 0, 0, 0);
        cmd_idx = 0;

        /* Initiate I2C transfer */
        i2c_ll_update(dev);
        i2c_ll_start_trans(dev);

        /* Wait for the transfer to complete */
        ret = lp_core_i2c_wait_for_interrupt(intr_mask, ticks_to_wait);
        if (ret != ESP_OK) {
            /* Transaction error. Abort. */
            return ret;
        }

        /* Update data_idx */
        data_idx += fifo_size;

        /* We now have the full fifo available for writing */
        fifo_available = LP_I2C_FIFO_LEN;
    }

    /* Reset command index */
    cmd_idx = 0;

    /* Execute RSTART command again to send a START condition for the read operation */
    lp_core_i2c_format_cmd(cmd_idx++, I2C_LL_CMD_RESTART, 0, 0, 0, 0);

    /* Write device addr again in read mode */
    lp_core_i2c_config_device_addr(cmd_idx++, device_addr, LP_I2C_READ_MODE, &addr_len);

    /* Read data */
    fifo_size = 0;
    data_idx = 0;
    remaining_bytes = read_size;

    /* The data is received in sequential slots of the Rx FIFO.
     * We must account for FIFO wraparound in case the length of data being received is greater than LP_I2C_FIFO_LEN.
     */
    while (remaining_bytes > 0) {
        /* Select the amount of data that fits in the Rx FIFO */
        fifo_size = MIN(remaining_bytes, LP_I2C_FIFO_LEN);

        /* Update the number of bytes remaining to be read */
        remaining_bytes -= fifo_size;

        /* Update HW command register to read bytes */
        if (fifo_size == 1) {
            /* Read 1 byte and send NACK */
            lp_core_i2c_format_cmd(cmd_idx++, I2C_LL_CMD_READ, LP_I2C_NACK, 0, 0, 1);

            /* STOP */
            lp_core_i2c_format_cmd(cmd_idx++, I2C_LL_CMD_STOP, 0, 0, 0, 0);
        } else if ((fifo_size > 1) && (remaining_bytes == 0)) {
            /* This means it is the last transaction.
             * Read fifo_size - 1 bytes and send ACKs
             */
            lp_core_i2c_format_cmd(cmd_idx++, I2C_LL_CMD_READ, LP_I2C_ACK, 0, 0, fifo_size - 1);

            /* Read last byte and send NACK */
            lp_core_i2c_format_cmd(cmd_idx++, I2C_LL_CMD_READ, LP_I2C_NACK, 0, 0, 1);

            /* STOP */
            lp_core_i2c_format_cmd(cmd_idx++, I2C_LL_CMD_STOP, 0, 0, 0, 0);
        } else {
            /* This means we have to read data more than what can fit in the Rx FIFO.
             * Read fifo_size bytes and send ACKs
             */
            lp_core_i2c_format_cmd(cmd_idx++, I2C_LL_CMD_READ, LP_I2C_ACK, 0, 0, fifo_size);
            lp_core_i2c_format_cmd(cmd_idx++, I2C_LL_CMD_END, 0, 0, 0, 0);
            cmd_idx = 0;
        }

        /* Initiate I2C transfer */
        i2c_ll_update(dev);
        i2c_ll_start_trans(dev);

        /* Wait for the transfer to complete */
        ret = lp_core_i2c_wait_for_interrupt(intr_mask, ticks_to_wait);
        if (ret != ESP_OK) {
            /* Transaction error. Abort. */
            return ret;
        }

        /* Read Rx FIFO */
        i2c_ll_read_rxfifo(dev, &data_rd[data_idx], fifo_size);

        /* Update data_idx */
        data_idx += fifo_size;
    }

    return ret;
}
//===============================lp_core_i2c.cから関数の引用=====================================

// LPコアのバイナリ（ビルド時に自動生成）
extern const uint8_t lp_core_main_bin_start[] asm("_binary_ulp_core_main_bin_start");
extern const uint8_t lp_core_main_bin_end[]   asm("_binary_ulp_core_main_bin_end");

// ADXL367のI2Cアドレス（SDO=GNDのとき0x1D、SDO=VDDのとき0x53）
#define ADXL367_I2C_ADDR 0x1D
// 歩行判定の感度（山と谷の落差の最小許容値）
// 実測データ(回転:1000~1500, 腕振り:3000~6000)に基づき、中間の1700に設定
// マンハッタン距離はユークリッド距離より最大√3倍大きくなるためADI元値(400)より大きい
#define SENSITIVITY 1700

// 時間窓の中心インデックス（WINDOW_SIZE=17のとき中心は8番目）
#define WINDOW_CENTER 8

#define _1_SECOND 50 // 20Hzなので1秒は50サンプル
// 移動平均フィルタの次数（直近4サンプルの平均を取る）
#define FILTER_ORDER 4
// 動的しきい値バッファのサイズ（直近4回の山谷中間値の平均でしきい値を更新）
#define THRESHOLD_ORDER 4
// 動的しきい値の初期値（約1g相当、歩行していない静止状態を想定）
#define INIT_OFFSET_VALUE 4000
// 時間窓のサイズ（FILTER_ORDER=4のとき (4<<2)+1 = 17サンプル = 340ms分）
#define WINDOW_SIZE ((FILTER_ORDER << 2) + 1) //=17

static int window[WINDOW_SIZE] = {0};
static bool window_filled = false;
static int center_val = 0;
int current_state = 0;
int time_since_mountain = 0;
int max_value = 0;
int consecutivesteps = 0;
int step_count = 0;

// フィルタリング用
int32_t buffer_RawData[FILTER_ORDER];
int32_t FilterMeanBuffer;
int8_t  IndexAverage;

// 動的しきい値用
int32_t buffer_dynamic_threshold[THRESHOLD_ORDER];
int32_t ThresholdSum;
int32_t old_threshold;
int32_t NewThreshold;
int8_t IndexThreshold;
int8_t flag_threshold_counter = 0;


static const char *TAG = "STEP_LOGGER";

//================================================SPIFFS=================================================
//SPIFFSの初期化
void init_spiffs() {
    ESP_LOGI(TAG, "Initializing SPIFFS");

    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = NULL,
      .max_files = 5,
      .format_if_mount_failed = true // 初回起動時に自動でフォーマットする
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount or format filesystem");
        return;
    }
    ESP_LOGI(TAG, "SPIFFS mounted successfully!");
}
//LPコアの初期化
static void lp_core_init(void)
{
    ESP_ERROR_CHECK(ulp_lp_core_load_binary(lp_core_main_bin_start, (lp_core_main_bin_end - lp_core_main_bin_start)));
}
//================================================I2C=================================================

extern int ulp_lp_core_i2c_master_write_to_device(int, uint16_t, const uint8_t *, size_t, int32_t);
extern int ulp_lp_core_i2c_master_read_from_device(int, uint16_t, uint8_t *, size_t, int32_t);

int conv(uint8_t * ary, int base) {
    return ((ary[base] << 24) | (ary[base+1] << 16)) >> 18;
}

// I2Cの初期化
static void lp_i2c_init(void)
{
    const lp_core_i2c_cfg_t conf = {
        .i2c_pin_cfg.sda_io_num = GPIO_NUM_6,
        .i2c_pin_cfg.scl_io_num = GPIO_NUM_7,
        .i2c_pin_cfg.sda_pullup_en = false,
        .i2c_pin_cfg.scl_pullup_en = false,
        .i2c_timing_cfg.clk_speed_hz = 20000,
        LP_I2C_DEFAULT_SRC_CLK()
    };
    //confの設定をI2C_NUM_0に適用し、I2Cドライバをインストールする
    ESP_ERROR_CHECK(lp_core_i2c_master_init(LP_I2C_NUM_0, &conf));
}

static void delayMs(int wait) {
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_sleep_enable_timer_wakeup(1000 * wait);
  esp_light_sleep_start();
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
}



// I2Cでセンサーからデータを読み取る関数
int sensor_read(int *x, int *y, int *z) {
    uint8_t data_rd[6];
    uint8_t reg_addr = 0x0E;
    // I2C通信でデータを読み取る
    // printf("before write_to_device\n");
    // ulp_lp_core_i2c_master_write_to_device(LP_I2C_NUM_0, ADXL367_I2C_ADDR, &reg_addr, sizeof(reg_addr), -1);
    // printf("after write_to_device\n");
    // delayMs(5);
    // printf("before read_from_device\n");
    // if(ulp_lp_core_i2c_master_read_from_device(LP_I2C_NUM_0, ADXL367_I2C_ADDR, data_rd, sizeof(data_rd), 10000) != ESP_OK) return 1;
    // printf("after read_from_device\n");
    esp_err_t ret1 = hp_lp_core_i2c_master_write_read_device(LP_I2C_NUM_0, ADXL367_I2C_ADDR, &reg_addr, 1, data_rd, sizeof(data_rd), 100000);
    if (ret1 != ESP_OK) {
        printf("I2C error: 0x%x\n", ret1);
        return 1;
    }
    *x = conv(data_rd, 0);
    *y = conv(data_rd, 2);
    *z = conv(data_rd, 4);
    return 0;
}

//変数の初期化
void init_algorithm() {
    int8_t i;
    step_count = 0;
    IndexAverage = 0;
    IndexThreshold = 0;
    FilterMeanBuffer = 0;
    NewThreshold = 0;
    flag_threshold_counter = 0;
    old_threshold = INIT_OFFSET_VALUE;
    ThresholdSum = INIT_OFFSET_VALUE * THRESHOLD_ORDER;//INIT_OFFSET_VALUE<<2
    for (i = 0; i < THRESHOLD_ORDER; i++) {
        buffer_dynamic_threshold[i] = INIT_OFFSET_VALUE;
    }
    for (i = 0; i < FILTER_ORDER; i++) {
        buffer_RawData[i] = 0;
    }
}

void step_algorithm_an2554(int x, int y, int z) {
    uint32_t ModuleData = abs(x) + abs(y) + abs(z); // 簡易的に絶対値の合計を使用

    //移動平均
    //FilterMeanBuffer:バッファの合計値
    FilterMeanBuffer = FilterMeanBuffer - buffer_RawData[IndexAverage] + ModuleData; //平均から最後の値を引き、新しい値を足す
    uint32_t FilterModuleData = FilterMeanBuffer / FILTER_ORDER; // 平均を計算
    buffer_RawData[IndexAverage] = ModuleData; // フィルタリングされていないバッファにモジュールを格納

    //タイムウィンドウ方式
    //データを一個ずつ前にずらす
    for(int i = 0; i < WINDOW_SIZE - 1; i++) {
        window[i] = window[i + 1];
    }
    //一番うしろに新しいデータを入れる
    window[WINDOW_SIZE - 1] = FilterModuleData;

    if(!window_filled){
        // ウィンドウがまだ埋まっていない場合は、埋まるまで待つ
        static int fill_count = 0;
        fill_count++;
        if(fill_count >= WINDOW_SIZE){
            window_filled = true;
            return;
        }
        
    }
    // ウィンドウが埋まったら中心の値を取得
    center_val = window[WINDOW_CENTER];
    bool is_max = true;
    bool is_min = true;

    //中心のデータが、その窓の中で最小か最大かを判定する
    for(int i=0;i < WINDOW_SIZE; i++){
        if(i == WINDOW_CENTER) continue;
        //中心の値と比較して、最大値か最小値かを判定
        if(window[i] >= center_val) is_max = false;
        if(window[i] <= center_val) is_min = false;
    }

    if(current_state == 1){
        time_since_mountain++;
    }

    switch (current_state){
        case 0: //山探し
            if(is_max){
                max_value = center_val;
                current_state = 1; //谷探しに移行
                time_since_mountain = 0; //タイムアウト計測スタート
            }
            break;
        case 1: //谷探し
            if(is_min){
                //center_val=谷の値になっている
                uint32_t Difference = max_value - center_val;

                // 動的しきい値の更新（差が大きい場合のみ）
                if (Difference > SENSITIVITY) {
                    
                    // 新しいしきい値を計算
                    NewThreshold = (max_value + center_val) / 2;
                    // 動的しきい値のバッファを更新　最後の値を引き、新しい値を足す
                    ThresholdSum = ThresholdSum - buffer_dynamic_threshold[IndexThreshold] + NewThreshold;
                    // 平均を計算して古いしきい値を更新
                    old_threshold = ThresholdSum / THRESHOLD_ORDER;
                    // バッファに新しいしきい値を格納
                    buffer_dynamic_threshold[IndexThreshold] = NewThreshold;
                    IndexThreshold++;
                    // インデックスが範囲を超えた場合は0に戻す
                    if (IndexThreshold > THRESHOLD_ORDER - 1) IndexThreshold = 0;
                }
                //山から谷の落差だけは確認する
                if ((max_value > (old_threshold + (SENSITIVITY >> 1))) && (center_val < (old_threshold - (SENSITIVITY >> 1)))){
                    // しきい値条件を満たしたのでカウンタをリセット
                    flag_threshold_counter = 0;
                    consecutivesteps++;
                    if(consecutivesteps == 4){
                        step_count += 4;
                        printf("連続歩行検知！計 %d 歩 (落差: %d) \n", step_count, max_value - center_val);
                    } else if(consecutivesteps > 4){
                        step_count++;
                        printf(" STEP! 計 %d 歩 (落差: %d) \n", step_count, max_value - center_val);
                    }
                } else {
                    flag_threshold_counter++;
                    //2回連続でしきい値を超えなかった場合は連続歩行カウンタをリセット
                    if(flag_threshold_counter > 1){
                        flag_threshold_counter = 0;
                        consecutivesteps = 0;
                        // デバッグ用に落差を表示
                        printf("歩行候補を検知 (現在 %d 連続, 落差: %d)\n", consecutivesteps, max_value - center_val);
                    }
                }

                current_state = 0; //山探しに戻る
            }
            else if(time_since_mountain > _1_SECOND){ //タイムアウト: 1秒以上経過しても谷が見つからない場合
                //タイムアウト: 山を見つけた後、谷が見つからない場合はリセット
                consecutivesteps = 0;
                current_state = 0; //山探しに戻る
            }
            break;
    } //switch文閉じる
    IndexAverage++;
    if (IndexAverage > FILTER_ORDER - 1) {
        IndexAverage = 0;
    }
}



//POWER_CTLレジスタに2を書き込むことで測定モードにする
const uint8_t CMD_MEASURE[] = {0x2D, 2};

int sensor_on(void) {
    return ulp_lp_core_i2c_master_write_to_device(LP_I2C_NUM_0, ADXL367_I2C_ADDR, CMD_MEASURE, sizeof(CMD_MEASURE), -1);
}


void app_app_main(void)
{
    int x, y, z;
    // printf("before sensor_read\n");
    if (sensor_read(&x, &y, &z) != 0) {
        ESP_LOGW(TAG, "Failed to read sensor data");
        return;
    }
    // printf("before algorithm\n");
    //生の加速度をアルゴリズムへ渡す
    step_algorithm_an2554(x, y, z);
    printf(">Step:%d\n", step_count);
}

void app_main(void) 
{
    init_spiffs();
    // 2. 前回の歩行データがあればシリアルモニタに出力する
    FILE* f_read = fopen("/spiffs/walk_log.csv", "r");
    if (f_read != NULL) {
        printf("\n\n=== PREVIOUS LOG DATA START ===\n");
        char line[128];
        while (fgets(line, sizeof(line), f_read) != NULL) {
            printf("%s", line); // CSVの1行をPCに送信
            delayMs(1); // 送信間隔を少し空ける
        }
        fclose(f_read);
        printf("=== PREVIOUS LOG DATA END ===\n\n");
        printf("※ 上記のデータをコピーしてExcel等に保存してください。\n");
        printf("※ 5秒後に新しいデータの記録を開始します...\n");
        delayMs(5000);
    } else {
        printf("※ 前回のデータはありません。新規作成します。\n");
    }

    // 3. 新しい記録用にファイルを開く（"a"モードなのでデータは追記されます）
    FILE* f_write = fopen("/spiffs/walk_log.csv", "a");
    if (f_write == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }
    
    // CSVのヘッダーを書き込む
    fprintf(f_write, "Time_ms,Filtered_Mag,Step_Count\n");
    fflush(f_write); // ここで一度確実に保存

    static int time_ms = 0;
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    lp_i2c_init();
    init_algorithm();
    lp_core_init();
    uint8_t reg = 0x00;  // DEVID register
    uint8_t devid;
    esp_err_t rett = hp_lp_core_i2c_master_write_read_device(LP_I2C_NUM_0, ADXL367_I2C_ADDR, &reg, 1, &devid, 1, 100000);
    // printf("DEVID test: ret=0x%x, id=0x%x\n", rett, devid);
    sensor_on();

    printf("Pedometer algorithm started...\n");
    // printf("before loop\n");
    while(1) {
        // printf("start loop\n");
        app_app_main();
        // printf("finish app_app_main\n");
        // vTaskDelay(20 / portTICK_PERIOD_MS);
        delayMs(20);
        i2c_ll_master_fsm_rst(dev);
        i2c_ll_txfifo_rst(dev);
        i2c_ll_rxfifo_rst(dev);
        fprintf(f_write, "%d,%d,%d\n", time_ms, center_val, step_count);
        if (time_ms % 1000 == 0) {
            fflush(f_write); 
        }

        time_ms += 20;
    }
}