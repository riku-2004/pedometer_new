#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "soc/rtc.h"
#include <stdbool.h>
#include <ulp_lp_core.h>
#include <math.h>
#include <string.h>
#include "limits.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_spiffs.h"
#include "esp_log.h"
#include "ulp_core_main.h"
#include "lp_core_i2c.h"

extern const uint8_t lp_core_main_bin_start[] asm("_binary_ulp_core_main_bin_start");
extern const uint8_t lp_core_main_bin_end[]   asm("_binary_ulp_core_main_bin_end");
#define ADXL367_I2C_ADDR 0x1D
// #define MEASURE_MODE_ON {0x2D, 0x02}

static const char *TAG = "STEP_LOGGER";

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

// LPコアの設定　HPコアからの起床をトリガーに設定
ulp_lp_core_cfg_t cfg = {
    .wakeup_source = ULP_LP_CORE_WAKEUP_SOURCE_HP_CPU
};

static void lp_core_init(void)
{
    ESP_ERROR_CHECK(ulp_lp_core_load_binary(lp_core_main_bin_start, (lp_core_main_bin_end - lp_core_main_bin_start)));
    ESP_ERROR_CHECK(ulp_lp_core_run(&cfg));
}

int conv(uint8_t * ary, int base) {
    return ((ary[base] << 24) | (ary[base+1] << 16)) >> 18;
}

static void i2c_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_6,
        .scl_io_num = GPIO_NUM_7,
        .sda_pullup_en = false,
        .scl_pullup_en = false,
        .master.clk_speed = 100000,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
}

static void lp_i2c_init(void)
{
    /* Initialize LP I2C with default configuration */
    const lp_core_i2c_cfg_t i2c_cfg = {
        .i2c_pin_cfg.sda_io_num = GPIO_NUM_6,
        .i2c_pin_cfg.scl_io_num = GPIO_NUM_7,
        .i2c_pin_cfg.sda_pullup_en = false,
        .i2c_pin_cfg.scl_pullup_en = false,
        .i2c_timing_cfg.clk_speed_hz = 20000,
        LP_I2C_DEFAULT_SRC_CLK()
    };
    ESP_ERROR_CHECK(lp_core_i2c_master_init(LP_I2C_NUM_0, &i2c_cfg));
}


// ==========================================
// AN-2554 準拠 パラメータ設定
// ==========================================
// 実測データ(回転:1000~1500, 腕振り:3000~6000)に基づき、中間の1700に設定
#define SENSITIVITY 1700

//タイムウィンドウ方式(20ms周期で50Hz)のため、17サンプル分のウィンドウを使用 20ms * 17 = 340ms = 0.34秒
#define WINDOW_SIZE 17
#define WINDOW_CENTER 8

#define _1_SECOND 50 // 50サンプル = 1秒

#define FILTER_ORDER 4 // 移動平均のサンプル数
#define THRESHOLD_ORDER 4 // 動的しきい値のサンプル数
#define INIT_OFFSET_VALUE 4000 // 初期しきい値のオフセット値

//DeepSleepから復帰後も変数を保持するためにRTC_DATA_ATTRを使用
RTC_DATA_ATTR static int32_t buffer_RawData[FILTER_ORDER];
RTC_DATA_ATTR static int32_t FilterMeanBuffer;
RTC_DATA_ATTR static int32_t buffer_dynamic_threshold[THRESHOLD_ORDER];
RTC_DATA_ATTR static int32_t BufferDinamicThreshold;
RTC_DATA_ATTR static int32_t old_threshold;
RTC_DATA_ATTR static int32_t NewThreshold;
RTC_DATA_ATTR static int8_t IndexThreshold;
RTC_DATA_ATTR static int8_t flag_threshold_counter;
RTC_DATA_ATTR static int IndexAverage = 0;
RTC_DATA_ATTR static int window[WINDOW_SIZE] = {0};
RTC_DATA_ATTR static bool window_filled = false;
RTC_DATA_ATTR static int center_val = 0;
RTC_DATA_ATTR int current_state = 0;
RTC_DATA_ATTR int time_since_mountain = 0;
RTC_DATA_ATTR int max_value = 0;
RTC_DATA_ATTR int consecutivesteps = 0;
RTC_DATA_ATTR int step_count = 0;
RTC_DATA_ATTR int time_ms = 0; // 経過時間を保持する変数

void init_algorithm() {
    int8_t i;
    step_count = 0;
    IndexAverage = 0;
    IndexThreshold = 0;
    FilterMeanBuffer = 0;
    NewThreshold = 0;
    flag_threshold_counter = 0;
    old_threshold = INIT_OFFSET_VALUE;
    BufferDinamicThreshold = INIT_OFFSET_VALUE * THRESHOLD_ORDER;//INIT_OFFSET_VALUE<<2
    for (i = 0; i < THRESHOLD_ORDER; i++) {
        buffer_dynamic_threshold[i] = INIT_OFFSET_VALUE;
    }
    for (i = 0; i < FILTER_ORDER; i++) {
        buffer_RawData[i] = 0;
    }
}

//タイムウィンドウ方式
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
        RTC_DATA_ATTR static int fill_count = 0;
        fill_count++;
        if(fill_count >= WINDOW_SIZE){
            window_filled = true;
            return;
        }
    }

    center_val = window[WINDOW_CENTER];
    bool is_max = true;
    bool is_min = true;

    //中心のデータが、その窓の中で最小か最大かを判定する
    for(int i=0;i < WINDOW_SIZE; i++){
        if(i == WINDOW_CENTER) continue;
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
                //山から谷の落差だけは確認する
                if(Difference > SENSITIVITY){
                    // 新しいしきい値を計算
                    NewThreshold = (max_value + center_val) / 2;
                    // 動的しきい値のバッファを更新　最後の値を引き、新しい値を足す
                    BufferDinamicThreshold = BufferDinamicThreshold - buffer_dynamic_threshold[IndexThreshold] + NewThreshold;
                    // 平均を計算して古いしきい値を更新
                    old_threshold = BufferDinamicThreshold / THRESHOLD_ORDER;
                    // バッファに新しいしきい値を格納
                    buffer_dynamic_threshold[IndexThreshold] = NewThreshold;
                    IndexThreshold++;
                    // インデックスが範囲を超えた場合は0に戻す
                    if (IndexThreshold > THRESHOLD_ORDER - 1) IndexThreshold = 0;
                }
                if(max_value > (old_threshold + (SENSITIVITY >> 1)) && center_val < (old_threshold - (SENSITIVITY >> 1))){
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
                    if(flag_threshold_counter > 1){
                        flag_threshold_counter = 0;
                        consecutivesteps = 0;
                        // デバッグ用に落差を表示
                        printf("歩行候補を検知 (現在 %d 連続, 落差: %d)\n", consecutivesteps, max_value - center_val);
                    }
                }

                current_state = 0; //山探しに戻る
            }
            else if(time_since_mountain > _1_SECOND){
                //タイムアウト: 山を見つけた後、谷が見つからない場合はリセット
                consecutivesteps = 0;
                current_state = 0; //山探しに戻る
            }
            break;
    }
    IndexAverage++;
    if(IndexAverage >  FILTER_ORDER - 1) IndexAverage = 0;
}

//測定モードは2、スタンバイモードは0
const uint8_t CMD_MEASURE[] = {0x2D, 2};
const uint8_t CMD_STANDBY[] = {0x2D, 0};

int sensor_on(void) {
    return i2c_master_write_to_device(I2C_NUM_0, ADXL367_I2C_ADDR, CMD_MEASURE, sizeof(CMD_MEASURE), portMAX_DELAY);
}

void app_app_main(void)
{
    int *shared_buf = (int*)&ulp_shared_buffer;   
    int x, y, z;
    // LPコアからのデータを読み取る
    for(int i = 0; i < 500; i++) {
        //x[0],y[0],z[0],x[1],y[1],z[1],...の順で格納されているので、3つずつ取り出す
        x = shared_buf[i * 3 + 0];
        y = shared_buf[i * 3 + 1];
        z = shared_buf[i * 3 + 2];
        step_algorithm_an2554(x, y, z);
    }

    printf(">Step:%d\n", step_count);
}

void app_main(void) 
{
    //初回起動
    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_UNDEFINED) {
        init_spiffs();
        // 2. 前回の歩行データがあればシリアルモニタに出力する
        FILE* f_read = fopen("/spiffs/walk_log.csv", "r");
        if (f_read != NULL) {
            printf("\n\n=== PREVIOUS LOG DATA START ===\n");
            char line[128];
            while (fgets(line, sizeof(line), f_read) != NULL) {
                printf("%s", line); // CSVの1行をPCに送信
                vTaskDelay(1); // 送信間隔を少し空ける
            }
            fclose(f_read);
            printf("=== PREVIOUS LOG DATA END ===\n\n");
            printf("※ 上記のデータをコピーしてExcel等に保存してください。\n");
            printf("※ 5秒後に新しいデータの記録を開始します...\n");
            vTaskDelay(5000 / portTICK_PERIOD_MS);
        } else {
            printf("※ 前回のデータはありません。新規作成します。\n");
        }
        FILE* f_write = fopen("/spiffs/walk_log.csv", "a"); // データの追記
        if (f_write == NULL) {
            ESP_LOGE(TAG, "Failed to open file for writing");
            return;
        }
        fprintf(f_write, "Time_ms,Filtered_Mag,Step_Count\n");
        fflush(f_write); // ここで一度確実に保存
        fclose(f_write);
        i2c_init();
        sensor_on();
        lp_i2c_init();
        //初回は初期化してすぐに寝る
        init_algorithm();
        lp_core_init();
        esp_sleep_enable_timer_wakeup(10000000); // 10秒ごとに起床
        esp_deep_sleep_start();
    }
    //deepsleepから復帰したとき
    else if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER) {
        init_spiffs();
        FILE* f_write = fopen("/spiffs/walk_log.csv", "a");
        if (f_write == NULL) {
            ESP_LOGE(TAG, "Failed to open file for writing");
            return;
        }
        app_app_main();
        time_ms += 10 * 1000; // 10秒加算
        fprintf(f_write, "%d,%d,%d\n", time_ms, center_val, step_count);
        fflush(f_write); // ここで一度確実に保存
        fclose(f_write);
        esp_sleep_enable_timer_wakeup(10000000); // 10秒ごとに起床
        esp_deep_sleep_start();
    }
}