#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rtc_io.h"
#include "soc/rtc.h"
#include "driver/i2c.h"
#include <stdbool.h>
#include <ulp_lp_core.h>
#include <esp_log.h>
#include <math.h>

extern const uint8_t lp_core_main_bin_start[] asm("_binary_ulp_core_main_bin_start");
extern const uint8_t lp_core_main_bin_end[]   asm("_binary_ulp_core_main_bin_end");
#define ADXL367_I2C_ADDR 0x1D
#define MEASURE_MODE_ON {0x2D, 0x02}
#define BUFFER_SIZE 4


static void lp_core_init(void)
{
    ESP_ERROR_CHECK(ulp_lp_core_load_binary(lp_core_main_bin_start, (lp_core_main_bin_end - lp_core_main_bin_start))); //(ポインタ、サイズ)
}

int conv(uint8_t * ary, int base) {
  return ((ary[base] << 24) | (ary[base+1] << 16)) >> 18;
}

int sensor_read(int *x, int *y, int *z) {
    uint8_t data_rd[6];
    
    // 変更点1: FIFOではなく、最新データが入っている XDATA_L レジスタ(0x0E)を指定
    uint8_t reg_addr = 0x0E; 

    // 変更点2: WriteとReadを一度に行う専用関数を使用する（delayは不要）
    int ret = i2c_master_write_read_device(
        I2C_NUM_0, 
        ADXL367_I2C_ADDR, 
        &reg_addr, 1,             // 書き込むレジスタアドレス（1バイト）
        data_rd, sizeof(data_rd), // 読み取ったデータを入れる配列（6バイト）
        pdMS_TO_TICKS(500)
    );

    printf("read result = %d d0=%02x d1=%02x d2=%02x d3=%02x d4=%02x d5=%02x\n", 
            ret, data_rd[0], data_rd[1], data_rd[2], data_rd[3], data_rd[4], data_rd[5]);

    if (ret != ESP_OK) {
        return 1;
    }

    *x = conv(data_rd, 0);
    *y = conv(data_rd, 2);
    *z = conv(data_rd, 4);
    return 0;
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

static int HYSTERESIS = 500; //ヒステリシスの値
static int SENSITIVITY = 4000; //1gモード //一歩とみなすための加速度の変化量の閾値

typedef struct{
    int buffer[BUFFER_SIZE];
    int head; 
    int tail; 
    int count; 
}CircularBuffer; //4サイクルバッファ構造体

static CircularBuffer cb; //サイキュラーバッファのインスタンス

//初期化
void initBuffer(CircularBuffer *cb) {
    cb->head = 0;
    cb->tail = 0;
    cb->count = 0;
}

bool enqueue(CircularBuffer *cb, int value) { //valueをバッファに追加
    if (cb->count == BUFFER_SIZE) {
        return false; //バッファが満杯
    }
    cb->buffer[cb->head] = value;
    cb->head = (cb->head + 1) % BUFFER_SIZE;
    cb->count++;
    return true;
}

bool dequeue(CircularBuffer *cb, int *value) { //バッファから値を取得
    if (cb->count == 0) {
        return false; //バッファが空
    }
    *value = cb->buffer[cb->tail];
    cb->tail = (cb->tail + 1) % BUFFER_SIZE;
    cb->count--;
    return true;
}


static int max_time_counter = 0;
static int max_value = 0;
static int min_value = INT_MAX;
static int current_state = 0; //0:SEARCHING_MAX、1:SEARCHING_MIN、2:JUDGING_PAIR、3:COUNTING_CONSECUTIVE、4:REGULATION_MODE
static int step_count = 0;
enum {
    STATE_SEARCHING_MAX_ID,
    STATE_SEARCHING_MIN_ID,
    STATE_JUDGING_PAIR_ID,
    STATE_COUNTING_CONSECUTIVE_ID,
    STATE_REGULATION_MODE_ID
};

void STATE_SEARCHING_MAX(int mag) {
    
    if (mag > max_value) {
        max_value = mag;
    } else if((max_value - mag) > HYSTERESIS) {
        current_state = STATE_SEARCHING_MIN_ID; //谷を探しに行く
        min_value = mag; //最小値を更新
        max_time_counter = 0; //最大値を更新した後にカウンターをリセット
    }
}

void STATE_SEARCHING_MIN(int mag) {
    max_time_counter++;
    if (mag < min_value) {
        //谷を下っている状態
        min_value = mag;
    } else if((mag - min_value) > HYSTERESIS) {
        current_state = STATE_JUDGING_PAIR_ID;
    } else if (max_time_counter > 50) { //最大値が更新されないまま1秒以上経過したらリセット
        max_value = 0;
        min_value = INT_MAX;
        current_state = STATE_SEARCHING_MAX_ID;
    }
}

void STATE_JUDGING_PAIR(int mag) {
    if (abs(max_value - min_value) > SENSITIVITY) {
        step_count++;
        printf("step_count=%d\n", step_count);
        current_state = STATE_COUNTING_CONSECUTIVE_ID;
    } else {
        //リセットして最初からやり直す
        max_value = 0;
        current_state = STATE_SEARCHING_MAX_ID;
    }
}


void STATE_REGULATION_MODE(int mag) {
    static int cooldown_counter = 0;
    cooldown_counter++;
    if (cooldown_counter > 5) { //クールダウン期間が終了したらリセット
        cooldown_counter = 0;
        max_value = 0;
        min_value = INT_MAX;
        current_state = STATE_SEARCHING_MAX_ID;
    }
}




const uint8_t CMD_MEASURE[] = {0x2D, 2};
const uint8_t CMD_STANDBY[] = {0x2D, 0};
static int sum = 0; //平均計算のための合計値
int sensor_on(void) {
  return i2c_master_write_to_device(I2C_NUM_0, ADXL367_I2C_ADDR, CMD_MEASURE, sizeof(CMD_MEASURE), portMAX_DELAY);
}
int sensor_off(void) {
  return i2c_master_write_to_device(I2C_NUM_0, ADXL367_I2C_ADDR, CMD_STANDBY, sizeof(CMD_STANDBY), portMAX_DELAY);
}

void app_app_main(void)
{
    int x, y, z;
    
    // 1. センサーから値を読み取る（共通）
    if (sensor_read(&x, &y, &z) != 0) {
        return; // 読み取り失敗時は今回のループをスキップ
    }

    // int mag = abs(x) + abs(y) + abs(z);
    int mag = (int)sqrt((double)x*x + (double)y*y + (double)z*z);
    // 2. バッファリングと移動平均の計算（共通）
    if (cb.count < BUFFER_SIZE) {
        // バッファが満杯になるまで待つ
        printf("buffering... %d/%d\n", cb.count, BUFFER_SIZE);
        enqueue(&cb, mag);
        sum += mag; // ← 【超重要】この加算がないと平均が破綻します
        return;     // 満杯になるまではここで処理を終える
    }

    // バッファ満杯なら平均を計算
    int old_value;
    dequeue(&cb, &old_value);
    int ave = (sum - old_value + mag) / BUFFER_SIZE;
    enqueue(&cb, mag);
    sum = sum - old_value + mag;

    // 3. 状態遷移（ステートマシン）
    // 計算された ave を使って、現在の状態に応じた関数に丸投げするだけ！
    switch (current_state) {
        case STATE_SEARCHING_MAX_ID:
            STATE_SEARCHING_MAX(ave);
            break;
            
        case STATE_SEARCHING_MIN_ID:
            STATE_SEARCHING_MIN(ave);
            break;
            
        case STATE_JUDGING_PAIR_ID:
            STATE_JUDGING_PAIR(ave);
            break;
            
        case STATE_REGULATION_MODE_ID:
            STATE_REGULATION_MODE(ave); // ← クールダウン関数を呼ぶ
            break;
            
        default:
            // 予期せぬ状態になったらリセット
            current_state = STATE_SEARCHING_MAX_ID;
            break;
    }
}

void app_main(void)
{
    //デバッグ用
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    printf("Pedometer starting...\n");
    i2c_init();
    printf("I2C initialized.\n");
    lp_core_init();
    printf("ULP core initialized.\n");
    sensor_on();
    printf("Sensor turned on.\n");
    initBuffer(&cb); //サイキュラーバッファの初期化

    sum = 0; //合計値の初期化
    printf("start loop\n");
    while(1) {
        app_app_main();
        vTaskDelay(pdMS_TO_TICKS(20)); // 20ms待機
    }
}


