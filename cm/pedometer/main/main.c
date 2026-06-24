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

static void lp_core_init(void)
{
    ESP_ERROR_CHECK(ulp_lp_core_load_binary(lp_core_main_bin_start, (lp_core_main_bin_end - lp_core_main_bin_start)));
}

int conv(uint8_t * ary, int base) {
    return ((ary[base] << 24) | (ary[base+1] << 16)) >> 18;
}

int sensor_read(int *x, int *y, int *z) {
    uint8_t data_rd[6];
    uint8_t reg_addr = 0x0E; 

    int ret = i2c_master_write_read_device(
        I2C_NUM_0,
        ADXL367_I2C_ADDR, 
        &reg_addr, 1,
        data_rd, sizeof(data_rd),
        pdMS_TO_TICKS(500)
    );

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

// ==========================================
// AN-2554 準拠 パラメータ設定
// ==========================================
// 実測データ(回転:1000~1500, 腕振り:3000~6000)に基づき、中間の1700に設定
#define SENSITIVITY 1700   
#define SAMPLE_RATE_MS 20    

// 時間枠の定義 (ノイズ対策で少し厳しく0.3秒からに設定)
#define TIME_0_3_SEC 15
#define TIME_1_0_SEC 50

int current_state = 0;
int max_value = 0;
int min_value = INT_MAX;
int dynamic_threshold = 0;
int time_counter = 0;
int consecutive_steps = 0;
int step_count = 0;

void step_algorithm_an2554(int mag) {
    if (dynamic_threshold == 0) dynamic_threshold = mag;
    dynamic_threshold = (dynamic_threshold * 15 + mag) / 16; 

    time_counter++;

    switch (current_state) {
        case 0: // 山探し
            if (mag > max_value) {
                // 新しい山を検知
                max_value = mag;
            } 
            else if ((max_value - mag) > (SENSITIVITY / 2) && 
                     max_value > (dynamic_threshold + SENSITIVITY / 2)) {
                    //検知した山から明確に下がり始めた && 山がノイズではない
                current_state = 1;
                min_value = mag;
                time_counter = 0;
            }
            break;

        case 1: // 谷探し
            if (mag < min_value) {
                min_value = mag;
            } 
            else if ((mag - min_value) > (SENSITIVITY / 2) && 
                     min_value < (dynamic_threshold - SENSITIVITY / 2)) {
                
                if (time_counter >= TIME_0_3_SEC) {
                    //間隔が短すぎる場合はノイズとみなす
                    consecutive_steps++;
                    
                    if (consecutive_steps == 4) {
                        step_count += 4;
                        printf("連続歩行検知！計 %d 歩 (落差: %d) \n", step_count, max_value - min_value);
                    } else if (consecutive_steps > 4) {
                        step_count++;
                        printf(" STEP! 計 %d 歩 (落差: %d) \n", step_count, max_value - min_value);
                    } else {
                        // デバッグ用に落差を表示
                        printf("歩行候補を検知 (現在 %d 連続, 落差: %d)\n", consecutive_steps, max_value - min_value);
                    }
                } else {
                    consecutive_steps = 0;
                    // printf("ノイズ検知（早すぎる）\n");
                }

                max_value = mag;
                current_state = 0;
            } 
            else if (time_counter > TIME_1_0_SEC) {
                // タイムアウト: 山を見つけた後、谷が見つからない場合はリセット
                consecutive_steps = 0;
                max_value = mag;
                current_state = 0;
                // printf("タイムアウト\n");
            }
            break;
    }
}

const uint8_t CMD_MEASURE[] = {0x2D, 2};
const uint8_t CMD_STANDBY[] = {0x2D, 0};

int sensor_on(void) {
    return i2c_master_write_to_device(I2C_NUM_0, ADXL367_I2C_ADDR, CMD_MEASURE, sizeof(CMD_MEASURE), portMAX_DELAY);
}
int sensor_off(void) {
    return i2c_master_write_to_device(I2C_NUM_0, ADXL367_I2C_ADDR, CMD_STANDBY, sizeof(CMD_STANDBY), portMAX_DELAY);
}

// フィルタリング値を保持する静的変数
static int filtered_mag = 0;

void app_app_main(void)
{
    int x, y, z;
    
    if (sensor_read(&x, &y, &z) != 0) {
        return;
    }
    // 1. 生の合成加速度（ユークリッド距離）
    int raw_mag = (int)sqrt((double)x*x + (double)y*y + (double)z*z);

    // 2. 超軽量ローパスフィルタ（EMA）
    if (filtered_mag == 0) {
        filtered_mag = raw_mag; // 初回
    } else {
        // 過去の滑らかな波(3) に、最新の生データ(1) を少しだけ混ぜる
        // これにより電気的ノイズや微小な震えを吸収します
        filtered_mag = (filtered_mag * 3 + raw_mag) / 4;
    }

    // 3. フィルタリング後の綺麗な波をアルゴリズムへ
    step_algorithm_an2554(filtered_mag);
    // printf(">Raw:%d\n>Filtered:%d\n>Threshold:%d\n", raw_mag, filtered_mag, dynamic_threshold);
}

void app_main(void) 
{
    // esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    i2c_init();
    lp_core_init();
    sensor_on();

    printf("Pedometer algorithm started...\n");

    while(1) {
        app_app_main();
        vTaskDelay(pdMS_TO_TICKS(20)); // 20ms待機 (50Hz)
    }
}