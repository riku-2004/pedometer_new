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
#include <math.h>
#include <string.h>
#include "esp_spiffs.h"
#include "esp_log.h"
#include "limits.h"

extern const uint8_t lp_core_main_bin_start[] asm("_binary_ulp_core_main_bin_start");
extern const uint8_t lp_core_main_bin_end[]   asm("_binary_ulp_core_main_bin_end");
#define ADXL367_I2C_ADDR 0x1D
#define MEASURE_MODE_ON {0x2D, 0x02}

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

// 時間枠の定義 (ノイズ対策で少し厳しく0.2秒からに設定) 50Hzのため、0.2秒は10サンプル、1.0秒は50サンプル
#define TIME_0_2_SEC 10
#define TIME_1_0_SEC 50

//タイムウィンドウ方式(20ms周期で50Hz)のため、15サンプル分のウィンドウを使用 20ms * 15 = 300ms = 0.3秒
#define WINDOW_SIZE 15
#define WINDOW_CENTER 7

static int window[WINDOW_SIZE] = {0};
static bool window_filled = false;
static int center_val = 0;
int current_state = 0;
int time_since_mountain = 0;
int max_value = 0;
int min_value = INT_MAX;
int dynamic_threshold = 0;
int time_counter = 0;
int consecutivesteps = 0;
int step_count = 0;

// void step_algorithm_an2554(int mag) {
//     if (dynamic_threshold == 0) dynamic_threshold = mag;
//     dynamic_threshold = (dynamic_threshold * 15 + mag) / 16; 

//     time_counter++;

//     switch (current_state) {
//         case 0: // 山探し
//             if (mag > max_value) {
//                 // 新しい山を検知
//                 max_value = mag;
//             } 
//             else if ((max_value - mag) > (SENSITIVITY / 2) && 
//                      max_value > (dynamic_threshold + SENSITIVITY / 2)) {
//                     //検知した山から明確に下がり始めた && 山がノイズではない
//                 current_state = 1;
//                 min_value = mag;
//                 time_counter = 0;
//             }
//             break;

//         case 1: // 谷探し
//             if (mag < min_value) {
//                 min_value = mag;
//             } 
//             else if ((mag - min_value) > (SENSITIVITY / 2) && 
//                      min_value < (dynamic_threshold - SENSITIVITY / 2)) {
                
//                 if (time_counter >= TIME_0_2_SEC) {
//                     //間隔が短すぎる場合はノイズとみなす
//                     consecutive_steps++;
                    
//                     if (consecutive_steps == 4) {
//                         step_count += 4;
//                         printf("連続歩行検知！計 %d 歩 (落差: %d) \n", step_count, max_value - min_value);
//                     } else if (consecutive_steps > 4) {
//                         step_count++;
//                         printf(" STEP! 計 %d 歩 (落差: %d) \n", step_count, max_value - min_value);
//                     } else {
//                         // デバッグ用に落差を表示
//                         printf("歩行候補を検知 (現在 %d 連続, 落差: %d)\n", consecutive_steps, max_value - min_value);
//                     }
//                 } else {
//                     consecutive_steps = 0;
//                     // printf("ノイズ検知（早すぎる）\n");
//                 }

//                 max_value = mag;
//                 current_state = 0;
//             } 
//             else if (time_counter > TIME_1_0_SEC) {
//                 // タイムアウト: 山を見つけた後、谷が見つからない場合はリセット
//                 consecutive_steps = 0;
//                 max_value = mag;
//                 current_state = 0;
//                 // printf("タイムアウト\n");
//             }
//             break;
//     }
// }

//タイムウィンドウ方式
void step_algorithm_an2554(int mag){
    //データを一個ずつ前にずらす
    for(int i = 0; i < WINDOW_SIZE - 1; i++) {
        window[i] = window[i + 1];
    }
    //一番うしろに新しいデータを入れる
    window[WINDOW_SIZE - 1] = mag;

    if(!window_filled){
        // ウィンドウがまだ埋まっていない場合は、埋まるまで待つ
        static int fill_count = 0;
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
                //山から谷の落差だけは確認する
                if((max_value - center_val) > (SENSITIVITY)){
                    consecutivesteps++;
                    if(consecutivesteps == 4){
                        step_count += 4;
                        printf("連続歩行検知！計 %d 歩 (落差: %d) \n", step_count, max_value - center_val);
                    } else if(consecutivesteps > 4){
                        step_count++;
                        printf(" STEP! 計 %d 歩 (落差: %d) \n", step_count, max_value - center_val);
                    }
                } else {
                        consecutivesteps = 0;
                        // デバッグ用に落差を表示
                        printf("歩行候補を検知 (現在 %d 連続, 落差: %d)\n", consecutivesteps, max_value - center_val);
                }

                current_state = 0; //山探しに戻る
            }
            else if(time_since_mountain > TIME_1_0_SEC){
                //タイムアウト: 山を見つけた後、谷が見つからない場合はリセット
                consecutivesteps = 0;
                current_state = 0; //山探しに戻る
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

#define BUFFER_SIZE 4
static int circ_buffer[BUFFER_SIZE]={0};
static int buffer_index = 0;
static bool is_first_sample = true;


void app_app_main(void)
{
    int x, y, z;
    
    if (sensor_read(&x, &y, &z) != 0) {
        ESP_LOGW(TAG, "Failed to read sensor data");
        return;
    }
    // 1. 生の合成加速度（ユークリッド距離）
    int raw_mag = (int)sqrt((double)x*x + (double)y*y + (double)z*z);

    // 2. フィルタリング (サーキュラーバッファ)
    if (is_first_sample) {
        //初回のサンプルでは、バッファをすべて同じ値で初期化
        for (int i = 0; i < BUFFER_SIZE; i++) {
            circ_buffer[i] = raw_mag;
        }
        is_first_sample = false;
    } else {
        circ_buffer[buffer_index] = raw_mag;
        buffer_index = (buffer_index + 1) % BUFFER_SIZE;
    }

    int sum = 0;
    for (int i = 0; i < BUFFER_SIZE; i++) {
        sum += circ_buffer[i];
    }
    filtered_mag = sum / BUFFER_SIZE;
    //デバッグ用
    printf(">Filtered:%d\n", filtered_mag);
    

    // printf("/*%d,%d*/\n", raw_mag, filtered_mag);
    // 3. フィルタリング後の綺麗な波をアルゴリズムへ
    step_algorithm_an2554(filtered_mag);
    // printf(">Raw:%d\n>Filtered:%d\n>Threshold:%d\n", raw_mag, filtered_mag, dynamic_threshold);
    // CSVに1行書き込む (書式: "時間, 加速度, 歩数")
    // ※ `center_val` はタイム・ウィンドウの中心データ
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

    // esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    i2c_init();
    lp_core_init();
    sensor_on();

    printf("Pedometer algorithm started...\n");

    while(1) {
        app_app_main();
        vTaskDelay(pdMS_TO_TICKS(20)); // 20ms待機 (50Hz)
        // CSVに1行書き込む (書式: "時間, 加速度, 歩数")
        // ※ `center_val` はタイム・ウィンドウの中心データ
        fprintf(f_write, "%d,%d,%d\n", time_ms, center_val, step_count);
        if (time_ms % 1000 == 0) {
            fflush(f_write); 
        }

        time_ms += 20;
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}