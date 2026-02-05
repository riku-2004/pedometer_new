#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "esp_sleep.h"
#include "ulp_lp_core.h"
#include "lp_core_i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rtc_io.h"
#include "soc/rtc.h"

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

sensor_read(int *x, int *y, int *z) {
  uint8_t data_rd[6];
  const uint8_t FIFO_READ[] = {0x18};
  ulp_lp_core_i2c_master_write_to_device(LP_I2C_NUM_0, ADXL367_I2C_ADDR, FIFO_READ, sizeof(FIFO_READ), 500); //書き込む
  delayMs(5);
  if(ulp_lp_core_i2c_master_read_from_device(LP_I2C_NUM_0, ADXL367_I2C_ADDR, data_rd, sizeof(data_rd), -1) != ESP_OK) return 1; //読み込む(タイムアウトは-1にしておく)
  *x = conv(data_rd, 0);
  *y = conv(data_rd, 2);
  *z = conv(data_rd, 4);
  return 0;
}
static void lp_i2c_init(void)
{
    /* Initialize LP I2C with default configuration */
    const lp_core_i2c_cfg_t i2c_cfg = {
        .i2c_pin_cfg.sda_io_num = GPIO_NUM_6, //SDAをGPIO6に指定
        .i2c_pin_cfg.scl_io_num = GPIO_NUM_7, //SCLをGPIO7に指定
        .i2c_pin_cfg.sda_pullup_en = false, //プルアップ抵抗無効
        .i2c_pin_cfg.scl_pullup_en = false, //プルアップ抵抗無効
        .i2c_timing_cfg.clk_speed_hz = 20000, //i2cクロック20kHz
        LP_I2C_DEFAULT_SRC_CLK()
    };
    ESP_ERROR_CHECK(lp_core_i2c_master_init(LP_I2C_NUM_0, &i2c_cfg)); //(i2c番号、構成)
}

static int SENSITIVITY = 4000 //4gモード

typedef struct{
    int buffer[BUFFER_SIZE];
    int head; 
    int tail; 
    int count; 
}CircularBuffer; //4サイクルバッファ構造体

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

app_app_main(void)
{
    int x, y, z;
    sensor_read(&x, &y, &z);
    printf("X: %d, Y: %d, Z: %d\n", x, y, z);
    int mag = abs(x) + abs(y) + abs(z); //マンハッタン距離で計算
    printf("Magnitude: %d\n", mag);

    enqueue(&cb, mag);
    int sum = 0;
    int value;
    for (int i = 0; i < BUFFER_SIZE; i++) {
        if (dequeue(&cb, &value)) {
            sum += value;
        }
    }
    int ave = sum / BUFFER_SIZE; //4ウィンドウサイキュラーバッファ

    vTaskDelay(pdMS_TO_TICKS(1000));
}
void app_main(void)
{
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    lp_i2c_init();
    lp_core_init();
    while(1) {
        app_app_main();
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1秒待機
        return 0;
    }
}


