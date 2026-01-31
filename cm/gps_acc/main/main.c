#include <stdio.h>
#include <stdint.h>
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

static void lp_core_init(void)
{
    ESP_ERROR_CHECK(ulp_lp_core_load_binary(lp_core_main_bin_start, (lp_core_main_bin_end - lp_core_main_bin_start)));
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
static uint32_t getCpuFrequencyMhz() {
  rtc_cpu_freq_config_t conf;
  rtc_clk_cpu_freq_get_config(&conf);
  return conf.freq_mhz;
}

static void delayUs(int wait) {
  if(wait > 100) {
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
    esp_sleep_enable_timer_wakeup(wait);
    esp_light_sleep_start();
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
    goto FIN;
  }
  if(wait > 1000) {
    vTaskDelay(wait / 1000 / portTICK_PERIOD_MS);
    wait = wait % 1000;
  }
  if(wait == 0) goto FIN;
  uint32_t end = (wait * getCpuFrequencyMhz()) + esp_cpu_get_cycle_count();
  while(esp_cpu_get_cycle_count()< end);
FIN:
  return;
}

static void delayMs(int wait) {
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_sleep_enable_timer_wakeup(1000 * wait);
  esp_light_sleep_start();
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
}

int32_t value_x[12];
int32_t value_y[12];
int32_t value_z[12];

extern int ulp_lp_core_i2c_master_write_to_device(int, uint16_t, const uint8_t *, size_t, int32_t);
extern int ulp_lp_core_i2c_master_read_from_device(int, uint16_t, uint8_t *, size_t, int32_t);

int conv(uint8_t * ary, int base) {
  return ((ary[base] << 24) | (ary[base+1] << 16)) >> 18;
}
const uint8_t FIFO_READ[] = {0x18};
int sensor_read(int idx) {
  uint8_t data_rd[6];
  ulp_lp_core_i2c_master_write_to_device(LP_I2C_NUM_0, ADXL367_I2C_ADDR, FIFO_READ, sizeof(FIFO_READ), -1);
  delayMs(5);
  if(ulp_lp_core_i2c_master_read_from_device(LP_I2C_NUM_0, ADXL367_I2C_ADDR, data_rd, sizeof(data_rd), -1) != ESP_OK) return 1;
  value_x[idx] = conv(data_rd, 0);
  value_y[idx] = conv(data_rd, 2);
  value_z[idx] = conv(data_rd, 4);
  return 0;
}

const uint8_t CMD_MEASURE[] = {0x2D, 2};
const uint8_t CMD_STANDBY[] = {0x2D, 0};

int sensor_on(void) {
  return ulp_lp_core_i2c_master_write_to_device(LP_I2C_NUM_0, ADXL367_I2C_ADDR, CMD_MEASURE, sizeof(CMD_MEASURE), -1);
}
int sensor_off(void) {
  return ulp_lp_core_i2c_master_write_to_device(LP_I2C_NUM_0, ADXL367_I2C_ADDR, CMD_STANDBY, sizeof(CMD_STANDBY), -1);
}

int read_for_1sec(void) {
  int res = 0;
  sensor_on();
  delayMs(20);
  for(int i = 0; i < 12; ++i) {
    delayMs(80);
    if(sensor_read(i)) res = 1;
  }
  sensor_off();
  return res;
}

#define GRAVITY 4000
#define THRESHOLD 2000

int app_app_main (void)
{
  while(1) {
    acc = 
 FAIL:
    return 0;
}

measure(sda){
  int acc; //絶対値の合計
  int x = 
  acc = abs(x) + abs(y) + abs(z);
}

void app_main(void)
{
    // AUTO makes the RTC IO unstable.
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON); //RTC回路をON二設定
    lp_i2c_init();
    lp_core_init();
    while(1) {
        app_app_main();
        return;
    }
}

