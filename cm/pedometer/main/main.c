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
    ESP_ERROR_CHECK(ulp_lp_core_load_binary(lp_core_main_bin_start, (lp_core_main_bin_end - lp_core_main_bin_start))); //(ポインタ、サイズ)
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

static 
void app_main(void)
{
    rtc_gpio_init(1);
    rtc_gpio_set_direction(1, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_pulldown_dis(1);
    rtc_gpio_pullup_dis(1);
    rtc_gpio_set_level(1, 1);
    // AUTO makes the RTC IO unstable.
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    lp_i2c_init();
    lp_core_init();
    while(1) {
        rtc_gpio_set_level(1, 0); //(PIN1をOFF)
        app_app_main();
        rtc_gpio_set_level(1, 1); //(PIN1をON)
        return;
    }
}


