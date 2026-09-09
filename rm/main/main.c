#include <stdio.h>
#include <stdint.h>
#include "esp_sleep.h"
#if CONFIG_IDF_TARGET_ESP32C6
#include "ulp_lp_core.h"
#else
#include "ulp_riscv.h"
#include "ulp_riscv_lock.h"
#endif
#include "esp_log.h"
#include "unistd.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/ledc.h"
#include "mrubyc.h"
#include "c_hash.h"
#include "copro/copro.h"
#include "esp_spiffs.h"

///// CHANGE HERE!

//#include "gather_sht30_fast.c"
//#include "gather_sht30.c"
//#include "gps_acc.c"
//#include "breathingled.c"
//#include "tofsense.c"
#include "pedometer.c"
/////

extern const uint8_t bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t bin_end[] asm("_binary_ulp_main_bin_end");

#if !defined(MRBC_MEMORY_SIZE)
#define MRBC_MEMORY_SIZE (1024*40)
#endif
static uint8_t memory_pool[MRBC_MEMORY_SIZE];

#define DEFAULT_LEDC_TIMER          LEDC_TIMER_0
#define DEFAULT_LEDC_MODE           LEDC_LOW_SPEED_MODE // ESP32-C6はLOW SPEEDのみ
#define DEFAULT_LEDC_DUTY_RES       LEDC_TIMER_10_BIT
#define DEFAULT_LEDC_FREQUENCY      (5000)

//================================================================
/*! LEDC.new(gpio: num, ch: num, resolution: bits, freq: hz)
    LEDCを初期化する
*/
static void mrbc_ledc_initialize(mrbc_vm *vm, mrbc_value *v, int argc)
{
    if (mrbc_type(v[1]) != MRBC_TT_HASH) {
        mrbc_raise(vm, MRBC_CLASS(ArgumentError), "Argument must be a hash");
        return;
    }

    ledc_timer_config_t timer_conf = {
        .speed_mode = DEFAULT_LEDC_MODE,
        .duty_resolution = DEFAULT_LEDC_DUTY_RES,
        .timer_num = DEFAULT_LEDC_TIMER,
        .freq_hz = DEFAULT_LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_channel_config_t channel_conf = {
        .speed_mode = DEFAULT_LEDC_MODE,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = DEFAULT_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0
    };

    mrbc_value *val;
    val = mrbc_hash_get_p(&v[1], &mrbc_symbol_value(mrbc_str_to_symid("gpio")));
    if (val == NULL || mrbc_type(*val) != MRBC_TT_INTEGER) {
        mrbc_raise(vm, MRBC_CLASS(ArgumentError), "missing or invalid gpio");
        return;
    }
    channel_conf.gpio_num = mrbc_integer(*val);

    val = mrbc_hash_get_p(&v[1], &mrbc_symbol_value(mrbc_str_to_symid("ch")));
    if (val == NULL || mrbc_type(*val) != MRBC_TT_INTEGER) {
        mrbc_raise(vm, MRBC_CLASS(ArgumentError), "missing or invalid ch");
        return;
    }
    channel_conf.channel = mrbc_integer(*val);

    val = mrbc_hash_get_p(&v[1], &mrbc_symbol_value(mrbc_str_to_symid("resolution")));
    if (val != NULL && mrbc_type(*val) == MRBC_TT_INTEGER) {
        timer_conf.duty_resolution = mrbc_integer(*val);
    }

    val = mrbc_hash_get_p(&v[1], &mrbc_symbol_value(mrbc_str_to_symid("freq")));
    if (val != NULL && mrbc_type(*val) == MRBC_TT_INTEGER) {
        timer_conf.freq_hz = mrbc_integer(*val);
    }
    //LED制御の部分なのでコメントアウト
    // val = mrbc_hash_get_p(&v[1], &mrbc_symbol_value(mrbc_str_to_symid("sleep_alive")));
    // if (val != NULL && mrbc_type(*val) >= MRBC_TT_TRUE) {
    //     channel_conf.sleep_mode = LEDC_SLEEP_MODE_KEEP_ALIVE;
    // }

    ledc_timer_config(&timer_conf);
    ledc_channel_config(&channel_conf);
    ledc_fade_func_install(0);

    mrbc_instance_setiv(v, mrbc_str_to_symid("@ch"), &mrbc_integer_value(channel_conf.channel));
    mrbc_instance_setiv(v, mrbc_str_to_symid("@mode"), &mrbc_integer_value(channel_conf.speed_mode));
    mrbc_instance_setiv(v, mrbc_str_to_symid("@res"), &mrbc_integer_value(timer_conf.duty_resolution));
}

//================================================================
/*! led.fade(target_duty, duration_ms)
    指定時間で輝度を変化させる
*/
static void mrbc_ledc_fade(mrbc_vm *vm, mrbc_value *v, int argc)
{
    // 引数チェック
    if (argc != 2 && argc != 3) {
        mrbc_raise(vm, MRBC_CLASS(ArgumentError), "wrong number of arguments");
        return;
    }

    int target_duty = mrbc_integer(v[1]);
    int duration_ms = mrbc_integer(v[2]);

    // インスタンス変数を取得
    mrbc_value val_ch = mrbc_instance_getiv(v, mrbc_str_to_symid("@ch"));
    mrbc_value val_mode = mrbc_instance_getiv(v, mrbc_str_to_symid("@mode"));
    
    // フェード設定と開始
    ledc_set_fade_with_time(mrbc_integer(val_mode), mrbc_integer(val_ch), target_duty, duration_ms);
    ledc_fade_start(mrbc_integer(val_mode), mrbc_integer(val_ch), (argc == 3 && v[3].tt >= MRBC_TT_TRUE) ? LEDC_FADE_NO_WAIT : LEDC_FADE_WAIT_DONE);
}

void mrbc_normal_sleep(struct VM * vm, mrbc_value * v, int argc) {
  vTaskDelay(v[1].i / portTICK_PERIOD_MS);
  SET_NIL_RETURN();
}

void mrbc_add_ledc_class(struct VM * vm) {
  mrb_class *cls_ledc = mrbc_define_class(vm, "Ledc", mrbc_class_object);
  mrbc_define_method(vm, cls_ledc, "initialize", mrbc_ledc_initialize);
  mrbc_define_method(vm, cls_ledc, "fade", mrbc_ledc_fade);
  mrbc_define_method(vm, mrbc_class_object, "sleep", mrbc_normal_sleep);
}

//==========================spiffs関連===============================
void mrbc_spiffs_init(struct VM * vm, mrbc_value * v, int argc){
    printf("SPIFFS init called\n");
    static const char *TAG = "STEP_LOGGER";
    // ESP_LOGI(TAG, "Initializing SPIFFS");

    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = NULL,
      .max_files = 5,
      .format_if_mount_failed = true // 初回起動時に自動でフォーマットする
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    printf("SPIFFS register result: %d\n", ret);
    if (ret != ESP_OK) {
        printf("SPIFFS mount FAILED\n");
        // ESP_LOGE(TAG, "Failed to mount or format filesystem");
        return;
    }
    // ESP_LOGI(TAG, "SPIFFS mounted successfully!");

    //前回の歩行データがあればシリアルモニタに出力する
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
    }
    //CSVのヘッダーを書き込む
    FILE* f_write = fopen("/spiffs/walk_log.csv", "a");
    if (f_write == NULL) {
        return;
    }
    fprintf(f_write, "Time_ms,Filtered_Mag,Step_Count\n");
    fflush(f_write); // ここで一度確実に保存
    fclose(f_write);
}


void mrbc_spiffs_write(struct VM *vm, mrbc_value *v, int argc) { 
        
        // const char *line = (const char *)v[1].string->data;
        FILE *f = fopen("/spiffs/walk_log.csv", "a");
        if (f == NULL){
            printf("SPIFFS write FAILED\n");
            return;
        }
        fprintf(f, "%.*s\n", (int)v[1].string->size, v[1].string->data);
        printf("spiffs write OK\n");
        fclose(f);
    }

void mrbc_add_spiffs_class(struct VM * vm) {
  mrb_class *cls_spiffs = mrbc_define_class(vm, "Spiffs", mrbc_class_object);
  mrbc_define_method(vm, cls_spiffs, "init", mrbc_spiffs_init);
  mrbc_define_method(vm, cls_spiffs, "write", mrbc_spiffs_write);
}


void app_main(void)
{
#if CONFIG_IDF_TARGET_ESP32C6
  ulp_lp_core_load_binary(bin_start,(bin_end-bin_start));
  //printf("ulp_lp_core_load_binary: %d\n", ulp_lp_core_load_binary(bin_start,(bin_end-bin_start)));
  // LP コアを起動する
  ulp_lp_core_cfg_t cfg = {
      .wakeup_source = ULP_LP_CORE_WAKEUP_SOURCE_HP_CPU
  };
  ulp_lp_core_run(&cfg);
#else
  ulp_riscv_load_binary(bin_start,(bin_end-bin_start));
  //printf("ulp_riscv_load_binary: %d\n", ulp_riscv_load_binary(bin_start,(bin_end-bin_start)));
#endif
  //printf("size: %d\n", bin_end-bin_start);
  esp_sleep_enable_ulp_wakeup();
  //printf("esp_sleep_enable_ulp_wakeup: %d\n", esp_sleep_enable_ulp_wakeup());
  mrbc_init(memory_pool, MRBC_MEMORY_SIZE);
  mrbc_add_ledc_class(0);
  mrbc_add_copro_class(0);
  mrbc_add_spiffs_class(0);

  if( mrbc_create_task(mrbbuf, 0) != NULL ){
    mrbc_run();
  }
}
