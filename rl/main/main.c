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
#include "copro/copro.h"
#include "esp_spiffs.h"


///// CHANGE HERE!
//#include "gather_sht30_fast.c"
//#include "gather_sht30.c"
//#include "gps_acc.c"
//#include "breathingled.c"
//#include "tofsense_fast.c"
// #include "tofsense.c"
#include "pedometer.c"
/////

extern const uint8_t bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t bin_end[] asm("_binary_ulp_main_bin_end");

#if !defined(MRBC_MEMORY_SIZE)
#define MRBC_MEMORY_SIZE (1024*40)
#endif
static uint8_t memory_pool[MRBC_MEMORY_SIZE];


#define CHECK_WAKEUP_OVERHEAD 0

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
#if CHECK_WAKEUP_OVERHEAD
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_sleep_enable_timer_wakeup(1000 * 1000);
  esp_light_sleep_start();
  esp_sleep_enable_timer_wakeup(1000 * 1000);
  esp_light_sleep_start();
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
#else
#if CONFIG_IDF_TARGET_ESP32C6
  ulp_lp_core_load_binary(bin_start,(bin_end-bin_start));
  //printf("ulp_lp_core_load_binary: %d\n", ulp_lp_core_load_binary(bin_start,(bin_end-bin_start)));
#else
  ulp_riscv_load_binary(bin_start,(bin_end-bin_start));
  //printf("ulp_riscv_load_binary: %d\n", ulp_riscv_load_binary(bin_start,(bin_end-bin_start)));
#endif
  //printf("size: %d\n", bin_end-bin_start);
  esp_sleep_enable_ulp_wakeup();
  //printf("esp_sleep_enable_ulp_wakeup: %d\n", esp_sleep_enable_ulp_wakeup());
  mrbc_init(memory_pool, MRBC_MEMORY_SIZE);
  
  mrbc_add_copro_class(0);
  mrbc_add_spiffs_class(0);

  if( mrbc_create_task(mrbbuf, 0) != NULL ){
    mrbc_run();
  }
#endif
}


