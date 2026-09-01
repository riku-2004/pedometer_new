#include <stdlib.h>
#include <limits.h>
#include <stdint.h>
#include <stdbool.h>
#include "soc/soc_caps.h"
#include "hal/lp_core_ll.h"
#include "riscv/rv_utils.h"
#include "riscv/rvruntime-frames.h"
#include "ulp_lp_core.h"
#include "ulp_lp_core_i2c.h"
#include "ulp_lp_core_utils.h"
#include "ulp_lp_core_gpio.h"
#include "ulp_lp_core_interrupts.h"
#include "ulp_lp_core_lp_timer_shared.h"

#define ADXL367_I2C_ADDR 0x1D
int shared_buffer[1500]; // 30秒 × 50Hz
int write_index;     // LPコアが書き込む位置
int conv(uint8_t * ary, int base) {
    return ((ary[base] << 24) | (ary[base+1] << 16)) >> 18;
}
int main (void)
{
    uint8_t reg_addr = 0x0E; // データレジスタのアドレス
    uint8_t data_rd[6]; // 6バイトのデータを格納
    while(1){
        //センサーをI2Cで読み取る
        //合成加速度を計算してRTCばっふぁに書く
        //20ms待機する
        lp_core_i2c_master_write_to_device(0, ADXL367_I2C_ADDR, &reg_addr, sizeof(reg_addr), 100000);
        lp_core_i2c_master_read_from_device(0, ADXL367_I2C_ADDR, data_rd, sizeof(data_rd), 100000);
        int x = conv(data_rd, 0);
        int y = conv(data_rd, 2);
        int z = conv(data_rd, 4);
        //二乗和の計算はHP側で行う
        shared_buffer[write_index * 3 + 0] = x;
        shared_buffer[write_index * 3 + 1] = y;
        shared_buffer[write_index * 3 + 2] = z;
        write_index = (write_index + 1) % 500; // バッファのインデックスを更新
        ulp_lp_core_delay_us(20000); // 20ms待機
    }
}

