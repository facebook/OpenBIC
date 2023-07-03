/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef PLAT_SENSOR_TABLE_H
#define PLAT_SENSOR_TABLE_H

#include <stdint.h>
#include "sensor.h"
#include "common_i2c_mux.h"

#define PEX_MAX_NUMBER 2

/*  define config for sensors  */
#define TMP75_IN_ADDR 0x48
#define TMP75_OUT_ADDR 0x4A
#define TMP75_TEMP_OFFSET 0x00
#define LM75BD118_TEMP_OFFSET 0x00
#define XDPE15284D_ADDR 0x64
#define ADM1272_1_ADDR 0x10
#define ADM1272_2_ADDR 0x13
#define LTC4286_1_ADDR 0x40
#define LTC4286_2_ADDR 0x41
#define POWER_BRICK_1_ADDR 0x60
#define POWER_BRICK_2_ADDR 0x61
#define INA233_12V_1_7_ADDR 0x45
#define INA233_12V_2_8_ADDR 0x44
#define INA233_12V_3_9_ADDR 0x43
#define INA233_12V_4_10_ADDR 0x42
#define INA233_12V_5_11_ADDR 0x41
#define INA233_12V_6_ADDR 0x46
#define INA233_12V_12_ADDR 0x40
#define PEX89144_I2C0_ADDR 0x61
#define PEX89144_I2CS_ADDR 0x59
#define SQ52205_P1V25_1_ADDR 0x40
#define SQ52205_P1V25_2_ADDR 0x41
#define FIO_THERMAL_ADDR 0x48
#define P1V8_PEX_ADC_CHANNEL 4
#define P1V8_PEX_LOW_THRESHOLD 1.674

/* Temperature sensor number, 1 based  */
#define SENSOR_NUM_TEMP_OUTLET_1 0x01
#define SENSOR_NUM_TEMP_OUTLET_2 0x02
#define SENSOR_NUM_TEMP_HSC_1 0x03
#define SENSOR_NUM_TEMP_HSC_2 0x04
#define SENSOR_NUM_TEMP_PEX_0 0x05
#define SENSOR_NUM_TEMP_PEX_1 0x06
#define SENSOR_NUM_TEMP_XDPE15284 0x07
#define SENSOR_NUM_TEMP_POWER_BRICK_1 0x09
#define SENSOR_NUM_TEMP_POWER_BRICK_2 0x0A
#define SENSOR_NUM_TEMP_P0V8_VDD_1 0x5F
#define SENSOR_NUM_TEMP_P0V8_VDD_2 0x60
#define SENSOR_NUM_TEMP_FIO_INLET 0x67

/* Voltage sensor number */
#define SENSOR_NUM_VOL_P51V_AUX_L 0x0B
#define SENSOR_NUM_VOL_P51V_AUX_R 0x0C
#define SENSOR_NUM_VOL_P51V_STBY_L 0x53
#define SENSOR_NUM_VOL_P51V_STBY_R 0x54
#define SENSOR_NUM_VOL_P12V_AUX_1 0x0D
#define SENSOR_NUM_VOL_P12V_AUX_2 0x0E
#define SENSOR_NUM_VOL_P5V_AUX 0x10
#define SENSOR_NUM_VOL_P3V3_AUX 0x11
#define SENSOR_NUM_VOL_P1V2_AUX 0x12
#define SENSOR_NUM_VOL_P1V8_PEX 0x13
#define SENSOR_NUM_VOL_P1V8_VDD_1 0x1E
#define SENSOR_NUM_VOL_P1V8_VDD_2 0x1F
#define SENSOR_NUM_VOL_P1V25_VDD_1 0x20
#define SENSOR_NUM_VOL_P1V25_VDD_2 0x21
#define SENSOR_NUM_VOL_P0V8_VDD_1 0x57
#define SENSOR_NUM_VOL_P0V8_VDD_2 0x58
#define SENSOR_NUM_VOL_P12V_1_M_AUX 0x61
#define SENSOR_NUM_VOL_P12V_2_M_AUX 0x62

#define SENSOR_NUM_VOL_P12V_ACCL_1 0x23
#define SENSOR_NUM_VOL_P12V_ACCL_2 0x24
#define SENSOR_NUM_VOL_P12V_ACCL_3 0x25
#define SENSOR_NUM_VOL_P12V_ACCL_4 0x26
#define SENSOR_NUM_VOL_P12V_ACCL_5 0x27
#define SENSOR_NUM_VOL_P12V_ACCL_6 0x28
#define SENSOR_NUM_VOL_P12V_ACCL_7 0x29
#define SENSOR_NUM_VOL_P12V_ACCL_8 0x2A
#define SENSOR_NUM_VOL_P12V_ACCL_9 0x2C
#define SENSOR_NUM_VOL_P12V_ACCL_10 0x2D
#define SENSOR_NUM_VOL_P12V_ACCL_11 0x2E
#define SENSOR_NUM_VOL_P12V_ACCL_12 0x2F

/* Current sensor number */
#define SENSOR_NUM_CUR_P51V_AUX_L 0x30
#define SENSOR_NUM_CUR_P51V_AUX_R 0x31
#define SENSOR_NUM_CUR_P51V_STBY_L 0x5D
#define SENSOR_NUM_CUR_P51V_STBY_R 0x5E
#define SENSOR_NUM_CUR_P12V_AUX_1 0x32
#define SENSOR_NUM_CUR_P12V_AUX_2 0x33
#define SENSOR_NUM_CUR_P0V8_VDD_1 0x59
#define SENSOR_NUM_CUR_P0V8_VDD_2 0x5A
#define SENSOR_NUM_CUR_P12V_1_M_AUX 0x63
#define SENSOR_NUM_CUR_P12V_2_M_AUX 0x64

#define SENSOR_NUM_CUR_P12V_ACCL_1 0x34
#define SENSOR_NUM_CUR_P12V_ACCL_2 0x35
#define SENSOR_NUM_CUR_P12V_ACCL_3 0x36
#define SENSOR_NUM_CUR_P12V_ACCL_4 0x37
#define SENSOR_NUM_CUR_P12V_ACCL_5 0x38
#define SENSOR_NUM_CUR_P12V_ACCL_6 0x39
#define SENSOR_NUM_CUR_P12V_ACCL_7 0x3A
#define SENSOR_NUM_CUR_P12V_ACCL_8 0x3C
#define SENSOR_NUM_CUR_P12V_ACCL_9 0x3D
#define SENSOR_NUM_CUR_P12V_ACCL_10 0x3E
#define SENSOR_NUM_CUR_P12V_ACCL_11 0x3F
#define SENSOR_NUM_CUR_P12V_ACCL_12 0x40

/* Power sensor number */
#define SENSOR_NUM_PWR_P51V_AUX_L 0x41
#define SENSOR_NUM_PWR_P51V_AUX_R 0x42
#define SENSOR_NUM_PWR_P51V_STBY_L 0x55
#define SENSOR_NUM_PWR_P51V_STBY_R 0x56
#define SENSOR_NUM_PWR_P12V_AUX_1 0x43
#define SENSOR_NUM_PWR_P12V_AUX_2 0x44
#define SENSOR_NUM_PWR_P0V8_VDD_1 0x5B
#define SENSOR_NUM_PWR_P0V8_VDD_2 0x5C
#define SENSOR_NUM_PWR_P12V_1_M_AUX 0x65
#define SENSOR_NUM_PWR_P12V_2_M_AUX 0x66

#define SENSOR_NUM_PWR_P12V_ACCL_1 0x45
#define SENSOR_NUM_PWR_P12V_ACCL_2 0x46
#define SENSOR_NUM_PWR_P12V_ACCL_3 0x47
#define SENSOR_NUM_PWR_P12V_ACCL_4 0x48
#define SENSOR_NUM_PWR_P12V_ACCL_5 0x49
#define SENSOR_NUM_PWR_P12V_ACCL_6 0x4A
#define SENSOR_NUM_PWR_P12V_ACCL_7 0x4B
#define SENSOR_NUM_PWR_P12V_ACCL_8 0x4C
#define SENSOR_NUM_PWR_P12V_ACCL_9 0x4D
#define SENSOR_NUM_PWR_P12V_ACCL_10 0x50
#define SENSOR_NUM_PWR_P12V_ACCL_11 0x51
#define SENSOR_NUM_PWR_P12V_ACCL_12 0x52

/** ACCL sensor config **/
#define ACCL_FREYA_1_ADDR (0xD6 >> 1)
#define ACCL_FREYA_2_ADDR (0xD0 >> 1)
#define ACCL_12V_INA233_ADDR (0x80 >> 1)
#define ACCL_3V3_1_INA233_ADDR (0x82 >> 1)
#define ACCL_3V3_2_INA233_ADDR (0x84 >> 1)
#define ACCL_12V_INA233_INIT_ARG_OFFSET 0
#define ACCL_3V3_1_INA233_INIT_ARG_OFFSET 1
#define ACCL_3V3_2_INA233_INIT_ARG_OFFSET 2

/** ACCL sensor number **/
#define SENSOR_NUM_TEMP_ACCL_FREYA_1 0x01
#define SENSOR_NUM_TEMP_ACCL_FREYA_2 0x02
#define SENSOR_NUM_VOL_ACCL_P12V_EFUSE 0x03
#define SENSOR_NUM_VOL_ACCL_P3V3_1 0x04
#define SENSOR_NUM_VOL_ACCL_P3V3_2 0x05
#define SENSOR_NUM_VOL_ACCL_FREYA_1_1 0x06
#define SENSOR_NUM_VOL_ACCL_FREYA_1_2 0x07
#define SENSOR_NUM_VOL_ACCL_FREYA_2_1 0x0B
#define SENSOR_NUM_VOL_ACCL_FREYA_2_2 0x0C
#define SENSOR_NUM_CUR_ACCL_P12V_EFUSE 0x08
#define SENSOR_NUM_CUR_ACCL_P3V3_1 0x09
#define SENSOR_NUM_CUR_ACCL_P3V3_2 0x0A
#define SENSOR_NUM_PWR_ACCL_P12V_EFUSE 0x0D
#define SENSOR_NUM_PWR_ACCL_P3V3_1 0x0E
#define SENSOR_NUM_PWR_ACCL_P3V3_2 0x0F
#define SENSOR_NUM_PWR_ACCL_FREYA_1 0x10
#define SENSOR_NUM_PWR_ACCL_FREYA_2 0x11

extern const int ACCL_SENSOR_CONFIG_SIZE;

void load_sensor_config(void);
bool is_acb_power_good();
bool is_dc_access(uint8_t sensor_num);
bool is_pcie_device_access(uint8_t card_id);
struct k_mutex *get_i2c_mux_mutex(uint8_t i2c_bus);
int get_accl_bus(uint8_t card_id, uint8_t sensor_number);
sensor_cfg *get_accl_sensor_config(uint8_t card_id, uint8_t sensor_num);
bool get_accl_mux_config(uint8_t card_id, mux_config *accl_mux);
bool get_mux_channel_config(uint8_t card_id, uint8_t sensor_number, mux_config *channel_mux);
sensor_cfg *get_accl_sensor_cfg_info(uint8_t card_id, uint8_t *cfg_count);

#endif
