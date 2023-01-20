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

#define MUTEX_LOCK_INTERVAL_MS 1000
#define MEB_CXL_BUS I2C_BUS2

#define MUTEX_LOCK_INTERVAL_MS 1000

/*  define config for sensors  */
#define TMP75_IN_ADDR (0x90 >> 1)
#define TMP75_OUT_ADDR (0x92 >> 1)
#define TMP75_TEMP_OFFSET 0x00
#define MPS_MP5990_ADDR (0x80 >> 1)
#define E1S_ADDR (0xD4 >> 1)
#define E1S_OFFSET 0x00
#define SQ52205_1_ADDR (0x80 >> 1)
#define SQ52205_2_ADDR (0x82 >> 1)
#define SQ52205_3_ADDR (0x88 >> 1)
#define SQ52205_4_ADDR (0x8A >> 1)

/*  threshold sensor number, 1 based  */
#define SENSOR_NUM_TEMP_TMP75_IN 0x01
#define SENSOR_NUM_TEMP_TMP75_OUT 0x02
#define SENSOR_NUM_TEMP_PU4 0x03
#define SENSOR_NUM_TEMP_E1S_0 0x34
#define SENSOR_NUM_TEMP_E1S_1 0x35
#define SENSOR_NUM_TEMP_E1S_2 0x36
#define SENSOR_NUM_TEMP_E1S_3 0x37

#define SENSOR_NUM_VOL_P12V_AUX 0x04
#define SENSOR_NUM_VOL_P3V3_AUX 0x05
#define SENSOR_NUM_VOL_P1V2_AUX 0x06
#define SENSOR_NUM_VOL_P3V3 0x07
#define SENSOR_NUM_VOL_P12V_AUX_CARD01 0x08
#define SENSOR_NUM_VOL_P12V_AUX_CARD02 0x09
#define SENSOR_NUM_VOL_P12V_AUX_CARD03 0x0A
#define SENSOR_NUM_VOL_P12V_AUX_CARD04 0x0B
#define SENSOR_NUM_VOL_P12V_AUX_CARD05 0x0C
#define SENSOR_NUM_VOL_P12V_AUX_CARD06 0x0D
#define SENSOR_NUM_VOL_P12V_AUX_CARD07 0x0E
#define SENSOR_NUM_VOL_P12V_AUX_CARD08 0x0F
#define SENSOR_NUM_VOL_P12V_AUX_CARD09 0x10
#define SENSOR_NUM_VOL_P12V_AUX_CARD10 0x11
#define SENSOR_NUM_VOL_P12V_AUX_CARD11 0x12
#define SENSOR_NUM_VOL_P12V_AUX_CARD12 0x13
#define SENSOR_NUM_VOL_P12V_AUX_CARD13 0x14
#define SENSOR_NUM_VOL_P12V_AUX_CARD14 0x15

#define SENSOR_NUM_CUR_P12V_AUX 0x16
#define SENSOR_NUM_CUR_P12V_AUX_CARD01 0x17
#define SENSOR_NUM_CUR_P12V_AUX_CARD02 0x18
#define SENSOR_NUM_CUR_P12V_AUX_CARD03 0x19
#define SENSOR_NUM_CUR_P12V_AUX_CARD04 0x1A
#define SENSOR_NUM_CUR_P12V_AUX_CARD05 0x1B
#define SENSOR_NUM_CUR_P12V_AUX_CARD06 0x1C
#define SENSOR_NUM_CUR_P12V_AUX_CARD07 0x1D
#define SENSOR_NUM_CUR_P12V_AUX_CARD08 0x1E
#define SENSOR_NUM_CUR_P12V_AUX_CARD09 0x1F
#define SENSOR_NUM_CUR_P12V_AUX_CARD10 0x20
#define SENSOR_NUM_CUR_P12V_AUX_CARD11 0x21
#define SENSOR_NUM_CUR_P12V_AUX_CARD12 0x22
#define SENSOR_NUM_CUR_P12V_AUX_CARD13 0x23
#define SENSOR_NUM_CUR_P12V_AUX_CARD14 0x24

#define SENSOR_NUM_PWR_P12V_AUX 0x25
#define SENSOR_NUM_PWR_P12V_AUX_CARD01 0x26
#define SENSOR_NUM_PWR_P12V_AUX_CARD02 0x27
#define SENSOR_NUM_PWR_P12V_AUX_CARD03 0x28
#define SENSOR_NUM_PWR_P12V_AUX_CARD04 0x29
#define SENSOR_NUM_PWR_P12V_AUX_CARD05 0x2A
#define SENSOR_NUM_PWR_P12V_AUX_CARD06 0x2B
#define SENSOR_NUM_PWR_P12V_AUX_CARD07 0x2C
#define SENSOR_NUM_PWR_P12V_AUX_CARD08 0x2D
#define SENSOR_NUM_PWR_P12V_AUX_CARD09 0x2E
#define SENSOR_NUM_PWR_P12V_AUX_CARD10 0x2F
#define SENSOR_NUM_PWR_P12V_AUX_CARD11 0x30
#define SENSOR_NUM_PWR_P12V_AUX_CARD12 0x31
#define SENSOR_NUM_PWR_P12V_AUX_CARD13 0x32
#define SENSOR_NUM_PWR_P12V_AUX_CARD14 0x33

/** PCIE card sensor config **/
#define CXL_TMP75_IN_ADDR (0x92 >> 1)
#define CXL_CTRL_ADDR (0x98 >> 1)
#define CXL_U6_INA233_ADDR (0x8A >> 1)
#define CXL_U7_INA233_ADDR (0x80 >> 1)
#define CXL_VR_A0V8_ADDR (0xC8 >> 1)
#define CXL_VR_A0V9_ADDR (0xC8 >> 1)
#define CXL_VR_D0V8_ADDR (0xB0 >> 1)
#define CXL_VR_VDDQAB_ADDR (0xB0 >> 1)
#define CXL_VR_VDDQCD_ADDR (0xB4 >> 1)
#define CXL_U8_LTC2991_ADDR (0x90 >> 1)
#define CXL_U9_LTC2991_ADDR (0x98 >> 1)

/** PCIE card sensor number **/
/* E1.S */
#define SENSOR_NUM_TEMP_JCN_E1S_0 0x01
#define SENSOR_NUM_TEMP_JCN_E1S_1 0x02

/* CXL */
#define SENSOR_NUM_TEMP_CXL_TMP75_IN 0x01
#define SENSOR_NUM_TEMP_CXL_CTRL 0x02

#define SENSOR_NUM_VOL_P12V_STBY_4CP 0x03
#define SENSOR_NUM_VOL_P3V3_STBY_4CP 0x04
#define SENSOR_NUM_VOL_P5V_STBY 0x05
#define SENSOR_NUM_VOL_P1V8_ASIC 0x06
#define SENSOR_NUM_VOL_P12V_STBY 0x07
#define SENSOR_NUM_VOL_P3V3_STBY 0x08
#define SENSOR_NUM_VOL_PVPP_AB 0x09
#define SENSOR_NUM_VOL_PVTT_AB 0x0A
#define SENSOR_NUM_VOL_PVPP_CD 0x0B
#define SENSOR_NUM_VOL_PVTT_CD 0x0C
#define SENSOR_NUM_VOL_P0V8_ASICA 0x0D
#define SENSOR_NUM_VOL_P0V9_ASICA 0x0E
#define SENSOR_NUM_VOL_P0V8_ASICD 0x0F
#define SENSOR_NUM_VOL_PVDDQ_AB 0x10
#define SENSOR_NUM_VOL_PVDDQ_CD 0x11

#define SENSOR_NUM_CUR_P12V_STBY_4CP 0x12
#define SENSOR_NUM_CUR_P3V3_STBY_4CP 0x13
#define SENSOR_NUM_CUR_P0V8_ASICA 0x14
#define SENSOR_NUM_CUR_P0V9_ASICA 0x15
#define SENSOR_NUM_CUR_P0V8_ASICD 0x16
#define SENSOR_NUM_CUR_PVDDQ_AB 0x17
#define SENSOR_NUM_CUR_PVDDQ_CD 0x18

#define SENSOR_NUM_PWR_P12V_STBY_4CP 0x19
#define SENSOR_NUM_PWR_P3V3_STBY_4CP 0x1A
#define SENSOR_NUM_PWR_P0V8_ASICA 0x1B
#define SENSOR_NUM_PWR_P0V9_ASICA 0x1C
#define SENSOR_NUM_PWR_P0V8_ASICD 0x1D
#define SENSOR_NUM_PWR_PVDDQ_AB 0x1E
#define SENSOR_NUM_PWR_PVDDQ_CD 0x1F

extern sensor_cfg plat_e1s_1_12_sensor_config[];
extern sensor_cfg plat_e1s_13_14_sensor_config[];
extern sensor_cfg plat_cxl_sensor_config[];
extern const int E1S_SENSOR_CONFIG_SIZE;
extern const int CXL_SENSOR_CONFIG_SIZE;

void load_sensor_config(void);
bool is_dc_access(uint8_t sensor_num);
bool is_e1s_access(uint8_t sensor_num);
struct k_mutex *get_i2c_mux_mutex(uint8_t i2c_bus);
bool get_cxl_sensor_config_index(uint8_t sensor_num, uint8_t *index);
bool get_pcie_card_mux_config(uint8_t dev_type, uint8_t card_id, uint8_t sensor_num,
			      mux_config *card_mux_cfg, mux_config *cxl_mux_cfg);
void pal_init_drive(sensor_cfg *cfg_table, uint8_t cfg_size, uint8_t device_type, uint8_t card_id);

#endif
