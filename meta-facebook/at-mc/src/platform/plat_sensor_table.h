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

void load_sensor_config(void);
bool is_dc_access(uint8_t sensor_num);
bool is_e1s_access(uint8_t sensor_num);
struct k_mutex *get_i2c_mux_mutex(uint8_t i2c_bus);

#endif
