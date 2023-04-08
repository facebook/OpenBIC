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

#define TMP75_1OU_BOARD_ADDR (0x92 >> 1)
#define TMP641_CXL_CNTR_ADDR (0x98 >> 1)
#define INA233_12V_ADDR (0x8A >> 1)
#define INA233_3V3_ADDR (0x80 >> 1)
#define VR_P0V85_ASIC_ADDR (0xEC >> 1)
#define VR_PVDDQAB_ADDR (0xEC >> 1)
#define VR_P0V8_ASIC_ADDR (0xE4 >> 1)
#define VR_PVDDQCD_ADDR (0xE4 >> 1)

#define TMP75_TEMP_OFFSET 0x00
#define VR_PAGE_OFFSET 0x00

#define SENSOR_NUM_TEMP_TMP75 0x50
#define SENSOR_NUM_TEMP_CXL 0x51
#define SENSOR_NUM_TEMP_P0V85_ASIC 0x52
#define SENSOR_NUM_TEMP_PVDDQ_AB 0x53
#define SENSOR_NUM_TEMP_P0V8_ASIC 0x54
#define SENSOR_NUM_TEMP_PVDDQ_CD 0x55

#define SENSOR_NUM_POWER_ERROR 0x56
#define SENSOR_NUM_VOL_P1V2_STBY 0x57
#define SENSOR_NUM_VOL_P1V2_ASIC 0x58
#define SENSOR_NUM_VOL_P1V8_ASIC 0x59
#define SENSOR_NUM_VOL_PVPP_AB 0x5A
#define SENSOR_NUM_VOL_PVPP_CD 0x5B
#define SENSOR_NUM_VOL_PVTT_AB 0x5C
#define SENSOR_NUM_VOL_PVTT_CD 0x5D
#define SENSOR_NUM_VOL_P0V75_ASIC 0x5E
#define SENSOR_NUM_VOL_P12V_STBY 0x5F
#define SENSOR_NUM_VOL_P3V3_STBY 0x60
#define SENSOR_NUM_VOL_P0V85_ASIC 0x61
#define SENSOR_NUM_VOL_PVDDQ_AB 0x62
#define SENSOR_NUM_VOL_P0V8_ASIC 0x63
#define SENSOR_NUM_VOL_PVDDQ_CD 0x64

#define SENSOR_NUM_CUR_P12V_STBY 0x65
#define SENSOR_NUM_CUR_P3V3_STBY 0x66
#define SENSOR_NUM_CUR_P0V85_ASIC 0x67
#define SENSOR_NUM_CUR_PVDDQ_AB 0x68
#define SENSOR_NUM_CUR_P0V8_ASIC 0x69
#define SENSOR_NUM_CUR_PVDDQ_CD 0x6A

#define SENSOR_NUM_PWR_P12V_STBY 0x6B
#define SENSOR_NUM_PWR_P3V3_STBY 0x6C
#define SENSOR_NUM_PWR_P0V85_ASIC 0x6D
#define SENSOR_NUM_PWR_PVDDQ_AB 0x6E
#define SENSOR_NUM_PWR_P0V8_ASIC 0x6F
#define SENSOR_NUM_PWR_PVDDQ_CD 0x70

#endif
