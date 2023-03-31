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

/*  define config for sensors  */
#define TMP75_MB_ADDR (0x92 >> 1)
#define TMP75_ASIC_ADDR (0x98 >> 1)
#define ASIC_ADDR (0x00 >> 1) // TBU
#define INA233_12V_ADDR (0x8A >> 1)
#define INA233_3V3_ADDR (0x80 >> 1)
#define VR_A0V9_ADDR (0xC8 >> 1)
#define VR_A0V8_ADDR (0xC8 >> 1)
#define VR_D0V8_ADDR (0xB0 >> 1)
#define VR_VDDQAB_ADDR (0xB0 >> 1)
#define VR_VDDQCD_ADDR (0xB4 >> 1)

#define ASIC_OFFSET 0x00 // TBU
#define TMP75_TEMP_OFFSET 0x00
#define VR_PAGE_OFFSET 0x00
#define SMBUS_VOL_CMD 0x8B
#define SMBUS_CUR_CMD 0x8C
#define SMBUS_TEMP_CMD 0x8D
#define SMBUS_PWR_CMD 0x96

#define CHIP_TEMP_OFFSET 0x00
#define DIMM_TEMP_OFFSET 0x01

#define DIMMA_SPD_ADDR 0x50
#define DIMMB_SPD_ADDR 0x51
#define DIMMC_SPD_ADDR 0x52
#define DIMMD_SPD_ADDR 0x53

/*  threshold sensor number, 1 based  */
#define SENSOR_NUM_TEMP_TMP75 0x50
#define SENSOR_NUM_TEMP_CXL 0x51
#define SENSOR_NUM_TEMP_CXL_CNTR 0x52
#define SENSOR_NUM_TEMP_DIMMA 0x53
#define SENSOR_NUM_TEMP_DIMMB 0x54
#define SENSOR_NUM_TEMP_DIMMC 0x55
#define SENSOR_NUM_POWER_ERROR 0x56
#define SENSOR_NUM_TEMP_DIMMD 0x57
#define SENSOR_NUM_TEMP_VR0V9A 0x58
#define SENSOR_NUM_TEMP_VR0V8A 0x59
#define SENSOR_NUM_TEMP_VR0V8D 0x5A
#define SENSOR_NUM_TEMP_VRVDDQAB 0x5B
#define SENSOR_NUM_TEMP_VRVDDQCD 0x5C

#define SENSOR_NUM_VOL_STBY12V 0x5D
#define SENSOR_NUM_VOL_STBY3V3 0x5E
#define SENSOR_NUM_VOL_STBY5V 0x5F
#define SENSOR_NUM_VOL_STBY1V2 0x60
#define SENSOR_NUM_VOL_ASIC_1V8 0x61
#define SENSOR_NUM_VOL_VR0V9A 0x62
#define SENSOR_NUM_VOL_VR0V8A 0x64
#define SENSOR_NUM_VOL_VR0V8D 0x65
#define SENSOR_NUM_VOL_VRVDDQAB 0x66
#define SENSOR_NUM_VOL_VRVDDQCD 0x67
#define SENSOR_NUM_VOL_PVPP_AB 0x68
#define SENSOR_NUM_VOL_PVPP_CD 0x69
#define SENSOR_NUM_VOL_PVTT_AB 0x6A
#define SENSOR_NUM_VOL_PVTT_CD 0x6B

#define SENSOR_NUM_CUR_STBY12V 0x6C
#define SENSOR_NUM_CUR_STBY3V3 0x6D
#define SENSOR_NUM_CUR_VR0V9A 0x6E
#define SENSOR_NUM_CUR_VR0V8A 0x6F
#define SENSOR_NUM_CUR_VR0V8D 0x70
#define SENSOR_NUM_CUR_VRVDDQAB 0x71
#define SENSOR_NUM_CUR_VRVDDQCD 0x72

#define SENSOR_NUM_VR_ALERT 0x72

#define SENSOR_NUM_PWR_STBY12V 0x73
#define SENSOR_NUM_PWR_STBY3V3 0x74
#define SENSOR_NUM_PWR_VR0V9A 0x75
#define SENSOR_NUM_PWR_VR0V8A 0x76
#define SENSOR_NUM_PWR_VR0V8D 0x77
#define SENSOR_NUM_PWR_VRVDDQAB 0x78
#define SENSOR_NUM_PWR_VRVDDQCD 0x79

extern uint8_t plat_get_config_size();
extern void load_sensor_config(void);
int check_vr_type(void);

enum {
	VR_INF = 0,
	VR_RNS,
	PWR_INA233,
	PWR_SGY,
};

#endif
