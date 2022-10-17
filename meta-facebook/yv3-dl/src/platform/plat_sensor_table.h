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

extern struct k_mutex vr_page_mutex;
#define VR_PAGE_MUTEX_TIMEOUT_MS 1000

/*  define config for sensors  */
#define TMP75_IN_ADDR (0x92 >> 1)
#define TMP75_OUT_ADDR (0x94 >> 1)
#define TMP75_FIO_ADDR (0x90 >> 1)
#define TMP431_ADDR (0x98 >> 1)
#define TMP75_TEMP_OFFSET 0x00

#define SSD0_ADDR (0xD4 >> 1)
#define SSD0_OFFSET 0x00

#define PCH_ADDR (0x2C >> 1)
#define ME_SENSOR_NUM_TEMP_PCH 0x08
#define CPU_PECI_ADDR 0x30

#define VCCIN_VCCSA_ADDR (0xC0 >> 1)
#define VCCIO_P3V3_STBY_ADDR (0xC4 >> 1)
#define VDDQ_ABC_ADDR (0xC8 >> 1)
#define VDDQ_DEF_ADDR (0xCC >> 1)
#define VR_VOL_CMD 0x8B
#define VR_CUR_CMD 0x8C
#define VR_TEMP_CMD 0x8D
#define VR_PWR_CMD 0x96

#define LTC4282_ADDR (0x80 >> 1)
#define MPS_MP5990_ADDR (0x40 >> 1)

/*  YV3 DL sensor numbers  */
#define SENSOR_NUM_T_MB1 0x01
#define SENSOR_NUM_T_MB2 0x02
#define SENSOR_NUM_T_FIO 0x03
#define SENSOR_NUM_NM_T_PCH 0x04
#define SENSOR_NUM_T_CPU0 0x05
#define SENSOR_NUM_T_CPU0_THERMAL_MARGIN 0x0D
#define SENSOR_NUM_T_CPU0_TJMAX 0x25
#define SENSOR_NUM_T_CPU0_PKG_PWR 0x1E

#define SENSOR_NUM_NM_T_DIMMA 0x06
#define SENSOR_NUM_NM_T_DIMMB 0x07
#define SENSOR_NUM_NM_T_DIMMC 0x09
#define SENSOR_NUM_NM_T_DIMMD 0x0A
#define SENSOR_NUM_NM_T_DIMME 0x0B
#define SENSOR_NUM_NM_T_DIMMF 0x0C

#define SENSOR_NUM_T_DIMM_ABC_VR 0x14
#define SENSOR_NUM_T_DIMM_DEF_VR 0x15
#define SENSOR_NUM_CURR_DIMM_ABC_VR 0x35
#define SENSOR_NUM_CURR_DIMM_DEF_VR 0x36
#define SENSOR_NUM_V_DIMM_ABC_VR 0x2C
#define SENSOR_NUM_V_DIMM_DEF_VR 0x2D
#define SENSOR_NUM_PWR_DIMM_ABC_VR 0x3F
#define SENSOR_NUM_PWR_DIMM_DEF_VR 0x42

#define SENSOR_NUM_T_NVME0 0x1F
#define SENSOR_NUM_T_NVME1 0x0E

#define SENSOR_NUM_TEP_PVCCIN_VR 0x10
#define SENSOR_NUM_CUR_PVCCIN_VR 0x31
#define SENSOR_NUM_VOL_PVCCIN_VR 0x27
#define SENSOR_NUM_PWR_PVCCIN_VR 0x3A

#define SENSOR_NUM_TEP_PVCCSA_VR 0x11
#define SENSOR_NUM_CUR_PVCCSA_VR 0x32
#define SENSOR_NUM_VOL_PVCCSA_VR 0x28
#define SENSOR_NUM_PWR_PVCCSA_VR 0x3C

#define SENSOR_NUM_TEP_PVCCIO_VR 0x12
#define SENSOR_NUM_CUR_PVCCIO_VR 0x33
#define SENSOR_NUM_VOL_PVCCIO_VR 0x29
#define SENSOR_NUM_PWR_PVCCIO_VR 0x3D

#define SENSOR_NUM_TEP_P3V3_STBY_VR 0x13
#define SENSOR_NUM_CUR_P3V3_STBY_VR 0x34
#define SENSOR_NUM_VOL_P3V3_STBY_VR 0x2A
#define SENSOR_NUM_PWR_P3V3_STBY_VR 0x3E

#define SENSOR_NUM_HSC_TEMP 0x0F
#define SENSOR_NUM_HSC_VIN 0x26
#define SENSOR_NUM_HSC_COUT 0x30
#define SENSOR_NUM_HSC_EIN 0x39
#define SENSOR_NUM_HSC_PIN 0x2E

/***********************ADC***********************/
#define SENSOR_NUM_V_12 0x20
#define SENSOR_NUM_V_3_3 0xFF
#define SENSOR_NUM_V_3_3_S 0x22
#define SENSOR_NUM_V_1_5 0x23
#define SENSOR_NUM_V_BAT 0x21
#define SENSOR_NUM_V_PCH 0x24

/**********************Event**********************/

#define SENSOR_NUM_SYS_STA 0x46
#define SENSOR_NUM_POWER_ERR 0x56
#define SENSOR_NUM_PROC_FAIL 0x65
#define SENSOR_NUM_VR_HOT 0xB4
#define SENSOR_NUM_CPUDIMM_HOT 0xB3
#define SENSOR_NUM_NMI 0xEA
#define SENSOR_NUM_CATERR 0xEB

enum {
	VR_INF = 0,
	VR_RNS,
};

uint8_t plat_get_config_size();
void load_sensor_config(void);
uint8_t pal_get_extend_sensor_config();

#endif
