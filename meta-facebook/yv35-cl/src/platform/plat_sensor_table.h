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

/*  define config for sensors  */
#define TMP75_IN_ADDR (0x92 >> 1)
#define TMP75_OUT_ADDR (0x94 >> 1)
#define TMP75_FIO_ADDR (0x90 >> 1)
#define TMP431_ADDR (0x98 >> 1)
#define TMP75_TEMP_OFFSET 0x00
#define SSD0_ADDR (0xD4 >> 1)
#define SSD0_OFFSET 0x00
#define ADI_ADM1278_ADDR (0x80 >> 1)
#define ADI_LTC4286_ADDR (0x84 >> 1)
#define MPS_MP5990_ADDR (0x16 >> 1)
#define ADI_LTC4282_ADDR (0x88 >> 1)
#define PCH_ADDR (0x2C >> 1)
#define ME_SENSOR_NUM_TEMP_PCH 0x08
#define PVCCD_HV_ADDR (0xC4 >> 1)
#define PVCCINFAON_ADDR (0xEC >> 1)
#define PVCCFA_EHV_ADDR (0xEC >> 1)
#define PVCCIN_ADDR (0xC0 >> 1)
#define PVCCFA_EHV_FIVRA_ADDR (0xC0 >> 1)
#define VR_VOL_CMD 0x8B
#define VR_CUR_CMD 0x8C
#define VR_TEMP_CMD 0x8D
#define VR_PWR_CMD 0x96
#define SSD0_MUX_ADDR (0xE2 >> 1)
#define SSD0_CHANNEL 2
#define CPU_PECI_ADDR 0x30
#define TEMP_CPU_MARGIN_INDEX 0x02
#define TEMP_CPU_MARGIN_PARAM 0x00FF
#define TEMP_CPU_TJMAX_INDEX 0x10
#define TEMP_CPU_TJMAX_PARAM 0x0000
#define TEMP_DIMM_INDEX 0x0E
#define TEMP_DIMM_A0_PARAM 0x0000
#define TEMP_DIMM_A2_PARAM 0x0002
#define TEMP_DIMM_A3_PARAM 0x0003
#define TEMP_DIMM_A4_PARAM 0x0004
#define TEMP_DIMM_A6_PARAM 0x0006
#define TEMP_DIMM_A7_PARAM 0x0007
#define DPV2_16_ADDR 0x50

/*  threshold sensor number, 1 based  */
#define SENSOR_NUM_TEMP_TMP75_IN 0x01
#define SENSOR_NUM_TEMP_TMP75_OUT 0x02
#define SENSOR_NUM_TEMP_TMP75_FIO 0x03
#define SENSOR_NUM_TEMP_PCH 0x04
#define SENSOR_NUM_TEMP_CPU 0x05
#define SENSOR_NUM_TEMP_DIMM_A0 0x06
#define SENSOR_NUM_TEMP_DIMM_A2 0x07
#define SENSOR_NUM_TEMP_DIMM_A3 0x09
#define SENSOR_NUM_TEMP_DIMM_A4 0x0A
#define SENSOR_NUM_TEMP_DIMM_A6 0x0B
#define SENSOR_NUM_TEMP_DIMM_A7 0x0C
#define SENSOR_NUM_TEMP_SSD0 0x0D
#define SENSOR_NUM_TEMP_HSC 0x0E
#define SENSOR_NUM_TEMP_CPU_MARGIN 0x14
#define SENSOR_NUM_TEMP_CPU_TJMAX 0x15
#define SENSOR_NUM_TEMP_PVCCIN 0x0F
#define SENSOR_NUM_TEMP_PVCCFA_EHV_FIVRA 0x10
#define SENSOR_NUM_TEMP_PVCCFA_EHV 0x11
#define SENSOR_NUM_TEMP_PVCCD_HV 0x12
#define SENSOR_NUM_TEMP_PVCCINFAON 0x13

#define SENSOR_NUM_VOL_STBY12V 0x20
#define SENSOR_NUM_VOL_BAT3V 0x21
#define SENSOR_NUM_VOL_STBY3V 0x22
#define SENSOR_NUM_VOL_STBY1V05 0x24
#define SENSOR_NUM_VOL_STBY1V8 0x23
#define SENSOR_NUM_VOL_STBY5V 0x25
#define SENSOR_NUM_VOL_DIMM12V 0x26
#define SENSOR_NUM_VOL_STBY1V2 0x27
#define SENSOR_NUM_VOL_M2_3V3 0x28
#define SENSOR_NUM_VOL_HSCIN 0x29
#define SENSOR_NUM_VOL_PVCCIN 0x2A
#define SENSOR_NUM_VOL_PVCCFA_EHV_FIVRA 0x2C
#define SENSOR_NUM_VOL_PVCCFA_EHV 0x2D
#define SENSOR_NUM_VOL_PVCCD_HV 0x2E
#define SENSOR_NUM_VOL_PVCCINFAON 0x2F

#define SENSOR_NUM_CUR_HSCOUT 0x30
#define SENSOR_NUM_CUR_PVCCIN 0x31
#define SENSOR_NUM_CUR_PVCCFA_EHV_FIVRA 0x32
#define SENSOR_NUM_CUR_PVCCFA_EHV 0x33
#define SENSOR_NUM_CUR_PVCCD_HV 0x34
#define SENSOR_NUM_CUR_PVCCINFAON 0x35

#define SENSOR_NUM_PWR_CPU 0x38
#define SENSOR_NUM_PWR_HSCIN 0x39
#define SENSOR_NUM_PWR_PVCCIN 0x3A
#define SENSOR_NUM_PWR_PVCCFA_EHV_FIVRA 0x3C
#define SENSOR_NUM_PWR_PVCCFA_EHV 0x3D
#define SENSOR_NUM_PWR_PVCCD_HV 0x3E
#define SENSOR_NUM_PWR_PVCCINFAON 0x3F
#define SENSOR_NUM_PWR_DIMMA0_PMIC 0x1E
#define SENSOR_NUM_PWR_DIMMA2_PMIC 0x1F
#define SENSOR_NUM_PWR_DIMMA3_PMIC 0x36
#define SENSOR_NUM_PWR_DIMMA4_PMIC 0x37
#define SENSOR_NUM_PWR_DIMMA6_PMIC 0x42
#define SENSOR_NUM_PWR_DIMMA7_PMIC 0x47

#define SENSOR_NUM_SYSTEM_STATUS 0x10
#define SENSOR_NUM_POWER_ERROR 0x56
#define SENSOR_NUM_PROC_FAIL 0x65
#define SENSOR_NUM_VR_HOT 0xB2
#define SENSOR_NUM_CPUDIMM_HOT 0xB3
#define SENSOR_NUM_PMIC_ERROR 0xB4
#define SENSOR_NUM_CATERR 0xEB
#define SENSOR_NUM_NMI 0xEA

/*  threshold sensor number, DPV2  */
#define SENSOR_NUM_VOL_DPV2_12VIN 0x91
#define SENSOR_NUM_VOL_DPV2_12VOUT 0x92
#define SENSOR_NUM_CUR_DPV2OUT 0x93
#define SENSOR_NUM_TEMP_DPV2_EFUSE 0x94
#define SENSOR_NUM_PWR_DPV2 0x95

#define POLL_TIME_BAT3V 3600 // sec

typedef struct _dimm_pmic_mapping_cfg {
	uint8_t dimm_sensor_num;
	uint8_t mapping_pmic_sensor_num;
} dimm_pmic_mapping_cfg;

uint8_t plat_get_config_size();
uint8_t pal_get_extend_sensor_config();
void load_sensor_config(void);
bool disable_dimm_pmic_sensor(uint8_t sensor_num);
uint8_t get_dimm_status(uint8_t dimm_index);

#endif
