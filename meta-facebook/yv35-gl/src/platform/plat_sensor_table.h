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

#define INLET_TEP75_ADDR (0x92 >> 1)
#define OUTLET_TEP75_ADDR (0x98 >> 1)
#define FIO_ADDR (0x90 >> 1)
#define HSC_ADM1278_ADDR (0x22 >> 1)
#define HSC_MP5990_ADDR (0x16 >> 1)
#define PVCCIN_ADDR (0xC0 >> 1)
#define EHV_ADDR (0xC0 >> 1)
#define FIVRA_ADDR (0xDC >> 1)
#define INF_ADDR (0xDC >> 1)
#define PVCCD0_ADDR (0xD4 >> 1)
#define PVCCD1_ADDR (0xD4 >> 1)
#define EVT_FIVRA_ADDR (0xC4 >> 1)
#define EVT_PVCCD0_ADDR (0xE4 >> 1)
#define CPU_PECI_ADDR 0x30
#define SSD0_ADDR (0xD4 >> 1)
#define SSD0_OFFSET 0x00

#define TMP75_TEMP_OFFSET 0x0
#define VR_TEMP_OFFSET 0x8D
#define VR_VOL_OFFSET 0x8B
#define VR_CUR_OFFSET 0x8C
#define VR_PWR_OFFSET 0x96

#define POLL_TIME_BAT3V 3600 // sec

// Threshold sensor number definition
#define SENSOR_NUM_MB_INLET_TEMP_C 0x1
#define SENSOR_NUM_MB_OUTLET_TEMP_C 0x2
#define SENSOR_NUM_FIO_FRONT_TEMP_C 0x3
#define SENSOR_NUM_MB_SOC_CPU_TEMP_C 0x4
#define SENSOR_NUM_MB_DIMMA0_TEMP_C 0x5
#define SENSOR_NUM_MB_DIMMA1_TEMP_C 0x6
#define SENSOR_NUM_MB_DIMMA2_TEMP_C 0x7
#define SENSOR_NUM_MB_DIMMA3_TEMP_C 0x8
#define SENSOR_NUM_MB_DIMMA4_TEMP_C 0x9
#define SENSOR_NUM_MB_DIMMA5_TEMP_C 0xA
#define SENSOR_NUM_MB_DIMMA6_TEMP_C 0xB
#define SENSOR_NUM_MB_DIMMA7_TEMP_C 0xC
#define SENSOR_NUM_MB_SSD0_TEMP_C 0xD
#define SENSOR_NUM_MB_HSC_TEMP_C 0xE
#define SENSOR_NUM_MB_VR_VCCIN_TEMP_C 0xF
#define SENSOR_NUM_MB_VR_EHV_TEMP_C 0x10
#define SENSOR_NUM_MB_VR_FIVRA_TEMP_C 0x11
#define SENSOR_NUM_MB_VR_VCCINF_TEMP_C 0x12
#define SENSOR_NUM_MB_VR_VCCD0_TEMP_C 0x13
#define SENSOR_NUM_MB_VR_VCCD1_TEMP_C 0x14
#define SENSOR_NUM_MB_SOC_THERMAL_MARGIN_C 0x15
#define SENSOR_NUM_MB_SOC_TJMAX_C 0x16
#define SENSOR_NUM_MB_ADC_P12V_STBY_VOLT_V 0x17
#define SENSOR_NUM_MB_ADC_P3V_BAT_VOLT_V 0x18
#define SENSOR_NUM_MB_ADC_P3V3_STBY_VOLT_V 0x19
#define SENSOR_NUM_MB_ADC_P1V8_STBY_VOLT_V 0x1A
#define SENSOR_NUM_MB_ADC_P1V05_STBY_VOLT_V 0x1B
#define SENSOR_NUM_MB_HSC_INPUT_VOLT_V 0x1C
#define SENSOR_NUM_MB_VR_VCCIN_VOLT_V 0x1D
#define SENSOR_NUM_MB_VR_VCCINF_VOLT_V 0x1E
#define SENSOR_NUM_MB_VR_FIVRA_VOLT_V 0x1F
#define SENSOR_NUM_MB_VR_VCCD0_VOLT_V 0x20
#define SENSOR_NUM_MB_VR_VCCD1_VOLT_V 0x21
#define SENSOR_NUM_MB_VR_EHV_VOLT_V 0x22
#define SENSOR_NUM_MB_ADC_VNN_VOLT_V 0x23
#define SENSOR_NUM_MB_ADC_P5V_STBY_VOLT_V 0x24
#define SENSOR_NUM_MB_ADC_P12V_DIMM_VOLT_V 0x25
#define SENSOR_NUM_MB_ADC_P1V2_STBY_VOLT_V 0x26
#define SENSOR_NUM_MB_ADC_P3V3_M2_VOLT_V 0x27
#define SENSOR_NUM_MB_HSC_OUTPUT_CURR_A 0x28
#define SENSOR_NUM_MB_VR_VCCIN_CURR_A 0x29
#define SENSOR_NUM_MB_VR_EHV_CURR_A 0x2A
#define SENSOR_NUM_MB_VR_FIVRA_CURR_A 0x2B
#define SENSOR_NUM_MB_VR_VCCINF_CURR_A 0x2C
#define SENSOR_NUM_MB_VR_VCCD0_CURR_A 0x2D
#define SENSOR_NUM_MB_VR_VCCD1_CURR_A 0x2E
#define SENSOR_NUM_MB_HSC_INPUT_PWR_W 0x2F
#define SENSOR_NUM_MB_VR_VCCIN_PWR_W 0x30
#define SENSOR_NUM_MB_VR_EHV_PWR_W 0x31
#define SENSOR_NUM_MB_VR_FIVRA_PWR_W 0x32
#define SENSOR_NUM_MB_VR_VCCINF_PWR_W 0x33
#define SENSOR_NUM_MB_VR_VCCD0_PWR_W 0x34
#define SENSOR_NUM_MB_VR_VCCD1_PWR_W 0x35
#define SENSOR_NUM_MB_VR_DIMMA0_PMIC_PWR_W 0x36
#define SENSOR_NUM_MB_VR_DIMMA1_PMIC_PWR_W 0x37
#define SENSOR_NUM_MB_VR_DIMMA2_PMIC_PWR_W 0x38
#define SENSOR_NUM_MB_VR_DIMMA3_PMIC_PWR_W 0x39
#define SENSOR_NUM_MB_VR_DIMMA4_PMIC_PWR_W 0x3A
#define SENSOR_NUM_MB_VR_DIMMA5_PMIC_PWR_W 0x3B
#define SENSOR_NUM_MB_VR_DIMMA6_PMIC_PWR_W 0x3C
#define SENSOR_NUM_MB_VR_DIMMA7_PMIC_PWR_W 0x3D
#define SENSOR_NUM_MB_SOC_CPU_PWR_W 0x3E
#define SENSOR_NUM_MB_TOTAL_DIMM_PWR_W 0x3F

// Event-only sensor number definition
#define SENSOR_NUM_SYSTEM_STATUS 0x10
#define SENSOR_NUM_END_OF_POST 0x11
#define SENSOR_NUM_CPU0_THERM_STATUS 0x1C
#define SENSOR_NUM_POWER_ERROR 0x56
#define SENSOR_NUM_PROC_FAIL 0x65
#define SENSOR_NUM_VR_HOT 0xB2
#define SENSOR_NUM_CPUDIMM_HOT 0xB3
#define SENSOR_NUM_PMIC_ERROR 0xB4
#define SENSOR_NUM_CATERR 0xEB

typedef struct _dimm_pmic_mapping_cfg {
	uint8_t dimm_sensor_num;
	uint8_t mapping_pmic_sensor_num;
} dimm_pmic_mapping_cfg;

uint8_t get_dimm_status(uint8_t dimm_index);

#endif
