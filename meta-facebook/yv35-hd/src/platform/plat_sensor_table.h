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

/* define sensors address(7 bit) */
#define TMP75_IN_ADDR (0x94 >> 1)
#define TMP75_OUT_ADDR (0x92 >> 1)
#define TMP75_FIO_ADDR (0x90 >> 1)
#define SSD_ADDR (0xD4 >> 1)
#define APML_ADDR 0x3C
#define TSI_ADDR 0x4C

#define RAA229621_PVDDCR_CPU0_ADDR 0x61
#define RAA229621_PVDDCR_SOC_ADDR 0x61
#define RAA229621_PVDDCR_CPU1_ADDR 0x62
#define RAA229621_PVDDIO_ADDR 0x62
#define RAA229621_PVDD11_S3_ADDR 0x63

#define XDPE19283B_PVDDCR_CPU0_ADDR 0x64
#define XDPE19283B_PVDDCR_SOC_ADDR 0x64
#define XDPE19283B_PVDDCR_CPU1_ADDR 0x66
#define XDPE19283B_PVDDIO_ADDR 0x66
#define XDPE19283B_PVDD11_S3_ADDR 0x68

#define MP2856GUT_PVDDCR_CPU0_ADDR 0x4F
#define MP2856GUT_PVDDCR_SOC_ADDR 0x4F
#define MP2856GUT_PVDDCR_CPU1_ADDR 0x4E
#define MP2856GUT_PVDDIO_ADDR 0x4E
#define MP2856GUT_PVDD11_S3_ADDR 0x4B

#define ADM1278_ADDR (0x80 >> 1)
#define LTC4282_ADDR (0x82 >> 1)
#define TEMP_HSC_ADDR (0x98 >> 1)

/* define sensors offset */
#define TMP75_TEMP_OFFSET 0x00
#define SSD_TEMP_OFFSET 0x00
#define CPU_PWR_OFFSET 0x01

/*  threshold sensor number, 1 based  */
#define SENSOR_NUM_TEMP_TMP75_IN 0x1
#define SENSOR_NUM_TEMP_TMP75_OUT 0x2
#define SENSOR_NUM_TEMP_TMP75_FIO 0x3
#define SENSOR_NUM_TEMP_CPU 0x4
#define SENSOR_NUM_TEMP_DIMM_A0 0x5
#define SENSOR_NUM_TEMP_DIMM_A1 0x6
#define SENSOR_NUM_TEMP_DIMM_A2 0x7
#define SENSOR_NUM_TEMP_DIMM_A4 0x8
#define SENSOR_NUM_TEMP_DIMM_A6 0x9
#define SENSOR_NUM_TEMP_DIMM_A7 0xA
#define SENSOR_NUM_TEMP_DIMM_A8 0xB
#define SENSOR_NUM_TEMP_DIMM_A10 0xC
#define SENSOR_NUM_TEMP_SSD 0xD
#define SENSOR_NUM_TEMP_HSC 0xE
#define SENSOR_NUM_TEMP_PVDDCR_CPU0_VR 0xF
#define SENSOR_NUM_TEMP_PVDDCR_SOC_VR 0x10
#define SENSOR_NUM_TEMP_PVDDCR_CPU1_VR 0x11
#define SENSOR_NUM_TEMP_PVDDIO_VR 0x12
#define SENSOR_NUM_TEMP_PVDD11_S3_VR 0x13
#define SENSOR_NUM_VOL_P12V_STBY 0x14
#define SENSOR_NUM_VOL_PVDD18_S5 0x15
#define SENSOR_NUM_VOL_P3V3_STBY 0x16
#define SENSOR_NUM_VOL_PVDD11_S3 0x17
#define SENSOR_NUM_VOL_P3V_BAT 0x18
#define SENSOR_NUM_VOL_PVDD33_S5 0x19
#define SENSOR_NUM_VOL_P5V_STBY 0x1A
#define SENSOR_NUM_VOL_P12V_MEM_1 0x1B
#define SENSOR_NUM_VOL_P12V_MEM_0 0x1C
#define SENSOR_NUM_VOL_P1V2_STBY 0x1D
#define SENSOR_NUM_VOL_P3V3_M2 0x1E
#define SENSOR_NUM_VOL_P1V8_STBY 0x1F
#define SENSOR_NUM_VOL_HSCIN 0x20
#define SENSOR_NUM_VOL_PVDDCR_CPU0_VR 0x21
#define SENSOR_NUM_VOL_PVDDCR_SOC_VR 0x22
#define SENSOR_NUM_VOL_PVDDCR_CPU1_VR 0x23
#define SENSOR_NUM_VOL_PVDDIO_VR 0x24
#define SENSOR_NUM_VOL_PVDD11_S3_VR 0x25
#define SENSOR_NUM_CUR_HSCOUT 0x26
#define SENSOR_NUM_CUR_PVDDCR_CPU0_VR 0x27
#define SENSOR_NUM_CUR_PVDDCR_SOC_VR 0x28
#define SENSOR_NUM_CUR_PVDDCR_CPU1_VR 0x29
#define SENSOR_NUM_CUR_PVDDIO_VR 0x2A
#define SENSOR_NUM_CUR_PVDD11_S3_VR 0x2B
#define SENSOR_NUM_PWR_HSCIN 0x2C
#define SENSOR_NUM_PWR_PVDDCR_CPU0_VR 0x2D
#define SENSOR_NUM_PWR_PVDDCR_SOC_VR 0x2E
#define SENSOR_NUM_PWR_PVDDCR_CPU1_VR 0x2F
#define SENSOR_NUM_PWR_PVDDIO_VR 0x30
#define SENSOR_NUM_PWR_PVDD11_S3_VR 0x31
#define SENSOR_NUM_PWR_CPU 0x32
#define SENSOR_NUM_PWR_DIMM_A0 0x33
#define SENSOR_NUM_PWR_DIMM_A1 0x34
#define SENSOR_NUM_PWR_DIMM_A2 0x35
#define SENSOR_NUM_PWR_DIMM_A4 0x36
#define SENSOR_NUM_PWR_DIMM_A6 0x37
#define SENSOR_NUM_PWR_DIMM_A7 0x38
#define SENSOR_NUM_PWR_DIMM_A8 0x39
#define SENSOR_NUM_PWR_DIMM_A10 0x3A

#define POLL_TIME_BAT3V 3600 // second

#define SENSOR_NUM_SYSTEM_STATUS 0x10
#define SENSOR_NUM_POWER_ERROR 0x56
#define SENSOR_NUM_PROC_FAIL 0x65
#define SENSOR_NUM_VR_OCP 0x71
#define SENSOR_NUM_VR_ALERT 0x72
#define SENSOR_NUM_HDT_PRESENT 0xBD
#define SENSOR_NUM_PMIC_ERROR 0xB4

uint8_t plat_get_config_size();
void load_sensor_config(void);

#endif
