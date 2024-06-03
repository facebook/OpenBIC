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
#include "plat_def.h"

/* SENSOR POLLING TIME(second) */
#define POLL_TIME_BAT3V 3600

/* SENSOR ADDRESS(7-bit)/OFFSET */
#define E1S_SSD_ADDR (0xD4 >> 1)
#define FPGA_ADDR (0xC0 >> 1)
#define TEMP_HSC_ADDR (0x98 >> 1)
#define MP5990_ADDR (0xA0 >> 1)
#define MP5990_ADDR_1 (0xC0 >> 1) // for board >= EVT2
#define RS31380R_ADDR (0xC0 >> 1)
#define INA230_ADDR (0x8A >> 1)
#define INA3221_ADDR (0x80 >> 1)
#define TMP451_ADDR (0x98 >> 1)
#define TMP75_ADDR (0x90 >> 1)
#define AL_RETIMER_ADDR (0x40 >> 1)
#define TI_RETIMER_ADDR (0x20 >> 1)

/* SENSOR OFFSET */
#define SSD_TEMP_OFFSET 0x00
#define TMP75_TEMP_OFFSET 0x00

/*  threshold sensor number, 1 based - temperature */
#define SENSOR_NUM_TEMP_TMP451_IN 0x1
#define SENSOR_NUM_TEMP_TMP451_OUT 0x2
#define SENSOR_NUM_TEMP_TMP75_FIO 0x3
#define SENSOR_NUM_TEMP_CPU 0x4
#define SENSOR_NUM_TEMP_FPGA 0x5
#define SENSOR_NUM_TEMP_E1S_SSD 0x6
#define SENSOR_NUM_TEMP_HSC 0x7
#define SENSOR_NUM_TEMP_CPUDVDD 0x8
#define SENSOR_NUM_TEMP_CPUVDD 0x9
#define SENSOR_NUM_TEMP_SOCVDD 0xA
#define SENSOR_NUM_TEMP_RETIMER 0xB
#define SENSOR_NUM_TEMP_LPDDR5_UP 0xC
#define SENSOR_NUM_TEMP_LPDDR5_DOWN 0xD

/* SENSOR NUMBER(1 based) - voltage */
#define SENSOR_NUM_VOL_HSCIN 0x10
#define SENSOR_NUM_VOL_ADC0_P12V_STBY 0x11
#define SENSOR_NUM_VOL_ADC1_VDD_1V8 0x12
#define SENSOR_NUM_VOL_ADC2_P3V3_STBY 0x13
#define SENSOR_NUM_VOL_ADC3_SOCVDD 0x14
#define SENSOR_NUM_VOL_ADC4_P3V_BAT 0x15
#define SENSOR_NUM_VOL_ADC5_CPUVDD 0x16
#define SENSOR_NUM_VOL_ADC6_FPGA_VCC_AO 0x17
#define SENSOR_NUM_VOL_ADC7_1V2 0x18
#define SENSOR_NUM_VOL_ADC9_VDD_3V3 0x19
#define SENSOR_NUM_VOL_ADC10_P1V2_STBY 0x1A
#define SENSOR_NUM_VOL_ADC11_FBVDDQ 0x1B
#define SENSOR_NUM_VOL_ADC12_FBVDDP2 0x1C
#define SENSOR_NUM_VOL_ADC13_FBVDD1 0x1D
#define SENSOR_NUM_VOL_ADC14_P5V_STBY 0x1E
#define SENSOR_NUM_VOL_ADC15_CPU_DVDD 0x1F
#define SENSOR_NUM_VOL_E1S 0x20
#define SENSOR_NUM_VOL_CPUDVDD 0x21
#define SENSOR_NUM_VOL_CPUVDD 0x22
#define SENSOR_NUM_VOL_SOCVDD 0x23

/* SENSOR NUMBER(1 based) - current */
#define SENSOR_NUM_CUR_HSCOUT 0x25
#define SENSOR_NUM_CUR_E1S 0x26
#define SENSOR_NUM_CUR_CPUDVDD 0x27
#define SENSOR_NUM_CUR_CPUVDD 0x28
#define SENSOR_NUM_CUR_SOCVDD 0x29

/* SENSOR NUMBER(1 based) - power */
#define SENSOR_NUM_PWR_CPU 0x30
#define SENSOR_NUM_PWR_HSCIN 0x31
#define SENSOR_NUM_PWR_E1S 0x32
#define SENSOR_NUM_PWR_CPUDVDD 0x33
#define SENSOR_NUM_PWR_CPUVDD 0x34
#define SENSOR_NUM_PWR_SOCVDD 0x35

/* SENSOR NUMBER(1 based) - state */
#define SENSOR_NUM_OTH_CPU_THROTTLE 0x40
#define SENSOR_NUM_OTH_POWER_BREAK 0x41
#define SENSOR_NUM_OTH_SPARE_CHANNEL 0x42

/* SENSOR NUMBER - sel */
#define SENSOR_NUM_SYSTEM_STATUS 0x10
#define SENSOR_NUM_POWER_ERROR 0x56
#define SENSOR_NUM_PROC_FAIL 0x65
#define SENSOR_NUM_CPU_FAULT 0xC0

#define IPMI_EVENT_OFFSET_SYS_E1S_ALERT 0x86

uint8_t plat_get_config_size();
void load_sensor_config(void);
bool modify_sensor_cfg();

#endif
