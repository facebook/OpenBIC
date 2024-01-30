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
#include <pmbus.h>

/* define sensors address(7 bit) */
#define BB_TMP75_ADDR (0x9E >> 1)
#define BPB_TMP75_ADDR (0x90 >> 1)
#define BB_ADM1272_ADDR (0x26 >> 1)
#define BPB_ADM1272_ADDR (0x20 >> 1)
#define BPB_ADS112C_1_ADDR (0x88 >> 1)
#define BPB_ADS112C_2_ADDR (0x8A >> 1)
#define BPB_ADS112C_3_ADDR (0x82 >> 1)
#define BPB_ADS112C_4_ADDR (0x80 >> 1)
#define BPB_NCT7363_ADDR (0x40 >> 1)
#define FB_NCT7363_ADDR (0x42 >> 1)
#define PB_NCT7363_ADDR (0x40 >> 1)
#define HDC1080_ADDR (0x80 >> 1)
#define SB_TMP461_1_ADDR (0x90 >> 1)
#define SB_TMP461_2_ADDR (0x92 >> 1)
#define SB_TMP461_3_ADDR (0x94 >> 1)
#define SB_TMP461_4_ADDR (0x96 >> 1)

#define ADS112C_MUX_1_CON 0x80 //AINP = AIN0, AINN = AVSS
#define ADS112C_MUX_2_CON 0x90 //AINP = AIN1, AINN = AVSS
#define ADS112C_MUX_3_CON 0xA0 //AINP = AIN2, AINN = AVSS
#define ADS112C_MUX_4_CON 0xA1 //AINP = AIN3, AINN = AVSS


/* define sensors offset */
#define TMP75_TEMP_OFFSET 0x00

/*  threshold sensor number, 1 based  */
#define SENSOR_NUM_TEMP_BB_TMP75 0x01
#define SENSOR_NUM_TEMP_BPB_TMP75 0x02
#define SENSOR_NUM_TEMP_BPB_ADS_1 0x03
#define SENSOR_NUM_TEMP_BPB_ADS_2 0x04
#define SENSOR_NUM_TEMP_BPB_ADS_3 0x05
#define SENSOR_NUM_TEMP_MB 0x06
#define SENSOR_NUM_TEMP_PDB 0x07
#define SENSOR_NUM_TEMP_PB 0x08
#define SENSOR_NUM_TEMP_FB_BUS1 0x09
#define SENSOR_NUM_TEMP_FB_BUS2 0x0A
#define SENSOR_NUM_TEMP_FB_BUS6 0x0B
#define SENSOR_NUM_TEMP_FB_BUS7 0x0C
#define SENSOR_NUM_TEMP_SB_1 0x0D
#define SENSOR_NUM_TEMP_SB_2 0x0E
#define SENSOR_NUM_TEMP_SB_3 0x0F
#define SENSOR_NUM_TEMP_SB_4 0x10
#define SENSOR_NUM_PRESS_BPB_ADS_1 0x11
#define SENSOR_NUM_PRESS_BPB_ADS_2 0x12
#define SENSOR_NUM_PRESS_BPB_ADS_3 0x13
#define SENSOR_NUM_PRESS_BPB_ADS_4 0x14
#define SENSOR_NUM_FLOW_BPB_ADS 0x15
#define SENSOR_NUM_LEAK_BPB_ADS_1 0x16
#define SENSOR_NUM_LEAK_BPB_ADS_2 0x17
#define SENSOR_NUM_LEAK_BPB_ADS_3 0x18
#define SENSOR_NUM_FAN_BPB_1 0x19
#define SENSOR_NUM_FAN_BPB_2 0x1A
#define SENSOR_NUM_FAN_BPB_3 0x1B
#define SENSOR_NUM_FAN_FB_BUS1 0x1C
#define SENSOR_NUM_FAN_FB_BUS2 0x1D
#define SENSOR_NUM_FAN_FB_BUS6 0x1E
#define SENSOR_NUM_FAN_FB_BUS7 0x1F
#define SENSOR_NUM_FAN_PB_1 0x20
#define SENSOR_NUM_FAN_PB_2 0x21
#define SENSOR_NUM_FAN_PB_3 0x22
#define SENSOR_NUM_HUM_MB 0x23
#define SENSOR_NUM_HUM_PDB 0x24
#define SENSOR_NUM_HUM_PB 0x25
#define SENSOR_NUM_HUM_FB_BUS1 0x26
#define SENSOR_NUM_HUM_FB_BUS2 0x27
#define SENSOR_NUM_HUM_FB_BUS6 0x28
#define SENSOR_NUM_HUM_FB_BUS7 0x29

/* HSC Temp sensor number */
#define SENSOR_NUM_TEMP_BB_HSC 0x2A
#define SENSOR_NUM_TEMP_BPB_HSC 0x2B

/* HSC Voltage sensor number */
#define SENSOR_NUM_VOL_BB_P51V_STBY 0x2C
#define SENSOR_NUM_VOL_BB_P51V_AUX 0x2D
#define SENSOR_NUM_VOL_BPB_P51V_STBY 0x2E
#define SENSOR_NUM_VOL_BPB_P51V_AUX 0x2F

/* HSC Current sensor number */
#define SENSOR_NUM_CUR_BB_P51V_STBY 0x30
#define SENSOR_NUM_CUR_BB_P51V_AUX 0x31
#define SENSOR_NUM_CUR_BPB_P51V_STBY 0x32
#define SENSOR_NUM_CUR_BPB_P51V_AUX 0x33

/* HSC Power sensor number */
#define SENSOR_NUM_PWR_BB_P51V_STBY 0x34
#define SENSOR_NUM_PWR_BB_P51V_AUX 0x35
#define SENSOR_NUM_PWR_BPB_P51V_STBY 0x36
#define SENSOR_NUM_PWR_BPB_P51V_AUX 0x37



uint8_t plat_get_config_size();
void load_sensor_config(void);
#endif
