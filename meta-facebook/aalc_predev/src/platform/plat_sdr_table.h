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

#ifndef PLAT_SDR_TABLE_H
#define PLAT_SDR_TABLE_H

#include <stdint.h>

#define MAX_SENSOR_SIZE 60

/* ----- E1S ----- */
#define INA231_POWER_SEN_FACTOR_RB 0xF0
#define INA231_POWER_UCT 0x96

#define INA231_VOLTAGE_UCT 0x2F
#define INA231_VOLTAGE_LCT 0x25

#define ADC_12V_SEN_FACTOR_M 0x3E
#define ADC_12V_SEN_FACTOR_RB 0xD0
#define ADC_12V_UCT 0xD4
#define ADC_12V_LCT 0xAF

#define ADC_3V3_SEN_FACTOR_M 0x11
#define ADC_3V3_SEN_FACTOR_RB 0xD0
#define ADC_3V3_UCT 0xCD
#define ADC_3V3_LCT 0xB7

#define INA231_VOL_SEN_FACTOR_M ADC_12V_SEN_FACTOR_M
#define INA231_VOL_SEN_FACTOR_RB ADC_12V_SEN_FACTOR_RB
#define INA231_VOL_UCT ADC_12V_UCT
#define INA231_VOL_LCT ADC_12V_LCT

#define HSC_VIN_SEN_FACTOR_M 0x06
#define HSC_VIN_SEN_FACTOR_EXP_RB 0xE0

#define HSC_IOUT_SEN_FACTOR_M 0x05
#define HSC_IOUT_SEN_FACTOR_EXP_RB 0xE0

#define HSC_PIN_SEN_FACTOR_M 0x06
#define HSC_PIN_SEN_FACTOR_EXP_RB 0xF0

#define HSC_T_SEN_FACTOR_M 0x01
#define HSC_T_SEN_FACTOR_EXP_RB 0x00

uint8_t plat_get_sdr_size();
void load_sdr_table(void);

#endif
