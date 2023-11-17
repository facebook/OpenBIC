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

#ifndef PLAT_PLDM_SENSOR_H
#define PLAT_PLDM_SENSOR_H

#include "pdr.h"

#define ADDR_TMP75_INLET (0x92 >> 1)
#define ADDR_TMP461AIRUNR (0x98 >> 1)
#define ADDR_INA233 (0x8A >> 1)
#define ADDR_VR_PVDDQ_AB (0xEC >> 1)
#define ADDR_VR_P0V85_ASIC (0xEC >> 1)
#define ADDR_VR_P0V8_ASIC (0xE4 >> 1)
#define ADDR_VR_PVDDQ_CD (0xE4 >> 1)
#define ADDR_DIMM_A_CH0 0x50
#define ADDR_DIMM_B_CH0 0x51
#define ADDR_DIMM_A_CH1 0x52
#define ADDR_DIMM_B_CH1 0x53

#define OFFSET_TMP75_TEMP 0x00
#define OFFSET_TMP461_TEMP 0x00
#define OFFSET_INA233_VOL 0x8B
#define OFFSET_INA233_CURR 0x8C
#define OFFSET_INA233_PWR 0x96

#define UPDATE_INTERVAL_1S 1

enum SENSOR_THREAD_LIST {
	TMP_SENSOR_THREAD_ID = 0,
	ADC_SENSOR_THREAD_ID,
	INA233_SENSOR_THREAD_ID,
	VR_SENSOR_THREAD_ID,
	DIMM_SENSOR_THREAD_ID,
	MAX_SENSOR_THREAD_ID,
};

int plat_pldm_sensor_get_sensor_count(int thread_id);
void plat_pldm_sensor_get_pdr_numeric_sensor(int thread_id, int sensor_num,
					     PDR_numeric_sensor *numeric_sensor_table);

#endif
