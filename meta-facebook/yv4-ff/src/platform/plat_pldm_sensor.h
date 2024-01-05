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

#define OFFSET_TMP75_TEMP 0x00
#define OFFSET_TMP461_TEMP 0x00

#define UPDATE_INTERVAL_1S 1
#define UPDATE_INTERVAL_3S 3

#define CXL_ID 0

#define SENSOR_ID_CH0_DIMM_A_TEMP 0x0007
#define SENSOR_ID_CH0_DIMM_B_TEMP 0x0008
#define SENSOR_ID_CH1_DIMM_C_TEMP 0x0009
#define SENSOR_ID_CH1_DIMM_D_TEMP 0x000A

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
