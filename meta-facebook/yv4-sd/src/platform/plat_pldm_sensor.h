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
#define ADDR_TMP75_OUTLET (0x98 >> 1)
#define ADDR_TMP75_FIO (0x90 >> 1)
#define ADDR_VR_CPU0 (0XEC >> 1)
#define ADDR_VR_SOC (0XEC >> 1)
#define ADDR_VR_CPU1 (0XC6 >> 1)
#define ADDR_VR_PVDDIO (0XC6 >> 1)
#define ADDR_VR_PVDD11 (0XE4 >> 1)
#define ADDR_X8_INA233 (0x8A >> 1)
#define ADDR_X16_INA233 (0x82 >> 1)
#define ADDR_NVME (0xD4 >> 1)

#define OFFSET_TMP75_TEMP 0x00
#define OFFSET_NVME_TEMP 0x00

#define UPDATE_INTERVAL_1S 1
#define UPDATE_INTERVAL_3S 3

enum SENSOR_THREAD_LIST {
	ADC_SENSOR_THREAD_ID = 0,
	VR_SENSOR_THREAD_ID,
	MB_TEMP_SENSOR_THREAD_ID,
	CPU_SENSOR_THREAD_ID,
	INA233_SENSOR_THREAD_ID,
	MAX_SENSOR_THREAD_ID,
};

int plat_pldm_sensor_get_sensor_count(int thread_id);
void plat_pldm_sensor_get_pdr_numeric_sensor(int thread_id, int sensor_num,
					     PDR_numeric_sensor *numeric_sensor_table);

#endif
