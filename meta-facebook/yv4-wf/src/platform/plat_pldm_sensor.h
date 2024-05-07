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
#define ADDR_TMP461_CXL1 (0x98 >> 1)
#define ADDR_TMP461_CXL2 (0x90 >> 1)
#define ADDR_INA233_P12V_STBY (0x80 >> 1)
#define ADDR_INA233_E1S (0x8A >> 1)
#define ADDR_VR_P0V85_ASIC1 (0xC4 >> 1)
#define ADDR_VR_P0V8_ASIC1 (0xB4 >> 1)
#define ADDR_VR_PVDDQ_AB_ASIC1 (0xC4 >> 1)
#define ADDR_VR_PVDDQ_CD_ASIC1 (0xB4 >> 1)
#define ADDR_VR_P0V85_ASIC2 (0xEC >> 1)
#define ADDR_VR_P0V8_ASIC2 (0xE4 >> 1)
#define ADDR_VR_PVDDQ_AB_ASIC2 (0xEC >> 1)
#define ADDR_VR_PVDDQ_CD_ASIC2 (0xE4 >> 1)
#define ADDR_NVME (0xD4 >> 1)
#define ADDR_MAX11617 (0x6A >> 1)

#define OFFSET_TMP75_TEMP 0x00
#define OFFSET_TMP461_TEMP 0x00
#define OFFSET_NVME_TEMP 0x00

#define SENSOR_ID_ASIC1_DIMM_A_TEMP 0x000C
#define SENSOR_ID_ASIC1_DIMM_B_TEMP 0x000D
#define SENSOR_ID_ASIC1_DIMM_C_TEMP 0x000E
#define SENSOR_ID_ASIC1_DIMM_D_TEMP 0x000F
#define SENSOR_ID_ASIC2_DIMM_A_TEMP 0x0010
#define SENSOR_ID_ASIC2_DIMM_B_TEMP 0x0011
#define SENSOR_ID_ASIC2_DIMM_C_TEMP 0x0012
#define SENSOR_ID_ASIC2_DIMM_D_TEMP 0x0013

#define UPDATE_INTERVAL_1S 1
#define UPDATE_INTERVAL_3S 3

enum SENSOR_THREAD_LIST {
	ADC_SENSOR_THREAD_ID = 0,
	TMP_SENSOR_THREAD_ID,
	INA233_SENSOR_THREAD_ID,
	VR_SENSOR_THREAD_ID,
	DIMM_SENSOR_THREAD_ID,
	ADC_MONITOR_SENSOR_THREAD_ID,
	MAX_SENSOR_THREAD_ID,
};

enum GET_VR_DEV_STATUS {
	GET_VR_DEV_SUCCESS = 0,
	GET_VR_DEV_FAILED,
};

int plat_pldm_sensor_get_sensor_count(int thread_id);
void plat_pldm_sensor_get_pdr_numeric_sensor(int thread_id, int sensor_num,
					     PDR_numeric_sensor *numeric_sensor_table);
uint8_t plat_pldm_sensor_get_vr_dev(uint8_t *vr_dev);

#endif
