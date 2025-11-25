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

#ifndef PLAT_DIMM_H
#define PLAT_DIMM_H

#ifdef ENABLE_VISTARA

enum DIMM_ID {
	ASIC1_DIMM_ID_A,
	ASIC1_DIMM_ID_B,
	ASIC1_DIMM_ID_C,
	ASIC1_DIMM_ID_D,
	ASIC2_DIMM_ID_A,
	ASIC2_DIMM_ID_B,
	ASIC2_DIMM_ID_C,
	ASIC2_DIMM_ID_D,
	DIMM_ID_MAX,
	DIMM_ID_UNKNOWN = 0xff,
};

enum DIMM_PRSNT_STATUS {
	DIMM_NOT_PRSNT,
	DIMM_PRSNT,
};

enum NUMBER_DIMM_TEMP {
	SENSOR_NUM_ASIC1_DIMM_A_TEMP = 0x000C,
	SENSOR_NUM_ASIC1_DIMM_B_TEMP,
	SENSOR_NUM_ASIC1_DIMM_C_TEMP,
	SENSOR_NUM_ASIC1_DIMM_D_TEMP,
	SENSOR_NUM_ASIC2_DIMM_A_TEMP,
	SENSOR_NUM_ASIC2_DIMM_B_TEMP,
	SENSOR_NUM_ASIC2_DIMM_C_TEMP,
	SENSOR_NUM_ASIC2_DIMM_D_TEMP,
};

uint8_t sensor_num_map_dimm_id(uint8_t sensor_num);
uint8_t get_dimm_present(uint8_t dimm_id);
void create_init_ddr_slot_info_thread(void);

#endif

#endif