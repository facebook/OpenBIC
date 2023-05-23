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

#include <stdint.h>
#include <stdbool.h>

#define MAX_COUNT_DIMM 8
#define DIMM_INDEX_BYTE 5
#define DIMM_STATUS_BYTE 7
#define DIMM_INDEX_MIN 0
#define DIMM_INDEX_MAX 7
#define DIMM_PRESENT 0x1

enum DIMM_ID {
	DIMM_ID_A = 0,
	DIMM_ID_B,
	DIMM_ID_C,
	DIMM_ID_D,
	DIMM_ID_E,
	DIMM_ID_F,
	DIMM_ID_G,
	DIMM_ID_H,
	DIMM_ID_UNKNOWN = 0xff,
};

bool is_dimm_inited();
void init_dimm_status();
bool get_dimm_presence_status(uint8_t dimm_id);
void set_dimm_presence_status(uint8_t index, uint8_t status);
uint8_t sensor_num_map_dimm_id(uint8_t sensor_num);

#endif
