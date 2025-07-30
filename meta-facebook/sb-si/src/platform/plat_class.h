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

#ifndef PLAT_CLASS_H
#define PLAT_CLASS_H

#define MAX_SLOT 4
#define FLASH_SLOT_ADDRESS 0x0FF000
#define FLASH_SECTOR 0x1000

typedef enum {
	VR_MPS_MP2971_MP2891,
	VR_MPS_MP2971_MP29816A,
	VR_RNS_ISL69260_RAA228238,
	VR_RNS_ISL69260_RAA228249,
	VR_UNKNOWN,
} si_vr_type_t;

typedef struct {
	uint8_t slot;
	uint8_t eid;
	uint16_t pid;
} mmc_info_t;

extern const mmc_info_t mmc_info_table[MAX_SLOT];

typedef enum {
	TMP_TMP432,
	TMP_EMC1413,
	TMP_TYPE_UNKNOWN,
} si_tmp_type_t;

void init_platform_config();
void init_tmp_type();
uint8_t get_vr_type();
uint8_t get_tmp_type();
void plat_i3c_set_pid();
uint8_t get_slot_id();

#endif
