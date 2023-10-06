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

#include "pldm_monitor.h"

#ifndef PLAT_PLDM_MONITOR_H
#define PLAT_PLDM_MONITOR_H

#define MAX_STATE_EFFECTER_IDX 168

enum pldm_plat_effecter_id_high_byte {
	PLAT_EFFECTER_ID_GPIO_HIGH_BYTE = (0xFF << 8),
};

extern struct pldm_state_effecter_info plat_state_effecter_table[];

enum plat_pldm_event_sensor_num {
	// ACCL1 - ACCL12
	PLDM_EVENT_ACCL_1 = 0x01,
	PLDM_EVENT_ACCL_2,
	PLDM_EVENT_ACCL_3,
	PLDM_EVENT_ACCL_4,
	PLDM_EVENT_ACCL_5,
	PLDM_EVENT_ACCL_6,
	PLDM_EVENT_ACCL_7,
	PLDM_EVENT_ACCL_8,
	PLDM_EVENT_ACCL_9,
	PLDM_EVENT_ACCL_10,
	PLDM_EVENT_ACCL_11,
	PLDM_EVENT_ACCL_12,
	PLDM_EVENT_ACCL_PWR_CBL_1,
	PLDM_EVENT_ACCL_PWR_CBL_2,
	PLDM_EVENT_ACCL_PWR_CBL_3,
	PLDM_EVENT_ACCL_PWR_CBL_4,
	PLDM_EVENT_ACCL_PWR_CBL_5,
	PLDM_EVENT_ACCL_PWR_CBL_6,
	PLDM_EVENT_ACCL_PWR_CBL_7,
	PLDM_EVENT_ACCL_PWR_CBL_8,
	PLDM_EVENT_ACCL_PWR_CBL_9,
	PLDM_EVENT_ACCL_PWR_CBL_10,
	PLDM_EVENT_ACCL_PWR_CBL_11,
	PLDM_EVENT_ACCL_PWR_CBL_12,
	PLDM_EVENT_FIO,
};

enum plat_pldm_device_state_set_offset {
	PLDM_STATE_SET_OFFSET_DEVICE_PRESENCE = 0,
	PLDM_STATE_SET_OFFSET_DEVICE_STATUS = 1,
};

void plat_accl_present_check();
void plat_accl_power_cable_present_check();
void plat_fio_present_check();
uint8_t plat_set_effecter_states_req(uint8_t device_type, uint8_t board_info, uint8_t event_type);

#endif
