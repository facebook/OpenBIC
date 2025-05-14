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
#include "plat_sensor_table.h"

#ifndef PLAT_PLDM_MONITOR_H
#define PLAT_PLDM_MONITOR_H

#define MAX_STATE_EFFECTER_IDX 187

enum pldm_plat_effecter_id_high_byte {
	PLAT_EFFECTER_ID_GPIO_HIGH_BYTE = (0xFF << 8),
};

extern struct pldm_state_effecter_info plat_state_effecter_table[];

enum plat_pldm_bic_state_set_offset {
	PLDM_STATE_SET_OFFSET_BIC_BOOT_RESTART_CAUSE = 0,
};

enum plat_pldm_device_state_set_offset {
	PLDM_STATE_SET_OFFSET_DEVICE_PRESENCE = 0,
	PLDM_STATE_SET_OFFSET_DEVICE_STATUS = 1,
};

#endif
