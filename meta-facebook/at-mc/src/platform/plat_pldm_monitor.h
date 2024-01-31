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

#ifndef PLAT_PLDM_MONITOR_H
#define PLAT_PLDM_MONITOR_H

enum plat_pldm_event_sensor_num {
	PLDM_EVENT_SSD_1 = 0x80,
	PLDM_EVENT_SSD_2,
	PLDM_EVENT_SSD_3,
	PLDM_EVENT_SSD_4,
};

enum plat_pldm_device_state_set_offset {
	PLDM_STATE_SET_OFFSET_DEVICE_PRESENCE = 0,
	PLDM_STATE_SET_OFFSET_DEVICE_STATUS = 1,
	PLDM_STATE_SET_OFFSET_DEVICE_POWER_STATUS = 2,
};

void plat_send_ssd_present_event(uint8_t ssd_id);
void plat_ssd_present_check();
void plat_send_ssd_power_fault_event(uint8_t ssd_id, uint8_t status);

#endif
