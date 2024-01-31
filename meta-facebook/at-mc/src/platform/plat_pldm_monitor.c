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

#include <logging/log.h>

#include "sensor.h"
#include "hal_gpio.h"
#include "pldm.h"
#include "pmbus.h"
#include "plat_fru.h"
#include "plat_gpio.h"
#include "plat_sensor_table.h"
#include "plat_pldm_monitor.h"
#include "plat_class.h"
#include "pldm_state_set.h"

LOG_MODULE_REGISTER(plat_pldm_monitor);

void plat_send_ssd_present_event(uint8_t ssd_id)
{
	struct pldm_sensor_event_state_sensor_state event;
	event.sensor_offset = PLDM_STATE_SET_OFFSET_DEVICE_PRESENCE;
	event.event_state = PLDM_STATE_SET_NOT_PRESENT;
	event.previous_event_state = PLDM_STATE_SET_NOT_PRESENT;

	if (pldm_send_platform_event(PLDM_SENSOR_EVENT, PLDM_EVENT_SSD_1 + ssd_id,
				     PLDM_STATE_SENSOR_STATE, (uint8_t *)&event,
				     sizeof(struct pldm_sensor_event_state_sensor_state))) {
		LOG_ERR("Send SSD%d presence event log failed", ssd_id + 1);
	}
}

void plat_ssd_present_check()
{
	bool is_present = CARD_NOT_PRESENT;
	for (uint8_t i = CARD_8_INDEX; i >= CARD_5_INDEX; i--) {
		is_present = pcie_card_info[i].card_device_type;
		if (is_present != E1S_PRESENT) {
			plat_send_ssd_present_event(CARD_8_INDEX - i);
		}
	}
}

void plat_send_ssd_power_fault_event(uint8_t ssd_id, uint8_t status)
{
	struct pldm_sensor_event_state_sensor_state event;
	event.sensor_offset = PLDM_STATE_SET_OFFSET_DEVICE_POWER_STATUS;
	event.event_state = status;
	event.previous_event_state = PLDM_STATE_SET_OEM_DEVICE_NO_POWER_GOOD;
	if (pldm_send_platform_event(PLDM_SENSOR_EVENT, PLDM_EVENT_SSD_1 + ssd_id,
				     PLDM_STATE_SENSOR_STATE, (uint8_t *)&event,
				     sizeof(struct pldm_sensor_event_state_sensor_state))) {
		LOG_ERR("Send SSD%d power fault event log failed", ssd_id);
	}
}
