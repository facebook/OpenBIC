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
#include "plat_mctp.h"

LOG_MODULE_REGISTER(plat_pldm_monitor);

void plat_accl_present_check()
{
	bool is_present = ASIC_CARD_NOT_PRESENT;
	struct pldm_sensor_event_state_sensor_state event;
	for (uint8_t i = 0; i < ASIC_CARD_COUNT; i++) {
		is_present = asic_card_info[i].card_status;
		event.sensor_offset = PLDM_STATE_SET_OFFSET_DEVICE_PRESENCE;
		event.event_state =
			is_present ? PLDM_STATE_SET_PRESENT : PLDM_STATE_SET_NOT_PRESENT;
		event.previous_event_state = PLDM_STATE_SET_NOT_PRESENT;
		if (pldm_send_platform_event(PLDM_SENSOR_EVENT, PLDM_EVENT_ACCL_1 + i,
					     PLDM_STATE_SENSOR_STATE, (uint8_t *)&event,
					     sizeof(struct pldm_sensor_event_state_sensor_state))) {
			LOG_ERR("Send ACCL%d presence event log failed", PLDM_EVENT_ACCL_1 + i);
		}
	}
}

void plat_accl_power_cable_present_check()
{
	bool is_present = ASIC_CARD_NOT_PRESENT;
	struct pldm_sensor_event_state_sensor_state event;
	for (uint8_t i = 0; i < ASIC_CARD_COUNT; i++) {
		is_present = asic_card_info[i].pwr_cbl_status;
		event.sensor_offset = PLDM_STATE_SET_OFFSET_DEVICE_PRESENCE;
		event.event_state =
			is_present ? PLDM_STATE_SET_PRESENT : PLDM_STATE_SET_NOT_PRESENT;
		event.previous_event_state = PLDM_STATE_SET_NOT_PRESENT;
		if (pldm_send_platform_event(PLDM_SENSOR_EVENT, PLDM_EVENT_ACCL_PWR_CBL_1 + i,
					     PLDM_STATE_SENSOR_STATE, (uint8_t *)&event,
					     sizeof(struct pldm_sensor_event_state_sensor_state))) {
			LOG_ERR("Send ACCL%d power cable presence event log failed",
				PLDM_EVENT_ACCL_PWR_CBL_1 + i);
		}
	}
}

void plat_fio_present_check()
{
	struct pldm_sensor_event_state_sensor_state event;
	event.sensor_offset = PLDM_STATE_SET_OFFSET_DEVICE_PRESENCE;
	event.event_state = gpio_get(PRSNT_FIO_N) == LOW_ACTIVE ? PLDM_STATE_SET_PRESENT :
									PLDM_STATE_SET_NOT_PRESENT;
	event.previous_event_state = PLDM_STATE_SET_NOT_PRESENT;
	if (pldm_send_platform_event(PLDM_SENSOR_EVENT, PLDM_EVENT_FIO, PLDM_STATE_SENSOR_STATE,
				     (uint8_t *)&event,
				     sizeof(struct pldm_sensor_event_state_sensor_state))) {
		LOG_ERR("Send FIO cable presence event log failed");
	}
}

uint8_t plat_set_effecter_states_req(uint8_t device_type, uint8_t board_info, uint8_t event_type)
{
	uint8_t ret = 0;
	uint16_t effecter_id =
		(PLDM_EFFECTER_ID_FUNC_HIGH_BYTE << 8) | PLDM_EFFECTER_ID_ADDSEL_LOW_BYTE;
	struct pldm_set_state_effecter_states_req req = { 0 };
	void *mctp_inst = NULL;
	mctp_ext_params ext_params = { 0 };

	ret = get_mctp_route_info(MCTP_EID_BMC, &mctp_inst, &ext_params);
	if (ret != MCTP_SUCCESS) {
		LOG_ERR("Fail to find BMC mctp inst");
		return PLDM_ERROR;
	}

	ret = pldm_fill_addsel_req(&req, effecter_id, device_type, board_info, event_type);
	if (ret != PLDM_SUCCESS) {
		LOG_ERR("PLDM fill addsel req fail, effecter_id: 0x%x, device_type: 0x%x, board_info: 0x%x, event_type: 0x%x",
			effecter_id, device_type, board_info, event_type);
		return ret;
	}

	return pldm_send_set_state_effecter_states_req(&req, mctp_inst, ext_params);
}
