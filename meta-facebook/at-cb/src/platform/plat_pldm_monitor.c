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

LOG_MODULE_REGISTER(plat_pldm_monitor);

struct pldm_state_effecter_info plat_state_effecter_table[] = {
	[0 ... PLDM_PLATFORM_OEM_AST1030_GPIO_PIN_NUM_MAX] = {
		.entity_type = PLDM_ENTITY_IO_CONTROLLER,
	},
};

void plat_pldm_load_state_effecter_table(void)
{
	memcpy(state_effecter_table, plat_state_effecter_table, sizeof(plat_state_effecter_table));
	return;
}

uint8_t plat_pldm_set_state_effecter_state_handler(const uint8_t *buf, uint16_t len, uint8_t *resp,
						   uint16_t *resp_len,
						   struct pldm_state_effecter_info *info_p)
{
	CHECK_NULL_ARG_WITH_RETURN(buf, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp_len, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(info_p, PLDM_ERROR);
	CHECK_ARG_WITH_RETURN(!len, PLDM_ERROR);

	uint8_t *completion_code_p = resp;
	*resp_len = 1;

	switch (info_p->entity_type) {
	case PLDM_ENTITY_IO_CONTROLLER:
		set_effecter_state_gpio_handler(buf, len, resp, resp_len,
						(uint8_t)(info_p->effecter_id & GENMASK(7, 0)));
		break;
	default:
		LOG_ERR("Unsupport entity type, (%d)", info_p->entity_type);
		*completion_code_p = PLDM_ERROR_INVALID_DATA;
		break;
	}

	return PLDM_SUCCESS;
}

uint8_t plat_pldm_get_state_effecter_state_handler(const uint8_t *buf, uint16_t len, uint8_t *resp,
						   uint16_t *resp_len,
						   struct pldm_state_effecter_info *info_p)
{
	CHECK_NULL_ARG_WITH_RETURN(buf, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp_len, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(info_p, PLDM_ERROR);
	CHECK_ARG_WITH_RETURN(!len, PLDM_ERROR);

	uint8_t *completion_code_p = resp;
	*resp_len = 1;

	switch (info_p->entity_type) {
	case PLDM_ENTITY_IO_CONTROLLER:
		get_effecter_state_gpio_handler(buf, len, resp, resp_len,
						(uint8_t)(info_p->effecter_id & GENMASK(7, 0)));
		break;
	default:
		LOG_ERR("Unsupport entity type, (%d)", info_p->entity_type);
		*completion_code_p = PLDM_ERROR_INVALID_DATA;
		break;
	}

	return PLDM_SUCCESS;
}

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
