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
#include "util_sys.h"
#include "plat_dev.h"
#include "plat_ipmi.h"

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

void plat_send_accl_present_event(uint8_t card_id, uint8_t option)
{
	uint8_t start_id = 0;

	switch (option) {
	case ACCL_CARD_PRESENCE:
		start_id = PLDM_EVENT_ACCL_1;
		break;
	case ACCL_CABLE_PRESENCE:
		start_id = PLDM_EVENT_ACCL_PWR_CBL_1;
		break;
	default:
		LOG_ERR("Invalid option: 0x%x", option);
		return;
	}

	struct pldm_sensor_event_state_sensor_state event;
	event.sensor_offset = PLDM_STATE_SET_OFFSET_DEVICE_PRESENCE;
	event.event_state = PLDM_STATE_SET_NOT_PRESENT;
	event.previous_event_state = PLDM_STATE_SET_NOT_PRESENT;
	if (pldm_send_platform_event(PLDM_SENSOR_EVENT, start_id + card_id, PLDM_STATE_SENSOR_STATE,
				     (uint8_t *)&event,
				     sizeof(struct pldm_sensor_event_state_sensor_state))) {
		LOG_ERR("Send ACCL presence event log fail, card id: 0x%x, option: 0x%x", card_id,
			option);
	}
}

void plat_accl_present_check()
{
	bool is_present = ASIC_CARD_NOT_PRESENT;
	for (uint8_t i = 0; i < ASIC_CARD_COUNT; i++) {
		is_present = asic_card_info[i].card_status;
		if (is_present == ASIC_CARD_NOT_PRESENT) {
			plat_send_accl_present_event(i, ACCL_CARD_PRESENCE);
		}
	}
}

void plat_accl_power_cable_present_check()
{
	bool is_present = ASIC_CARD_NOT_PRESENT;
	for (uint8_t i = 0; i < ASIC_CARD_COUNT; i++) {
		is_present = asic_card_info[i].pwr_cbl_status;
		if (is_present == ASIC_CARD_NOT_PRESENT) {
			plat_send_accl_present_event(i, ACCL_CABLE_PRESENCE);
		}
	}
}

void plat_fio_present_check()
{
	struct pldm_sensor_event_state_sensor_state event;
	event.sensor_offset = PLDM_STATE_SET_OFFSET_DEVICE_PRESENCE;
	if (gpio_get(PRSNT_FIO_N) == HIGH_ACTIVE) {
		event.event_state = PLDM_STATE_SET_NOT_PRESENT;
		event.previous_event_state = PLDM_STATE_SET_NOT_PRESENT;
		if (pldm_send_platform_event(PLDM_SENSOR_EVENT, PLDM_EVENT_FIO,
					     PLDM_STATE_SENSOR_STATE, (uint8_t *)&event,
					     sizeof(struct pldm_sensor_event_state_sensor_state))) {
			LOG_ERR("Send FIO cable presence event log failed");
		}
	}
}

void plat_accl_power_good_fail_event(uint8_t card_id, uint8_t currenr_state)
{
	struct pldm_sensor_event_state_sensor_state event;
	event.sensor_offset = PLDM_STATE_SET_OFFSET_DEVICE_POWER_STATUS;
	event.previous_event_state = PLDM_STATE_SET_OEM_DEVICE_NO_POWER_GOOD;
	event.event_state = currenr_state;
	if (pldm_send_platform_event(PLDM_SENSOR_EVENT, PLDM_EVENT_ACCL_1 + card_id,
				     PLDM_STATE_SENSOR_STATE, (uint8_t *)&event,
				     sizeof(struct pldm_sensor_event_state_sensor_state))) {
		LOG_ERR("Send card_id: 0x%x power good fail event failed, current state: 0x%x",
			card_id, currenr_state);
	}
}

void plat_accl_cable_power_good_fail_event(uint8_t card_id, uint8_t status)
{
	struct pldm_sensor_event_state_sensor_state event;
	event.sensor_offset = PLDM_STATE_SET_OFFSET_DEVICE_POWER_STATUS;
	event.previous_event_state = PLDM_STATE_SET_OEM_DEVICE_NO_POWER_GOOD;
	event.event_state = status;
	if (pldm_send_platform_event(PLDM_SENSOR_EVENT, PLDM_EVENT_ACCL_PWR_CBL_1 + card_id,
				     PLDM_STATE_SENSOR_STATE, (uint8_t *)&event,
				     sizeof(struct pldm_sensor_event_state_sensor_state))) {
		LOG_ERR("Send card_id: 0x%x cable power good fail event failed, status: 0x%x",
			card_id, status);
	}
}

void plat_asic_nvme_status_event(uint8_t card_id, uint8_t device_id, uint8_t status)
{
	if (card_id >= ASIC_CARD_COUNT || device_id >= ACCL_CARD_DEV_COUNT) {
		LOG_ERR("Invalid card id: 0x%x, device id: 0x%x", card_id, device_id);
		return;
	}

	struct pldm_sensor_event_state_sensor_state event;
	event.sensor_offset = PLDM_STATE_SET_OFFSET_DEVICE_NVME_STATUS;
	event.previous_event_state = PLDM_STATE_SET_OEM_DEVICE_NVME_UNKNOWN_STATUS;
	event.event_state = status;
	if (pldm_send_platform_event(PLDM_SENSOR_EVENT,
				     (card_id * 2 + device_id) + PLDM_EVENT_ACCL_1_DEV_1,
				     PLDM_STATE_SENSOR_STATE, (uint8_t *)&event,
				     sizeof(struct pldm_sensor_event_state_sensor_state))) {
		LOG_ERR("Send card_id: 0x%x device id: 0x%x status:%s nvme event failed", card_id,
			device_id,
			((status == PLDM_STATE_SET_OEM_DEVICE_NVME_NOT_READY) ? "not ready" :
										"ready"));
	}
}

void plat_send_event_pre_work()
{
	uint8_t index = 0;
	uint8_t status = 0;
	for (index = 0; index < ASIC_CARD_COUNT; ++index) {
		if (accl_freya_info[index].is_cache_freya1_info) {
			status = ((accl_freya_info[index].freya1_fw_info.is_freya_ready ==
				   FREYA_READY) ?
					  PLDM_STATE_SET_OEM_DEVICE_NVME_READY :
					  PLDM_STATE_SET_OEM_DEVICE_NVME_NOT_READY);
			plat_asic_nvme_status_event(index, PCIE_DEVICE_ID1, status);
		}

		if (accl_freya_info[index].is_cache_freya2_info) {
			status = ((accl_freya_info[index].freya2_fw_info.is_freya_ready ==
				   FREYA_READY) ?
					  PLDM_STATE_SET_OEM_DEVICE_NVME_READY :
					  PLDM_STATE_SET_OEM_DEVICE_NVME_NOT_READY);
			plat_asic_nvme_status_event(index, PCIE_DEVICE_ID2, status);
		}
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
