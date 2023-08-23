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

#include "plat_gpio.h"
#include "plat_i2c.h"
#include "plat_sensor_table.h"
#include "plat_pldm_monitor.h"

LOG_MODULE_REGISTER(plat_pldm_monitor);

#define POWER_ON_BUTTON_MSEC 1000
#define RESET_BUTTON_MSEC 1000
#define GRACEFUL_SHUTDOWN_BUTTON_MSEC 1000
#define POWER_OFF_BUTTON_MSEC 6000
#define POWER_CYCLE_INTERVAL_MSEC 5000

static uint8_t power_sequence[3] = {PLDM_PLATFORM_HOST_PWR_CTRL_DEFAULT, PLDM_PLATFORM_HOST_PWR_BTN_LOW,
			PLDM_PLATFORM_HOST_PWR_CTRL_DEFAULT};
static uint8_t reset_sequence[3] = {PLDM_PLATFORM_HOST_PWR_CTRL_DEFAULT, PLDM_PLATFORM_HOST_RST_BTN_LOW,
			PLDM_PLATFORM_HOST_PWR_CTRL_DEFAULT};

enum pldm_plat_effecter_id_high_byte {
	PLAT_EFFECTER_ID_GPIO_HIGH_BYTE = (0xFF << 8)
};

static struct plat_state_effecter_info {
	uint16_t entity_type;
	uint16_t effecter_id;
} plat_state_effecter_table[] = {
	[0 ... PLDM_PLATFORM_OEM_AST1030_GPIO_PIN_NUM_NAX] = {
		.entity_type = PLDM_ENTITY_IO_CONTROLLER,
	},
	[PLDM_PLATFORM_OEM_AST1030_GPIO_PIN_NUM_NAX + 1] = {
		.entity_type = PLDM_ENTITY_SUB_CHASSIS,
		.effecter_id = 0x0000,
	},
};

static struct plat_state_effecter_info *find_state_effecter_info(uint16_t effecter_id)
{
	for (uint8_t i = 0; i < ARRAY_SIZE(plat_state_effecter_table); i++) {
		if (plat_state_effecter_table[i].effecter_id == effecter_id)
			return &plat_state_effecter_table[i];
	}

	return NULL;
}

uint8_t plat_pldm_set_state_effecter_state_handler(const uint8_t *buf, uint16_t len, uint8_t *resp,
						   uint16_t *resp_len)
{

	CHECK_NULL_ARG_WITH_RETURN(buf, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp_len, PLDM_ERROR);
	CHECK_ARG_WITH_RETURN(!len, PLDM_ERROR);

	struct pldm_set_state_effecter_states_req *req_p =
		(struct pldm_set_state_effecter_states_req *)buf;
	uint8_t *completion_code_p = resp;
	*resp_len = 1;


	if (req_p->composite_effecter_count < 0x01 || req_p->composite_effecter_count > 0x08) {
		*completion_code_p = PLDM_ERROR_INVALID_DATA;
		return PLDM_SUCCESS;
	}


	if (len != (PLDM_SET_STATE_EFFECTER_REQ_NO_STATE_FIELD_BYTES +
		    sizeof(set_effecter_state_field_t) * req_p->composite_effecter_count)) {
		*completion_code_p = PLDM_ERROR_INVALID_LENGTH;
		return PLDM_SUCCESS;
	}

	struct plat_state_effecter_info *info_p = find_state_effecter_info(req_p->effecter_id);

	if (!info_p) {
		LOG_ERR("Can't find effecter ID (0x%x) info", req_p->effecter_id);
		*completion_code_p = PLDM_ERROR_INVALID_DATA;
		return PLDM_SUCCESS;
	}

	switch (info_p->entity_type) {
	case PLDM_ENTITY_IO_CONTROLLER:
		set_effecter_state_gpio_handler(buf, len, resp, resp_len,
						(uint8_t)(info_p->effecter_id & GENMASK(7, 0)));
		break;
	case PLDM_ENTITY_SUB_CHASSIS:
		plat_pldm_set_effecter_state_host_power_control(buf, len, resp, resp_len);
		break;
	default:
		LOG_ERR("Unsupport entity type, (%d)", info_p->entity_type);
		*completion_code_p = PLDM_ERROR_INVALID_DATA;
		break;
	}

	return PLDM_SUCCESS;
}

uint8_t plat_pldm_get_state_effecter_state_handler(const uint8_t *buf, uint16_t len, uint8_t *resp,
						   uint16_t *resp_len)
{
	CHECK_NULL_ARG_WITH_RETURN(buf, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp_len, PLDM_ERROR);
	CHECK_ARG_WITH_RETURN(!len, PLDM_ERROR);

	struct pldm_get_state_effecter_states_req *req_p =
		(struct pldm_get_state_effecter_states_req *)buf;
	uint8_t *completion_code_p = resp;
	*resp_len = 1;

	struct plat_state_effecter_info *info_p = find_state_effecter_info(req_p->effecter_id);

	if (!info_p) {
		LOG_ERR("Can't find effecter ID (0x%x) info", req_p->effecter_id);
		*completion_code_p = PLDM_ERROR_INVALID_DATA;
		return PLDM_SUCCESS;
	}

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

void plat_pldm_assign_gpio_effecter_id()
{
	for (uint8_t i = 0; i <= PLDM_PLATFORM_OEM_AST1030_GPIO_PIN_NUM_NAX; i++) {
		plat_state_effecter_table[i].effecter_id = (PLAT_EFFECTER_ID_GPIO_HIGH_BYTE | i);
	}
}

uint8_t plat_pldm_host_button_sequence(const uint8_t *power_sequence, uint16_t pressing_interval)
{
	uint8_t sts_cnt = 3;
	uint8_t retry = 3;
	I2C_MSG i2c_msg;
	i2c_msg.bus = I2C_BUS1;
	i2c_msg.target_addr = 0x21; //server board cpld addr
	i2c_msg.tx_len = 2;
	i2c_msg.rx_len = 0;
	i2c_msg.data[0] = 0x00;

	for (int i = 0; i < sts_cnt; i++) {
		i2c_msg.data[1] = power_sequence[i];
		if (i2c_master_write(&i2c_msg, retry)) {
			return -1;
		}

		if (i == 1) {
			k_msleep(pressing_interval);
		}
	}
	return 0;
}

void plat_pldm_set_effecter_state_host_power_control(const uint8_t *buf, uint16_t len,
							  uint8_t *resp, uint16_t *resp_len)
{
	CHECK_NULL_ARG(buf);
	CHECK_NULL_ARG(resp);
	CHECK_NULL_ARG(resp_len);

	struct pldm_set_state_effecter_states_req *req_p =
		(struct pldm_set_state_effecter_states_req *)buf;
	uint8_t *completion_code_p = resp;
	*resp_len = 1;

	if (req_p->composite_effecter_count != PLDM_PLATFORM_OEM_HOST_POWER_CTRL_EFFECTER_STATE_FIELD_COUNT) {
		LOG_ERR("Unsupport host power control effecter count, (%d)", req_p->composite_effecter_count);
		*completion_code_p = PLDM_ERROR_INVALID_DATA;
		return;
	}

	set_effecter_state_field_t *host_power_state = &req_p->field[0];

	if (host_power_state->set_request >= PLDM_SET_REQUEST_MAX) {
		  LOG_ERR("Invalid host power control set request (%d)",
					host_power_state->set_request);
		*completion_code_p = PLDM_PLATFORM_UNSUPPORTED_EFFECTERSTATE;
		return;
	}

	switch (host_power_state->effecter_state) {
	case EFFECTER_STATE_POWER_STATUS_ON:
		if (plat_pldm_host_button_sequence(power_sequence, POWER_ON_BUTTON_MSEC) != 0) {
			LOG_ERR("Failed to do host power on");
		}
		break;
	case EFFECTER_STATE_POWER_STATUS_OFF:
		if (plat_pldm_host_button_sequence(power_sequence, POWER_OFF_BUTTON_MSEC) != 0) {
			LOG_ERR("Failed to do host power off");
		}
		break;
	case EFFECTER_STATE_POWER_STATUS_CYCLE:
		if (plat_pldm_host_button_sequence(power_sequence, POWER_OFF_BUTTON_MSEC) != 0) {
			LOG_ERR("Failed to do host power cycle");
		}
		k_msleep(POWER_CYCLE_INTERVAL_MSEC);
		if (plat_pldm_host_button_sequence(power_sequence, POWER_ON_BUTTON_MSEC) != 0) {
			LOG_ERR("Failed to do host power cycle");
		}
		break;
	case EFFECTER_STATE_POWER_STATUS_RESET:
		if (plat_pldm_host_button_sequence(reset_sequence, RESET_BUTTON_MSEC) != 0) {
			LOG_ERR("Failed to do host power reset");
		}
		break;
	case EFFECTER_STATE_POWER_STATUS_GRACEFUL_SHUTDOWN:
		if (plat_pldm_host_button_sequence(power_sequence, GRACEFUL_SHUTDOWN_BUTTON_MSEC) != 0) {
			LOG_ERR("Failed to do host graceful shutdown");
		}
		break;
	default:
		LOG_ERR("Unsupport host power control effecter state, (%d)",
				host_power_state->effecter_state);
		*completion_code_p = PLDM_ERROR_INVALID_DATA;
		break;
	}

	return;
}
