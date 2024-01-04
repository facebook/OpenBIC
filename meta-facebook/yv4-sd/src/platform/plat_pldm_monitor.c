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
#include <stdlib.h>

#include "sensor.h"
#include "hal_gpio.h"
#include "pldm.h"

#include "plat_gpio.h"
#include "plat_i2c.h"
#include "plat_sensor_table.h"
#include "plat_pldm_monitor.h"
#include "plat_isr.h"
#include "power_status.h"

LOG_MODULE_REGISTER(plat_pldm_monitor);
#define SET_POWER_STACK 1024
K_KERNEL_STACK_MEMBER(set_power_sequence_stack, SET_POWER_STACK);
static struct k_thread set_power_sequence_thread;
k_tid_t set_power_sequence_tid;
static K_SEM_DEFINE(cmd_sem, 1, 1);

#define PLAT_PLDM_POWER_ON_BUTTON_MSEC 1000
#define PLAT_PLDM_RESET_BUTTON_MSEC 1000
#define PLAT_PLDM_GRACEFUL_SHUTDOWN_BUTTON_MSEC 1000
#define PLAT_PLDM_POWER_OFF_BUTTON_MSEC 6000
#define PLAT_PLDM_POWER_CYCLE_INTERVAL_MSEC 5000

#define PLAT_PLDM_SEQUENCE_CNT 3

static uint8_t power_sequence[PLAT_PLDM_SEQUENCE_CNT] = { PLAT_PLDM_HOST_PWR_CTRL_DEFAULT,
							  PLAT_PLDM_HOST_PWR_BTN_LOW,
							  PLAT_PLDM_HOST_PWR_CTRL_DEFAULT };
static uint8_t reset_sequence[PLAT_PLDM_SEQUENCE_CNT] = { PLAT_PLDM_HOST_PWR_CTRL_DEFAULT,
							  PLAT_PLDM_HOST_RST_BTN_LOW,
							  PLAT_PLDM_HOST_PWR_CTRL_DEFAULT };

struct pldm_state_effecter_info plat_state_effecter_table[] = {
	[0 ... PLDM_PLATFORM_OEM_AST1030_GPIO_PIN_NUM_MAX] = {
		.entity_type = PLDM_ENTITY_IO_CONTROLLER,
	},
	[PLDM_PLATFORM_OEM_AST1030_GPIO_PIN_NUM_MAX + 1] = {
		.entity_type = PLDM_ENTITY_SUB_CHASSIS,
		.effecter_id = 0x0000,
	},
	[PLDM_PLATFORM_OEM_AST1030_GPIO_PIN_NUM_MAX + 2] = {
		.entity_type = PLDM_ENTITY_DEVICE_DRIVER,
		.effecter_id = PLAT_PLDM_EFFECTER_ID_REINIT_I3C_HUB,
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
	CHECK_ARG_WITH_RETURN(!len, PLDM_ERROR);

	uint8_t *completion_code_p = resp;
	*resp_len = 1;

	switch (info_p->entity_type) {
	case PLDM_ENTITY_IO_CONTROLLER:
		set_effecter_state_gpio_handler(buf, len, resp, resp_len,
						(uint8_t)(info_p->effecter_id & GENMASK(7, 0)));
		break;
	case PLDM_ENTITY_SUB_CHASSIS:
		plat_pldm_set_effecter_state_host_power_control(buf, len, resp, resp_len);
		break;
	case PLDM_ENTITY_DEVICE_DRIVER:
		plat_pldm_set_effecter_state_reinit_i3c_hub(buf, len, resp, resp_len);
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

void plat_pldm_do_host_button_sequence(void *power_sequence, void *pressing_interval, void *arvg1)
{
	uint8_t retry = 3;
	I2C_MSG i2c_msg;
	i2c_msg.bus = CPLD_IO_I2C_BUS;
	i2c_msg.target_addr = CPLD_IO_I2C_ADDR; //server board cpld addr
	i2c_msg.tx_len = 2;
	i2c_msg.rx_len = 0;
	i2c_msg.data[0] = 0x00;

	uint32_t interval = POINTER_TO_UINT(pressing_interval);

	for (int i = 0; i < PLAT_PLDM_SEQUENCE_CNT; i++) {
		i2c_msg.data[1] = ((uint8_t *)power_sequence)[i];
		if (i2c_master_write(&i2c_msg, retry)) {
			LOG_ERR("Fail to do i2c master write");
		}

		if (i == 1) {
			k_msleep(interval);
		}
	}
	k_sem_give(&cmd_sem);
}

void plat_pldm_power_cycle(void *arg1, void *arg2, void *arg3)
{
	uint8_t retry = 3;
	I2C_MSG i2c_msg;
	i2c_msg.bus = CPLD_IO_I2C_BUS;
	i2c_msg.target_addr = CPLD_IO_I2C_ADDR; //server board cpld addr
	i2c_msg.tx_len = 2;
	i2c_msg.rx_len = 0;
	i2c_msg.data[0] = 0x00;

	// Do power off when current power is ON
	if (get_DC_status() == true) {
		// Power off
		for (int i = 0; i < PLAT_PLDM_SEQUENCE_CNT; i++) {
			i2c_msg.data[1] = power_sequence[i];
			if (i2c_master_write(&i2c_msg, retry)) {
				LOG_ERR("Fail to do i2c master write");
			}

			if (i == 1) {
				k_msleep(PLAT_PLDM_POWER_OFF_BUTTON_MSEC);
			}
		}
		k_msleep(PLAT_PLDM_POWER_CYCLE_INTERVAL_MSEC);
	}

	// Power on
	for (int i = 0; i < PLAT_PLDM_SEQUENCE_CNT; i++) {
		i2c_msg.data[1] = power_sequence[i];
		if (i2c_master_write(&i2c_msg, retry)) {
			LOG_ERR("Fail to do i2c master write");
		}

		if (i == 1) {
			k_msleep(PLAT_PLDM_POWER_ON_BUTTON_MSEC);
		}
	}

	k_sem_give(&cmd_sem);
}

uint8_t plat_pldm_host_button_sequence(const uint8_t *power_sequence, uint16_t pressing_interval)
{
	uint8_t retry = 3;
	I2C_MSG i2c_msg;
	i2c_msg.bus = CPLD_IO_I2C_BUS;
	i2c_msg.target_addr = CPLD_IO_I2C_ADDR; //server board cpld addr
	i2c_msg.tx_len = 2;
	i2c_msg.rx_len = 0;
	i2c_msg.data[0] = 0x00;

	for (int i = 0; i < PLAT_PLDM_SEQUENCE_CNT; i++) {
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

	if (req_p->composite_effecter_count !=
	    PLDM_PLATFORM_OEM_HOST_POWER_CTRL_EFFECTER_STATE_FIELD_COUNT) {
		LOG_ERR("Unsupported host power control effecter count, (%d)",
			req_p->composite_effecter_count);
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
		if (get_DC_status() == true) {
			break;
		}

		if (plat_pldm_host_button_sequence(power_sequence,
						   PLAT_PLDM_POWER_ON_BUTTON_MSEC) != 0) {
			LOG_ERR("Failed to do host power on");
		}
		break;
	case EFFECTER_STATE_POWER_STATUS_OFF:
		if (get_DC_status() == false) {
			break;
		}

		if (k_sem_take(&cmd_sem, K_NO_WAIT) != 0) {
			LOG_ERR("Ignore. Previous cmd still executing.");
		} else {
			set_power_sequence_tid = k_thread_create(
				&set_power_sequence_thread, set_power_sequence_stack,
				K_THREAD_STACK_SIZEOF(set_power_sequence_stack),
				plat_pldm_do_host_button_sequence, (void *)power_sequence,
				UINT_TO_POINTER(PLAT_PLDM_POWER_OFF_BUTTON_MSEC), NULL,
				CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
			k_thread_name_set(&set_power_sequence_thread, "do_power_off_thread");
		}
		break;
	case EFFECTER_STATE_POWER_STATUS_CYCLE:
		if (k_sem_take(&cmd_sem, K_NO_WAIT) != 0) {
			LOG_ERR("Ignore. Previous cmd still executing.");
		} else {
			set_power_sequence_tid =
				k_thread_create(&set_power_sequence_thread,
						set_power_sequence_stack,
						K_THREAD_STACK_SIZEOF(set_power_sequence_stack),
						plat_pldm_power_cycle, NULL, NULL, NULL,
						CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
			k_thread_name_set(&set_power_sequence_thread, "do_power_cycle_thread");
		}
		break;
	case EFFECTER_STATE_POWER_STATUS_RESET:
		if (plat_pldm_host_button_sequence(reset_sequence, PLAT_PLDM_RESET_BUTTON_MSEC) !=
		    0) {
			LOG_ERR("Failed to do host power reset");
		}
		break;
	case EFFECTER_STATE_POWER_STATUS_GRACEFUL_SHUTDOWN:
		if (plat_pldm_host_button_sequence(power_sequence,
						   PLAT_PLDM_GRACEFUL_SHUTDOWN_BUTTON_MSEC) != 0) {
			LOG_ERR("Failed to do host graceful shutdown");
		}
		break;
	default:
		LOG_ERR("Unsupported host power control effecter state, (%d)",
			host_power_state->effecter_state);
		*completion_code_p = PLDM_ERROR_INVALID_DATA;
	}

	return;
}

void plat_pldm_set_effecter_state_reinit_i3c_hub(const uint8_t *buf, uint16_t len, uint8_t *resp,
						 uint16_t *resp_len)
{
	LOG_DBG("Access plat_pldm_set_effecter_state_reinit_i3c_hub");
	ARG_UNUSED(len);

	struct pldm_set_state_effecter_states_req *req_p =
		(struct pldm_set_state_effecter_states_req *)buf;
	uint8_t *completion_code_p = resp;
	*resp_len = 1;

	if (req_p->composite_effecter_count >
	    PLDM_PLATFORM_OEM_I3C_HUB_REINIT_EFFECTER_STATE_FIELD_COUNT) {
		LOG_ERR("Unsupported reinit i3c hub effecter count, (%d)",
			req_p->composite_effecter_count);
		*completion_code_p = PLDM_ERROR_INVALID_DATA;
		return;
	}

	set_effecter_state_field_t *reinit_i3c_hub_state = &req_p->field[0];

	switch (reinit_i3c_hub_state->set_request) {
	case PLDM_NO_CHANGE:
		return;
	case PLDM_REQUEST_SET:
		break;
	default:
		LOG_ERR("Invalid reinit i3c hub set request (%d)",
			reinit_i3c_hub_state->set_request);
		*completion_code_p = PLDM_PLATFORM_UNSUPPORTED_EFFECTERSTATE;
		return;
	}

	switch (reinit_i3c_hub_state->effecter_state) {
	case EFFECTER_STATE_I3C_HUB_REINIT:
		reinit_i3c_hub();
		break;
	default:
		LOG_ERR("Unsupported reinit i3c hub effecter state, (%d)",
			reinit_i3c_hub_state->effecter_state);
		*completion_code_p = PLDM_ERROR_INVALID_DATA;
	}

	return;
}
