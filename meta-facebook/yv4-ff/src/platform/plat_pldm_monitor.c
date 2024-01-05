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
#include <device.h>

#include "sensor.h"
#include "hal_gpio.h"
#include "pldm.h"

#include "plat_gpio.h"
#include "plat_i2c.h"
#include "plat_sensor_table.h"
#include "plat_pldm_monitor.h"

LOG_MODULE_REGISTER(plat_pldm_monitor);

struct pldm_state_effecter_info plat_state_effecter_table[] = {
	[0 ... PLDM_PLATFORM_OEM_AST1030_GPIO_PIN_NUM_MAX] = {
		.entity_type = PLDM_ENTITY_IO_CONTROLLER,
	},

	[PLDM_PLATFORM_OEM_AST1030_GPIO_PIN_NUM_MAX + 1] = {
		.entity_type = PLDM_ENTITY_OTHER_BUS,
		.effecter_id = PLAT_PLDM_EFFECTER_ID_UART_SWITCH,
	},

	[PLDM_PLATFORM_OEM_AST1030_GPIO_PIN_NUM_MAX + 2] = {
		.entity_type = PLDM_ENTITY_DEVICE_DRIVER,
		.effecter_id = PLAT_PLDM_EFFECTER_ID_SPI_REINIT,
	},
};

void plat_pldm_load_state_effecter_table(void)
{
	memcpy(state_effecter_table, plat_state_effecter_table, sizeof(plat_state_effecter_table));
	return;
}

void plat_pldm_dev_driver_handler(const uint8_t *buf, uint16_t len, uint8_t *resp,
				  uint16_t *resp_len, uint16_t effecter_id)
{
	CHECK_NULL_ARG(buf);
	CHECK_NULL_ARG(resp);
	CHECK_NULL_ARG(resp_len);

	uint8_t *completion_code_p = resp;

	switch (effecter_id) {
	case PLAT_PLDM_EFFECTER_ID_SPI_REINIT:
		pldm_spi_reinit("spi1_cs0", buf, len, resp, resp_len);
		break;
	default:
		LOG_ERR("Unsupport effecter id, (0x%x)", effecter_id);
		*completion_code_p = PLDM_ERROR_INVALID_DATA;
		*resp_len = 1;
		return;
		break;
	}
}

void plat_pldm_switch_uart(const uint8_t *buf, uint16_t len, uint8_t *resp, uint16_t *resp_len)
{
	CHECK_NULL_ARG(buf);
	CHECK_NULL_ARG(resp);
	CHECK_NULL_ARG(resp_len);

	struct pldm_set_state_effecter_states_req *req_p =
		(struct pldm_set_state_effecter_states_req *)buf;
	uint8_t *completion_code_p = resp;
	*resp_len = 1;

	if (req_p->composite_effecter_count !=
	    PLDM_PLATFORM_OEM_SWITCH_UART_EFFECTER_STATE_FIELD_COUNT) {
		LOG_ERR("Unsupported switch uart effecter count, (%d)",
			req_p->composite_effecter_count);
		*completion_code_p = PLDM_ERROR_INVALID_DATA;
		return;
	}

	set_effecter_state_field_t *uart = &req_p->field[0];

	if (uart->set_request >= PLDM_SET_REQUEST_MAX) {
		LOG_ERR("Invalid switch uart set request (%d)", uart->set_request);
		*completion_code_p = PLDM_PLATFORM_UNSUPPORTED_EFFECTERSTATE;
		return;
	}

	uint8_t uart_number = uart->effecter_state;
	uint32_t hicr9_val = 0, hicra_val = 0;

	// HICR9: Host Interface Control Register 9(offset: 98h)
	hicr9_val = sys_read32(LPC_HICR9_REG);
	clear_bits(&hicr9_val, 8, 11);

	// HICRA: Host Interface Control Register A(offset: 9Ch)
	hicra_val = sys_read32(LPC_HICRA_REG);
	clear_bits(&hicra_val, 0, 2);

	switch (uart_number) {
	case UART1:
		// IO1 to IO5: Ｗrite 0101b to bit[11:8]
		hicr9_val = SETBITS(hicr9_val, 0b0101, 8);
		sys_write32(hicr9_val, LPC_HICR9_REG);

		// IO5 to IO1: Write 111b to bit[2:0]
		hicra_val = SETBITS(hicra_val, 0b111, 0);
		sys_write32(hicra_val, LPC_HICRA_REG);
		break;
	case UART2:
		// IO5 to IO2: Write 0110b to bit[11:8]
		hicr9_val = SETBITS(hicr9_val, 0b0110, 8);
		sys_write32(hicr9_val, LPC_HICR9_REG);

		// IO2 to IO5: Write 111b to bit[5:3]
		hicra_val = SETBITS(hicra_val, 0b111, 3);
		sys_write32(hicra_val, LPC_HICRA_REG);
		break;
	case UART_BIC:
		// UART5 to IO5: Write 1010b to bit[11:8]
		hicr9_val = SETBITS(hicr9_val, 0b1010, 8);
		sys_write32(hicr9_val, LPC_HICR9_REG);

		// UART1 to IO1: Write 000b to bit[2:0]
		sys_write32(hicra_val, LPC_HICRA_REG);
		break;
	default:
		LOG_ERR("Unsupport uart number (%d)", uart_number);
		break;
	}

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
	case PLDM_ENTITY_OTHER_BUS:
		plat_pldm_switch_uart(buf, len, resp, resp_len);
		break;
	case PLDM_ENTITY_DEVICE_DRIVER:
		plat_pldm_dev_driver_handler(buf, len, resp, resp_len, info_p->effecter_id);
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
