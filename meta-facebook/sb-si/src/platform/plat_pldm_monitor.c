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
#include "plat_pldm_monitor.h"
#include "plat_class.h"
#include "plat_led.h"

LOG_MODULE_REGISTER(plat_pldm_monitor);

struct pldm_state_effecter_info plat_state_effecter_table[] = {
	[0 ... PLDM_PLATFORM_OEM_AST1030_GPIO_PIN_NUM_MAX] = {
		.entity_type = PLDM_ENTITY_IO_CONTROLLER,
	},
};

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

void plat_pldm_load_state_effecter_table(void)
{
	memcpy(state_effecter_table, plat_state_effecter_table, sizeof(plat_state_effecter_table));
	return;
}
