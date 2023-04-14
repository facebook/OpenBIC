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

#include "plat_sys.h"

#include <stdio.h>
#include <stdlib.h>
#include <logging/log.h>
#include "hal_gpio.h"
#include "plat_gpio.h"
#include "libutil.h"
#include "fru.h"
#include "eeprom.h"
#include "plat_fru.h"
#include "plat_isr.h"

LOG_MODULE_REGISTER(plat_sys);

uint8_t debug_sel_mode = 0;

/* BMC reset */
void BMC_reset_handler()
{
	LOG_WRN("BMC reset not supported from here");
}

K_WORK_DELAYABLE_DEFINE(BMC_reset_work, BMC_reset_handler);
int pal_submit_bmc_cold_reset()
{
	k_work_schedule(&BMC_reset_work, K_MSEC(1000));
	return 0;
}

int pal_get_set_add_debug_sel_mode_status(uint8_t options, uint8_t *status)
{
	CHECK_NULL_ARG_WITH_RETURN(status, -1);

	uint8_t ret;
	EEPROM_ENTRY fru_entry = { 0 };

	fru_entry.config.dev_id = SYS_DEBUG_ID;
	fru_entry.offset = BIC_CONFIG_START;
	fru_entry.data_len = 1;

	uint8_t fru_index = 0;
	bool is_id_find = find_FRU_ID(fru_entry.config.dev_id, &fru_index);
	if (is_id_find == false) {
		LOG_ERR("find fru write config fail via fru id: 0x%x", fru_entry.config.dev_id);
		return FRU_INVALID_ID;
	}

	memcpy(&fru_entry.config, &fru_config[fru_index], sizeof(fru_config[fru_index]));

	switch (options) {
	case GET_STATUS:
		ret = eeprom_read(&fru_entry);
		*status = fru_entry.data[0];
		break;
	case SET_STATUS:
		fru_entry.data[0] = *status;
		ret = eeprom_write(&fru_entry);
		debug_sel_mode = *status;
		break;
	default:
		LOG_ERR("Debug sel mode options unkown %d", options);
		return -1;
	}

	if (ret != FRU_READ_SUCCESS) {
		return -1;
	}

	return 0;
}

void check_debug_sel_mode_status()
{
	pal_get_set_add_debug_sel_mode_status(GET_STATUS, &debug_sel_mode);
	if (debug_sel_mode == 1) {
		LOG_INF("Enable debug sel mode");
	} else {
		LOG_INF("Disenable debug sel mode");
	}
}
