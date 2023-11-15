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
#include "power_status.h"
#include "apml.h"
#include "plat_apml.h"
#include "plat_hook.h"

LOG_MODULE_REGISTER(plat_hook);

adc_asd_init_arg ast_adc_init_args[] = {
	[0] = {
		.is_init = false,
		.deglitch[0] = { .deglitch_en = false,},
		.deglitch[1] = { .deglitch_en = false,},
		.deglitch[2] = { .deglitch_en = false,},
		.deglitch[3] = { .deglitch_en = false,},
		.deglitch[4] = { .deglitch_en = false,},
		.deglitch[5] = { .deglitch_en = false,},
	},
	[1] = {
		.is_init = false,
		.deglitch[0] = { .deglitch_en = false,},
		.deglitch[1] = { .deglitch_en = false,},
		.deglitch[2] = { .deglitch_en = false,},
		.deglitch[4] = { .deglitch_en = false,},
		.deglitch[5] = { .deglitch_en = false,},
		.deglitch[6] = { .deglitch_en = false,},
	}
};

apml_mailbox_init_arg apml_mailbox_init_args[] = { [0] = { .data = 0x00000000, .retry = 0 } };

vr_pre_proc_arg vr_pre_read_args[] = {
	[0] = { 0x0 },
	[1] = { 0x1 },
};

bool pre_vr_read(sensor_cfg *cfg, void *args)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(args, false);

	vr_pre_proc_arg *pre_proc_args = (vr_pre_proc_arg *)args;
	uint8_t retry = 5;
	I2C_MSG msg;

	/* set page */
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 2;
	msg.data[0] = 0x00;
	msg.data[1] = pre_proc_args->vr_page;
	if (i2c_master_write(&msg, retry)) {
		LOG_ERR("pre_vr_read, set page fail");
		return false;
	}
	return true;
}

bool post_amd_tsi_read(sensor_cfg *cfg, void *args, int *const reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	ARG_UNUSED(args);

	if (!reading) {
		return check_reading_pointer_null_is_allowed(cfg);
	}

	if (!get_tsi_status()) {
		LOG_DBG("TSI not initialized.");
		return true;
	}

	if (!get_post_status()) {
		LOG_DBG("Post code not complete.");
		return true;
	}

	uint8_t tsi_status = 0;
	if (apml_read_byte(I2C_BUS14, SB_TSI_ADDR, SBTSI_STATUS, &tsi_status)) {
		LOG_ERR("Failed to read TSI status");
		return true;
	}

	// TODO: if throttle send event to BMC

	return true;
}
