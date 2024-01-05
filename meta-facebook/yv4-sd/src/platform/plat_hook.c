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
#include "hal_gpio.h"
#include "plat_apml.h"
#include "plat_hook.h"
#include "plat_gpio.h"
#include "plat_dimm.h"

#define RETIMER_INIT_RETRY_COUNT 3

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

ina233_init_arg ina233_init_args[] = {
	[0] = {
		.is_init = false,
		.current_lsb = 0.001,
		.r_shunt = 0.002,
		.mfr_config_init = false,
		.is_need_mfr_device_config_init = false,
		.is_need_set_alert_threshold = false,
	},
	[1] = {
		.is_init = false,
		.current_lsb = 0.001,
		.r_shunt = 0.002,
		.mfr_config_init = false,
		.is_need_mfr_device_config_init = false,
		.is_need_set_alert_threshold = false,
	},
	[2] = {
		.is_init = false,
		.current_lsb = 0.001,
		.r_shunt = 0.002,
		.mfr_config_init = false,
		.is_need_mfr_device_config_init = false,
		.is_need_set_alert_threshold = false,
	},
	[3] = {
		.is_init = false,
		.current_lsb = 0.001,
		.r_shunt = 0.002,
		.mfr_config_init = false,
		.is_need_mfr_device_config_init = false,
		.is_need_set_alert_threshold = false,
	},
};

pt5161l_init_arg pt5161l_init_args[] = { [0] = { .is_init = false,
						 .temp_cal_code_pma_a = { 0, 0, 0, 0 },
						 .temp_cal_code_pma_b = { 0, 0, 0, 0 },
						 .temp_cal_code_avg = 0 },
					 [1] = { .is_init = false,
						 .temp_cal_code_pma_a = { 0, 0, 0, 0 },
						 .temp_cal_code_pma_b = { 0, 0, 0, 0 },
						 .temp_cal_code_avg = 0 } };

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

bool pre_dimm_i3c_read(sensor_cfg *cfg, void *args)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	ARG_UNUSED(args);

	if (!get_post_status()) {
		return true;
	}

	uint8_t dimm_id = sensor_num_map_dimm_id(cfg->num);
	if (get_dimm_present(dimm_id) ==
	    DIMM_NOT_PRSNT) { // Stop monitoring DIMM when it is not present.
		return false;
	}
	return true;
}

bool pre_p3v_bat_read(sensor_cfg *cfg, void *args)
{
	if (gpio_set(P3V_BAT_SCALED_R_EN, GPIO_HIGH)) {
		LOG_ERR("failed to enable p3v bat read");
		return false;
	}
	k_msleep(60);
	return true;
}

bool post_p3v_bat_read(sensor_cfg *cfg, void *args, int *const reading)
{
	if (gpio_set(P3V_BAT_SCALED_R_EN, GPIO_LOW)) {
		LOG_ERR("failed to disable p3v bat read");
		return false;
	}
	return true;
}

bool pre_retimer_read(sensor_cfg *cfg, void *args)
{
	pt5161l_init_arg *init_arg = (pt5161l_init_arg *)cfg->init_args;
	static uint8_t check_init_count = 0;
	bool ret = true;

	if (init_arg->is_init == false) {
		if (check_init_count >= RETIMER_INIT_RETRY_COUNT) {
			LOG_ERR("retimer initial fail reach max retry");
			return false;
		}

		check_init_count += 1;
		ret = init_drive_type_delayed(cfg);
		if (ret == false) {
			LOG_ERR("retimer initial fail");
			return ret;
		}
	}

	return ret;
}
