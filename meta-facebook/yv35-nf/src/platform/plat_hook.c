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
#include "hal_i2c.h"
#include "plat_sensor_table.h"
#include "plat_hook.h"

LOG_MODULE_REGISTER(plat_hook);

adc_asd_init_arg adc_asd_init_args[] = { [0] = { .is_init = false } };

ina233_init_arg ina233_init_args[] = {
	// P12V
	[0] = {
	.is_init = false,
	.current_lsb = 0.001,
	.r_shunt = 0.005,
	.mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011,
		.rsvd = 0b0100,
	},
	.is_need_mfr_device_config_init = false,
	},
	// P3V3
	[1] = {
	.is_init = false,
	.current_lsb = 0.001,
	.r_shunt = 0.005,
	.mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011,
		.rsvd = 0b0100,
	},
	.is_need_mfr_device_config_init = false,
	},
};

xdpe12284c_pre_arg xdpe12284c_pre_read_args[] = {
	[0] = { 0x0 },
	[1] = { 0x1 },
};

bool pre_xdpe12284c_read(sensor_cfg *cfg, void *args)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(args, false);

	xdpe12284c_pre_arg *page_selection = (xdpe12284c_pre_arg *)args;
	uint8_t retry = 5;
	I2C_MSG i2c_msg;
	int ret = 0;

	// Set page
	memset(&i2c_msg, 0, sizeof(I2C_MSG));
	i2c_msg.bus = cfg->port;
	i2c_msg.target_addr = cfg->target_addr;
	i2c_msg.tx_len = 2;
	i2c_msg.data[0] = VR_PAGE_OFFSET;
	i2c_msg.data[1] = page_selection->vr_page;

	ret = i2c_master_write(&i2c_msg, retry);
	if (ret != 0) {
		LOG_ERR("Failed to set vr page due to write i2c address 0x%x failed, ret: %d",
			i2c_msg.target_addr, ret);
		return false;
	}

	return true;
}
