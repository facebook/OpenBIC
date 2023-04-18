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

#include <stdio.h>
#include <string.h>
#include <logging/log.h>
#include "ast_adc.h"
#include "sensor.h"
#include "libutil.h"
#include "rg3mxxb12.h"
#include "plat_i2c.h"
#include "plat_hook.h"
#include "plat_sensor_table.h"

#define RETIMER_INIT_RETRY_COUNT 3

LOG_MODULE_DECLARE(sensor);
K_MUTEX_DEFINE(i2c_hub_mutex);

/**************************************************************************************************
 * INIT ARGS
**************************************************************************************************/
adc_asd_init_arg adc_asd_init_args[] = { [0] = { .is_init = false } };

ina233_init_arg ina233_init_args[] = {
	[0] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.002 },
	[1] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.002 },
	[2] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.002 },
	[3] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.002 },
	[4] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.002 },
	[5] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.002 },
};

sq52205_init_arg sq52205_init_args[] = {
	[0] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.002,
	.config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b000,
		.reset_bit = 0b0,
	},
	},
	[1] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.002,
	.config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b000,
		.reset_bit = 0b0,
	},
	},
	[2] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.002,
	.config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b000,
		.reset_bit = 0b0,
	},
	},
	[3] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.002,
	.config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b000,
		.reset_bit = 0b0,
	},
	},
	[4] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.002,
	.config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b000,
		.reset_bit = 0b0,
	},
	},
	[5] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.002,
	.config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b000,
		.reset_bit = 0b0,
	},
	},
};

i2c_proc_arg i2c_proc_args[] = {
	[0] = { .bus = I2C_BUS2, .channel = I2C_HUB_CHANNEL_0 },
	[1] = { .bus = I2C_BUS2, .channel = I2C_HUB_CHANNEL_1 },
	[2] = { .bus = I2C_BUS2, .channel = I2C_HUB_CHANNEL_2 },
	[3] = { .bus = I2C_BUS2, .channel = I2C_HUB_CHANNEL_3 },
	[4] = { .bus = I2C_BUS2, .channel = I2C_HUB_CHANNEL_4 },
	[5] = { .bus = I2C_BUS2, .channel = I2C_HUB_CHANNEL_5 },
};

pt5161l_init_arg pt5161l_init_args[] = { [0] = { .is_init = false,
						 .temp_cal_code_pma_a = { 0, 0, 0, 0 },
						 .temp_cal_code_pma_b = { 0, 0, 0, 0 },
						 .temp_cal_code_avg = 0 } };

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK FUNC
 **************************************************************************************************/

bool pre_i2c_bus_read(uint8_t sensor_num, void *args)
{
	CHECK_NULL_ARG_WITH_RETURN(args, false);

	if (k_mutex_lock(&i2c_hub_mutex, K_MSEC(I2C_HUB_MUTEX_TIMEOUT_MS))) {
		LOG_ERR("sensor number 0x%x mutex lock fail", sensor_num);
		return false;
	}

	i2c_proc_arg *pre_proc_args = (i2c_proc_arg *)args;

	if (!rg3mxxb12_select_slave_port_connect(pre_proc_args->bus, pre_proc_args->channel)) {
		k_mutex_unlock(&i2c_hub_mutex);
		return false;
	}

	return true;
}

bool post_i2c_bus_read(uint8_t sensor_num, void *args, int *reading)
{
	ARG_UNUSED(reading);
	CHECK_NULL_ARG_WITH_RETURN(args, false);

	i2c_proc_arg *post_proc_args = (i2c_proc_arg *)args;

	/**
       * close all channels after the sensor read to avoid conflict with 
       * other devices reading.
       */
	if (!rg3mxxb12_select_slave_port_connect(post_proc_args->bus,
						 RG3MXXB12_SSPORTS_ALL_DISCONNECT)) {
		k_mutex_unlock(&i2c_hub_mutex);
		LOG_ERR("Close HUB channel failed!");
		return false;
	}

	if (k_mutex_unlock(&i2c_hub_mutex)) {
		LOG_ERR("sensor num 0x%x mutex unlock failed!", sensor_num);
		return false;
	}

	return true;
}

bool pre_retimer_read(uint8_t sensor_num, void *args)
{
	ARG_UNUSED(args);

	sensor_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];
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
