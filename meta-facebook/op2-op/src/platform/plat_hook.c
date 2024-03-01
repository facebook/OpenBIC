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
#include "m88rt51632.h"
#include "rg3mxxb12.h"
#include "p3h284x.h"
#include "plat_class.h"
#include "plat_i2c.h"
#include "plat_hook.h"
#include "plat_sensor_table.h"

#define RETIMER_INIT_RETRY_COUNT 3

LOG_MODULE_DECLARE(sensor);
K_MUTEX_DEFINE(i2c_hub_mutex);

/**************************************************************************************************
 * INIT ARGS
**************************************************************************************************/
adc_asd_init_arg adc_expa_asd_init_args[] = {
	[0] = { 
		.is_init = false,
		.deglitch[0] = { .deglitch_en = true, .upper_bound = 0x2DB },
		.deglitch[1] = { .deglitch_en = true, .upper_bound = 0x2DB },
		.deglitch[2] = { .deglitch_en = true, .upper_bound = 0x2DB },
		.deglitch[5] = { .deglitch_en = true, .upper_bound = 0x2DB },
		.deglitch[6] = { .deglitch_en = true, .upper_bound = 0x31A },
		.deglitch[7] = { .deglitch_en = true, .upper_bound = 0X18D },
	},
	[1] = {
		.is_init = false,
		.deglitch[0] = { .deglitch_en = true, .upper_bound = 0x214 },
	}
};

adc_asd_init_arg adc_expb_asd_init_args[] = {
	[0] = { 
		.is_init = false,
		.deglitch[0] = { .deglitch_en = true, .upper_bound = 0x2DB },
		.deglitch[1] = { .deglitch_en = true, .upper_bound = 0x2DB },
		.deglitch[2] = { .deglitch_en = true, .upper_bound = 0x2DB },
		.deglitch[3] = { .deglitch_en = true, .upper_bound = 0x2DB },
		.deglitch[4] = { .deglitch_en = true, .upper_bound = 0x2DB },
		.deglitch[5] = { .deglitch_en = true, .upper_bound = 0x2DB },
	},
	[1] = {
		.is_init = false,
		.deglitch[3] = { .deglitch_en = true, .upper_bound = 0x31A },
		.deglitch[6] = { .deglitch_en = true, .upper_bound = 0x214 },
	}
};

ina233_init_arg ina233_init_args[] = {
	[0] = { .is_init = false,
		.current_lsb = 0.001,
		.r_shunt = 0.002,
		.is_need_mfr_device_config_init = false,
		.is_need_set_alert_threshold = false,
	},
	[1] = { .is_init = false,
		.current_lsb = 0.001,
		.r_shunt = 0.002,
		.is_need_mfr_device_config_init = false,
		.is_need_set_alert_threshold = false,
	},
	[2] = { .is_init = false,
		.current_lsb = 0.001,
		.r_shunt = 0.002,
		.is_need_mfr_device_config_init = false,
		.is_need_set_alert_threshold = false,
	},
	[3] = { .is_init = false,
		.current_lsb = 0.001,
		.r_shunt = 0.002,
		.is_need_mfr_device_config_init = false,
		.is_need_set_alert_threshold = false,
	},
	[4] = { .is_init = false,
		.current_lsb = 0.001,
		.r_shunt = 0.002,
		.is_need_mfr_device_config_init = false,
		.is_need_set_alert_threshold = false,
	},
	[5] = { .is_init = false,
		.current_lsb = 0.001,
		.r_shunt = 0.002,
		.is_need_mfr_device_config_init = false,
		.is_need_set_alert_threshold = false,
	},
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
	.is_need_accum_config_init = false,
	.is_need_set_alert_threshold = false,
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
	.is_need_accum_config_init = false,
	.is_need_set_alert_threshold = false,
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
	.is_need_accum_config_init = false,
	.is_need_set_alert_threshold = false,
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
	.is_need_accum_config_init = false,
	.is_need_set_alert_threshold = false,
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
	.is_need_accum_config_init = false,
	.is_need_set_alert_threshold = false,
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
	.is_need_accum_config_init = false,
	.is_need_set_alert_threshold = false,
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

bool pre_i2c_bus_read(sensor_cfg *cfg, void *args)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(args, false);
	uint16_t i3c_hub_type = get_i3c_hub_type();

	if (k_mutex_lock(&i2c_hub_mutex, K_MSEC(I2C_HUB_MUTEX_TIMEOUT_MS))) {
		LOG_ERR("sensor number 0x%x mutex lock fail", cfg->num);
		return false;
	}

	i2c_proc_arg *pre_proc_args = (i2c_proc_arg *)args;

	if (i3c_hub_type == RG3M87B12_DEVICE_INFO) {
		if (!rg3mxxb12_select_slave_port_connect(pre_proc_args->bus,
							 pre_proc_args->channel)) {
			k_mutex_unlock(&i2c_hub_mutex);
			return false;
		}
	} else {
		if (!p3h284x_select_slave_port_connect(pre_proc_args->bus,
						       pre_proc_args->channel)) {
			k_mutex_unlock(&i2c_hub_mutex);
			return false;
		}
	}

	return true;
}

bool post_i2c_bus_read(sensor_cfg *cfg, void *args, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(args, false);
	ARG_UNUSED(reading);
	uint16_t i3c_hub_type = get_i3c_hub_type();

	i2c_proc_arg *post_proc_args = (i2c_proc_arg *)args;

	/**
       * close all channels after the sensor read to avoid conflict with 
       * other devices reading.
       */
	if (i3c_hub_type == RG3M87B12_DEVICE_INFO) {
		if (!rg3mxxb12_select_slave_port_connect(post_proc_args->bus,
							 RG3MXXB12_SSPORTS_ALL_DISCONNECT)) {
			k_mutex_unlock(&i2c_hub_mutex);
			LOG_ERR("Close HUB channel failed!");
			return false;
		}
	} else {
		if (!p3h284x_select_slave_port_connect(post_proc_args->bus,
						       P3H284X_SSPORTS_ALL_DISCONNECT)) {
			k_mutex_unlock(&i2c_hub_mutex);
			LOG_ERR("Close HUB channel failed!");
			return false;
		}
	}

	if (k_mutex_unlock(&i2c_hub_mutex)) {
		LOG_ERR("sensor num 0x%x mutex unlock failed!", cfg->num);
		return false;
	}

	return true;
}

bool pre_retimer_read(sensor_cfg *cfg, void *args)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	ARG_UNUSED(args);
	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, false);

	pt5161l_init_arg *init_arg = (pt5161l_init_arg *)cfg->init_args;
	static uint8_t check_init_count = 0;
	bool ret = true;
	uint8_t retimer_type;

	if (init_arg->is_init == false) {
		if (check_init_count >= RETIMER_INIT_RETRY_COUNT) {
			LOG_ERR("retimer initial fail reach max retry");
			return false;
		}

		retimer_type = get_pcie_retimer_type();
		switch (retimer_type) {
		case RETIMER_TYPE_PT5161L:
			check_init_count += 1;
			cfg->type = sensor_dev_pt5161l;
			ret = init_drive_type_delayed(cfg);
			if (ret == false) {
				LOG_ERR("PT5161L retimer initial fail");
				return ret;
			}
			break;
		case RETIMER_TYPE_M88RT51632:
			check_init_count += 1;
			cfg->type = sensor_dev_m88rt51632;
			cfg->offset = M88RT51632_TEMP_OFFSET;
			ret = init_drive_type_delayed(cfg);
			if (ret == false) {
				LOG_ERR("M88RT51632 retimer initial fail");
				return ret;
			}
			break;
		default:
			break;
		}
	}

	return ret;
}
