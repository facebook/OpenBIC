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

#include "plat_hook.h"
#include <stdio.h>
#include <string.h>
#include <logging/log.h>
#include "sensor.h"
#include "pmbus.h"
#include "libutil.h"
#include "plat_i2c.h"
#include "power_status.h"
#include "common_i2c_mux.h"
#include "i2c-mux-tca9543a.h"
#include "i2c-mux-pi4msd5v9542.h"
#include "plat_sensor_table.h"
#include "i2c-mux-pca954x.h"
#include "plat_class.h"
#include "plat_dev.h"
#include "nvme.h"
#include "plat_ipmi.h"

LOG_MODULE_REGISTER(plat_hook);

#define PEX_SWITCH_INIT_RETRY_COUNT 20
#define ACCL_SENSOR_COUNT 6

struct k_mutex xdpe15284_mutex;

/**************************************************************************************************
 * INIT ARGS
**************************************************************************************************/
adc_asd_init_arg adc_asd_init_args[] = {
	[0] = { .is_init = false },
};

adm1272_init_arg adm1272_init_args[] = {
	[0] = { .is_init = false, .is_need_set_pwr_cfg = true, .pwr_monitor_cfg.value = 0x3F3F },
	[1] = { .is_init = false, .is_need_set_pwr_cfg = true, .pwr_monitor_cfg.value = 0x3F3F },
};

ltc4286_init_arg ltc4286_init_args[] = {
	[0] = { .is_init = false, .r_sense_mohm = 0.3, .mfr_config_1 = { 0x6D72 } },
	[1] = { .is_init = false, .r_sense_mohm = 0.3, .mfr_config_1 = { 0x6D72 } }
};

ina233_init_arg ina233_init_args[] = {
	[0] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.00117647, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode = 0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b0100,
	},
	.is_need_mfr_device_config_init = true,
	.mfr_device_config = {
		.apol = 0b0,
		.alert_behavior = 0b1,
		.ein_autoclear = 0b1,
		.i2c_filt = 0b0,
		.ein_accum = 0b00,
		.rsvd = 0b0,
		.ein_status = 0b0,
	},
	},
	[1] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.00117647, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode = 0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b0100,
	},
	.is_need_mfr_device_config_init = true,
	.mfr_device_config = {
		.apol = 0b0,
		.alert_behavior = 0b1,
		.ein_autoclear = 0b1,
		.i2c_filt = 0b0,
		.ein_accum = 0b00,
		.rsvd = 0b0,
		.ein_status = 0b0,
	},
	},
	[2] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.00117647, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode = 0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b0100,
	},
	.is_need_mfr_device_config_init = true,
	.mfr_device_config = {
		.apol = 0b0,
		.alert_behavior = 0b1,
		.ein_autoclear = 0b1,
		.i2c_filt = 0b0,
		.ein_accum = 0b00,
		.rsvd = 0b0,
		.ein_status = 0b0,
	},
	},
	[3] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.00117647, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode = 0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b0100,
	},
	.is_need_mfr_device_config_init = true,
	.mfr_device_config = {
		.apol = 0b0,
		.alert_behavior = 0b1,
		.ein_autoclear = 0b1,
		.i2c_filt = 0b0,
		.ein_accum = 0b00,
		.rsvd = 0b0,
		.ein_status = 0b0,
	},
	},
	[4] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.00117647, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode = 0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b0100,
	},
	.is_need_mfr_device_config_init = true,
	.mfr_device_config = {
		.apol = 0b0,
		.alert_behavior = 0b1,
		.ein_autoclear = 0b1,
		.i2c_filt = 0b0,
		.ein_accum = 0b00,
		.rsvd = 0b0,
		.ein_status = 0b0,
	},
	},
	[5] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode = 0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b0100,
	},
	.is_need_mfr_device_config_init = true,
	.mfr_device_config = {
		.apol = 0b0,
		.alert_behavior = 0b1,
		.ein_autoclear = 0b1,
		.i2c_filt = 0b0,
		.ein_accum = 0b00,
		.rsvd = 0b0,
		.ein_status = 0b0,
	},
	},
	[6] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.00117647, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode = 0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b0100,
	},
	.is_need_mfr_device_config_init = true,
	.mfr_device_config = {
		.apol = 0b0,
		.alert_behavior = 0b1,
		.ein_autoclear = 0b1,
		.i2c_filt = 0b0,
		.ein_accum = 0b00,
		.rsvd = 0b0,
		.ein_status = 0b0,
	},
	},
	[7] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.00117647, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode = 0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b0100,
	},
	.is_need_mfr_device_config_init = true,
	.mfr_device_config = {
		.apol = 0b0,
		.alert_behavior = 0b1,
		.ein_autoclear = 0b1,
		.i2c_filt = 0b0,
		.ein_accum = 0b00,
		.rsvd = 0b0,
		.ein_status = 0b0,
	},
	},
	[8] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.00117647, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode = 0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b0100,
	},
	.is_need_mfr_device_config_init = true,
	.mfr_device_config = {
		.apol = 0b0,
		.alert_behavior = 0b1,
		.ein_autoclear = 0b1,
		.i2c_filt = 0b0,
		.ein_accum = 0b00,
		.rsvd = 0b0,
		.ein_status = 0b0,
	},
	},
	[9] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.00117647, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode = 0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b0100,
	},
	.is_need_mfr_device_config_init = true,
	.mfr_device_config = {
		.apol = 0b0,
		.alert_behavior = 0b1,
		.ein_autoclear = 0b1,
		.i2c_filt = 0b0,
		.ein_accum = 0b00,
		.rsvd = 0b0,
		.ein_status = 0b0,
	},
	},
	[10] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.00117647, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode = 0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b0100,
	},
	.is_need_mfr_device_config_init = true,
	.mfr_device_config = {
		.apol = 0b0,
		.alert_behavior = 0b1,
		.ein_autoclear = 0b1,
		.i2c_filt = 0b0,
		.ein_accum = 0b00,
		.rsvd = 0b0,
		.ein_status = 0b0,
	},
	},
	[11] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode = 0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b0100,
	},
	.is_need_mfr_device_config_init = true,
	.mfr_device_config = {
		.apol = 0b0,
		.alert_behavior = 0b1,
		.ein_autoclear = 0b1,
		.i2c_filt = 0b0,
		.ein_accum = 0b00,
		.rsvd = 0b0,
		.ein_status = 0b0,
	},
	},
};

pex89000_init_arg pex_sensor_init_args[] = {
	[0] = { .idx = 0, .is_init = false },
	[1] = { .idx = 1, .is_init = false },
};

sq52205_init_arg sq52205_init_args[] = {
	[0] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.0010002,
	.config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b000,
		.reset_bit = 0b0,
	},
	},
	[1] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.0010002,
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

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK ARGS
 **************************************************************************************************/
mux_config tca9543_configs[] = {
	[0] = { .target_addr = PEX89144_0_MUX_ADDR, .channel = TCA9543A_CHANNEL_1 },
	[1] = { .target_addr = PEX89144_1_MUX_ADDR, .channel = TCA9543A_CHANNEL_1 },
};

mux_config ina233_configs[] = {
	[0] = { .target_addr = 0x70, .channel = TCA9543A_CHANNEL_0 },
	[1] = { .target_addr = 0x70, .channel = TCA9543A_CHANNEL_1 },
};

pwr_monitor_pre_proc_arg pwr_monitor_pre_dvt_args[] = {
	[0] = { .mux_configs = &ina233_configs[1], .card_id = 0 },
	[1] = { .mux_configs = &ina233_configs[1], .card_id = 1 },
	[2] = { .mux_configs = &ina233_configs[1], .card_id = 2 },
	[3] = { .mux_configs = &ina233_configs[1], .card_id = 3 },
	[4] = { .mux_configs = &ina233_configs[1], .card_id = 4 },
	[5] = { .mux_configs = &ina233_configs[1], .card_id = 5 },
	[6] = { .mux_configs = &ina233_configs[0], .card_id = 6 },
	[7] = { .mux_configs = &ina233_configs[0], .card_id = 7 },
	[8] = { .mux_configs = &ina233_configs[0], .card_id = 8 },
	[9] = { .mux_configs = &ina233_configs[0], .card_id = 9 },
	[10] = { .mux_configs = &ina233_configs[0], .card_id = 10 },
	[11] = { .mux_configs = &ina233_configs[0], .card_id = 11 },
};

pwr_monitor_pre_proc_arg pwr_monitor_args[] = {
	[0] = { .mux_configs = NULL, .card_id = 0 },
	[1] = { .mux_configs = NULL, .card_id = 1 },
	[2] = { .mux_configs = NULL, .card_id = 2 },
	[3] = { .mux_configs = NULL, .card_id = 3 },
	[4] = { .mux_configs = NULL, .card_id = 4 },
	[5] = { .mux_configs = NULL, .card_id = 5 },
	[6] = { .mux_configs = NULL, .card_id = 6 },
	[7] = { .mux_configs = NULL, .card_id = 7 },
	[8] = { .mux_configs = NULL, .card_id = 8 },
	[9] = { .mux_configs = NULL, .card_id = 9 },
	[10] = { .mux_configs = NULL, .card_id = 10 },
	[11] = { .mux_configs = NULL, .card_id = 11 },
};

vr_page_cfg xdpe15284_page[] = {
	[0] = { .vr_page = PMBUS_PAGE_0 },
	[1] = { .vr_page = PMBUS_PAGE_1 },
};

mux_config pca9548_configs[] = {
	[0] = { .bus = I2C_BUS8, .target_addr = 0x74, .channel = PCA9548A_CHANNEL_5 },
	[1] = { .bus = I2C_BUS8, .target_addr = 0x74, .channel = PCA9548A_CHANNEL_4 },
	[2] = { .bus = I2C_BUS8, .target_addr = 0x74, .channel = PCA9548A_CHANNEL_3 },
	[3] = { .bus = I2C_BUS8, .target_addr = 0x74, .channel = PCA9548A_CHANNEL_2 },
	[4] = { .bus = I2C_BUS8, .target_addr = 0x74, .channel = PCA9548A_CHANNEL_1 },
	[5] = { .bus = I2C_BUS8, .target_addr = 0x74, .channel = PCA9548A_CHANNEL_0 },
	[6] = { .bus = I2C_BUS7, .target_addr = 0x70, .channel = PCA9548A_CHANNEL_5 },
	[7] = { .bus = I2C_BUS7, .target_addr = 0x70, .channel = PCA9548A_CHANNEL_4 },
	[8] = { .bus = I2C_BUS7, .target_addr = 0x70, .channel = PCA9548A_CHANNEL_3 },
	[9] = { .bus = I2C_BUS7, .target_addr = 0x70, .channel = PCA9548A_CHANNEL_2 },
	[10] = { .bus = I2C_BUS7, .target_addr = 0x70, .channel = PCA9548A_CHANNEL_1 },
	[11] = { .bus = I2C_BUS7, .target_addr = 0x70, .channel = PCA9548A_CHANNEL_0 },
};

mux_config pca9546_configs[] = {
	[0] = { .target_addr = 0x72, .channel = PCA9546A_CHANNEL_0 },
	[1] = { .target_addr = 0x72, .channel = PCA9546A_CHANNEL_1 },
	[2] = { .target_addr = 0x72, .channel = PCA9546A_CHANNEL_2 },
	[3] = { .target_addr = 0x72, .channel = PCA9546A_CHANNEL_3 },
};

accl_card_info accl_card_info_args[] = {
	[0] = { .card_id = 0, .freya_info_ptr = &accl_freya_info[0] },
	[1] = { .card_id = 1, .freya_info_ptr = &accl_freya_info[1] },
	[2] = { .card_id = 2, .freya_info_ptr = &accl_freya_info[2] },
	[3] = { .card_id = 3, .freya_info_ptr = &accl_freya_info[3] },
	[4] = { .card_id = 4, .freya_info_ptr = &accl_freya_info[4] },
	[5] = { .card_id = 5, .freya_info_ptr = &accl_freya_info[5] },
	[6] = { .card_id = 6, .freya_info_ptr = &accl_freya_info[6] },
	[7] = { .card_id = 7, .freya_info_ptr = &accl_freya_info[7] },
	[8] = { .card_id = 8, .freya_info_ptr = &accl_freya_info[8] },
	[9] = { .card_id = 9, .freya_info_ptr = &accl_freya_info[9] },
	[10] = { .card_id = 10, .freya_info_ptr = &accl_freya_info[10] },
	[11] = { .card_id = 11, .freya_info_ptr = &accl_freya_info[11] },
};

accl_card_sensor_info accl_sensor_info_args[] = {
	[0] = { .is_sensor_init = false, .start_sensor_num = SENSOR_NUM_TEMP_ACCL_1_FREYA_1 },
	[1] = { .is_sensor_init = false, .start_sensor_num = SENSOR_NUM_TEMP_ACCL_2_FREYA_1 },
	[2] = { .is_sensor_init = false, .start_sensor_num = SENSOR_NUM_TEMP_ACCL_3_FREYA_1 },
	[3] = { .is_sensor_init = false, .start_sensor_num = SENSOR_NUM_TEMP_ACCL_4_FREYA_1 },
	[4] = { .is_sensor_init = false, .start_sensor_num = SENSOR_NUM_TEMP_ACCL_5_FREYA_1 },
	[5] = { .is_sensor_init = false, .start_sensor_num = SENSOR_NUM_TEMP_ACCL_6_FREYA_1 },
	[6] = { .is_sensor_init = false, .start_sensor_num = SENSOR_NUM_TEMP_ACCL_7_FREYA_1 },
	[7] = { .is_sensor_init = false, .start_sensor_num = SENSOR_NUM_TEMP_ACCL_8_FREYA_1 },
	[8] = { .is_sensor_init = false, .start_sensor_num = SENSOR_NUM_TEMP_ACCL_9_FREYA_1 },
	[9] = { .is_sensor_init = false, .start_sensor_num = SENSOR_NUM_TEMP_ACCL_10_FREYA_1 },
	[10] = { .is_sensor_init = false, .start_sensor_num = SENSOR_NUM_TEMP_ACCL_11_FREYA_1 },
	[11] = { .is_sensor_init = false, .start_sensor_num = SENSOR_NUM_TEMP_ACCL_12_FREYA_1 },
};

uint8_t plat_monitor_table_arg[] = { PCIE_CARD_1, PCIE_CARD_2,	PCIE_CARD_3,  PCIE_CARD_4,
				     PCIE_CARD_5, PCIE_CARD_6,	PCIE_CARD_7,  PCIE_CARD_8,
				     PCIE_CARD_9, PCIE_CARD_10, PCIE_CARD_11, PCIE_CARD_12 };

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK FUNC
 **************************************************************************************************/
bool pre_ina233_read(sensor_cfg *cfg, void *args)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(args, false);

	// Select Channel
	bool ret = true;
	bool is_power_good = false;
	bool is_sensor_init_done = false;
	int mutex_status = 0;
	pwr_monitor_pre_proc_arg *pre_args = (pwr_monitor_pre_proc_arg *)args;

	if (asic_card_info[pre_args->card_id].card_status == ASIC_CARD_NOT_PRESENT) {
		cfg->is_enable_polling = false;
		return false;
	}

	is_sensor_init_done = get_sensor_init_done_flag();
	if (is_sensor_init_done) {
		is_power_good = is_accl_power_good(pre_args->card_id);
		if (is_power_good != true) {
			cfg->cache_status = SENSOR_POLLING_DISABLE;
			return true;
		}

		if (cfg->cache_status == SENSOR_POLLING_DISABLE) {
			cfg->cache_status = SENSOR_INIT_STATUS;
		}
	}

	if (pre_args->mux_configs != NULL) {
		mux_config *mux_cfg = pre_args->mux_configs;
		mux_cfg->bus = cfg->port;

		struct k_mutex *mutex = get_i2c_mux_mutex(mux_cfg->bus);
		mutex_status = k_mutex_lock(mutex, K_MSEC(MUTEX_LOCK_INTERVAL_MS));
		if (mutex_status != 0) {
			LOG_ERR("Mutex lock fail, status: %d", mutex_status);
			return false;
		}

		ret = set_mux_channel(*mux_cfg, MUTEX_LOCK_ENABLE);
		if (ret == false) {
			LOG_ERR("ina233 switch mux fail");
			k_mutex_unlock(mutex);
		}

		return ret;
	}

	return true;
}

bool post_ina233_read(sensor_cfg *cfg, void *args, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(args, false);
	ARG_UNUSED(reading);

	pwr_monitor_pre_proc_arg *pre_args = (pwr_monitor_pre_proc_arg *)args;
	if (pre_args->mux_configs != NULL) {
		int unlock_status = 0;
		uint8_t bus = cfg->port;

		struct k_mutex *mutex = get_i2c_mux_mutex(bus);
		if (mutex->lock_count != 0) {
			unlock_status = k_mutex_unlock(mutex);
		}

		if (unlock_status != 0) {
			LOG_ERR("Mutex unlock fail, status: %d", unlock_status);
			return false;
		}
	}

	return true;
}

bool pre_xdpe15284_read(sensor_cfg *cfg, void *args)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(args, false);

	bool ret = false;
	int retry = 3;
	int mutex_status = 0;
	I2C_MSG msg = { 0 };
	vr_page_cfg *xdpe15284_vr_page = (vr_page_cfg *)args;

	mutex_status = k_mutex_lock(&xdpe15284_mutex, K_MSEC(MUTEX_LOCK_INTERVAL_MS));
	if (mutex_status != 0) {
		LOG_ERR("Mutex lock fail, status: %d", mutex_status);
		return false;
	}

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 2;
	msg.data[0] = PMBUS_PAGE;
	msg.data[1] = xdpe15284_vr_page->vr_page;

	ret = i2c_master_write(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Set xdpe15284 page fail, ret: %d", ret);
		return false;
	}

	return true;
}

bool post_xdpe15284_read(sensor_cfg *cfg, void *args, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	ARG_UNUSED(reading);
	ARG_UNUSED(args);

	int unlock_status = k_mutex_unlock(&xdpe15284_mutex);
	if (unlock_status != 0) {
		LOG_ERR("Mutex unlock fail, status: %d, sensor num: 0x%x", unlock_status, cfg->num);
		return false;
	}

	return true;
}

bool pre_pex89000_read(sensor_cfg *cfg, void *args)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(args, false);

	/* Can not access i2c mux and PEX89000 when DC off */
	if (is_acb_power_good() == false) {
		return false;
	}

	pex89000_init_arg *pex_init_arg = (pex89000_init_arg *)cfg->init_args;
	mux_config *pre_args = (mux_config *)args;
	pre_args->bus = cfg->port;

	bool ret = true;
	static uint8_t check_init_count = 0;

	ret = set_mux_channel(*pre_args, MUTEX_LOCK_ENABLE);
	if (ret == false) {
		LOG_ERR("PEX switch mux fail, sensor num: 0x%x", cfg->num);
		return ret;
	}

	if (pex_init_arg->is_init == false) {
		if (check_init_count >= PEX_SWITCH_INIT_RETRY_COUNT) {
			return false;
		}

		check_init_count += 1;
		ret = init_drive_type_delayed(cfg);
		if (ret == false) {
			LOG_ERR("PEX initial fail, sensor num: 0x%x", cfg->num);
			return ret;
		}
	}

	return ret;
}

bool pre_accl_nvme_read(sensor_cfg *cfg, void *args)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(args, false);

	accl_card_info *card_info_args = (accl_card_info *)args;
	uint8_t card_id = card_info_args->card_id;
	freya_info *accl_freya = card_info_args->freya_info_ptr;

	CHECK_NULL_ARG_WITH_RETURN(card_info_args->freya_info_ptr, false);

	/** Check whether Freya is accessible **/
	if (asic_card_info[card_id].card_status != ASIC_CARD_PRESENT) {
		cfg->cache_status = SENSOR_NOT_PRESENT;
		return true;
	}

	switch (cfg->target_addr) {
	case ACCL_FREYA_1_ADDR:
		if (asic_card_info[card_id].asic_1_status != ASIC_CARD_DEVICE_PRESENT) {
			cfg->cache_status = SENSOR_NOT_PRESENT;
			return true;
		}
		break;
	case ACCL_FREYA_2_ADDR:
		if (asic_card_info[card_id].asic_2_status != ASIC_CARD_DEVICE_PRESENT) {
			cfg->cache_status = SENSOR_NOT_PRESENT;
			return true;
		}
		break;
	default:
		break;
	}

	/** Check ACCL card power status **/
	bool is_power_good = false;
	uint8_t index = 0;

	is_power_good = is_accl_power_good(card_id);
	if (is_power_good != true) {
		cfg->cache_status = SENSOR_POLLING_DISABLE;
		accl_sensor_info_args[card_id].is_sensor_init = false;
		clear_freya_cache_flag(card_id);
		return true;
	}

	if (cfg->cache_status == SENSOR_POLLING_DISABLE) {
		cfg->cache_status = SENSOR_INIT_STATUS;
	}

	mux_config accl_mux = { 0 };
	if (get_accl_mux_config(card_id, &accl_mux) != true) {
		LOG_ERR("Fail to get ACCL card mux config, card id: 0x%x, sensor num: 0x%x",
			card_id, cfg->num);
		return false;
	}

	int mutex_status = 0;
	struct k_mutex *mutex = get_i2c_mux_mutex(cfg->port);

	mutex_status = k_mutex_lock(mutex, K_MSEC(MUTEX_LOCK_INTERVAL_MS));
	if (mutex_status != 0) {
		LOG_ERR("Mutex lock fail, status: %d, card id: 0x%x, sensor num: 0x%x",
			mutex_status, card_id, cfg->num);
		return false;
	}

	if (set_mux_channel(accl_mux, MUTEX_LOCK_ENABLE) == false) {
		LOG_ERR("ACCL switch card mux fail, card id: 0x%x, sensor num: 0x%x", card_id,
			cfg->num);
		goto error_exit;
	}

	if (accl_sensor_info_args[card_id].is_sensor_init != true) {
		for (index = 0; index < ACCL_SENSOR_COUNT; ++index) {
			uint8_t sensor_num =
				accl_sensor_info_args[card_id].start_sensor_num + index;

			sensor_cfg *accl_sensor_cfg = get_common_sensor_cfg_info(sensor_num);
			if (accl_sensor_cfg == NULL) {
				LOG_ERR("Fail to get sensor cfg info, card id: 0x%x, sensor num: 0x%x",
					card_id, sensor_num);
				goto error_exit;
			}

			if (init_drive_type_delayed(accl_sensor_cfg) != true) {
				LOG_ERR("Fail to delayed init sensor, card id: 0x%x, sensor num: 0x%x",
					card_id, accl_sensor_cfg->num);
				goto error_exit;
			}
		}

		accl_sensor_info_args[card_id].is_sensor_init = true;
	}

	int ret = 0;
	uint8_t nvme_temp = 0;
	uint8_t drive_ready_bit = 0;
	uint8_t nvme_status[FREYA_STATUS_BLOCK_LENGTH] = { 0 };

	ret = read_nvme_info(cfg->port, cfg->target_addr, FREYA_STATUS_BLOCK_OFFSET,
			     FREYA_STATUS_BLOCK_LENGTH, nvme_status);
	if (ret != 0) {
		LOG_ERR("ACCL pre-read get freya status fail, card id: 0x%x, sensor num: 0x%x",
			card_id, cfg->num);
		goto error_exit;
	}

	nvme_temp = nvme_status[NVME_TEMPERATURE_INDEX];
	drive_ready_bit = (nvme_status[FREYA_READY_STATUS_OFFSET] & FREYA_READY_STATUS_BIT);
	if ((nvme_temp == 0) || (drive_ready_bit != 0)) {
		/* Freya not ready */
		cfg->cache_status = SENSOR_POLLING_DISABLE;

		switch (cfg->target_addr) {
		case ACCL_FREYA_1_ADDR:
			if (accl_freya->is_cache_freya1_info) {
				accl_freya->is_cache_freya1_info = false;
				memset(&accl_freya->freya1_fw_info, 0, FREYA_FW_VERSION_LENGTH);
			}
			accl_freya->freya1_fw_info.is_freya_ready = FREYA_NOT_READY;
			break;
		case ACCL_FREYA_2_ADDR:
			if (accl_freya->is_cache_freya2_info) {
				accl_freya->is_cache_freya2_info = false;
				memset(&accl_freya->freya2_fw_info, 0, FREYA_FW_VERSION_LENGTH);
			}
			accl_freya->freya2_fw_info.is_freya_ready = FREYA_NOT_READY;
			break;
		default:
			break;
		}

		k_mutex_unlock(mutex);
		return true;
	}

	switch (cfg->target_addr) {
	case ACCL_FREYA_1_ADDR:
		if (accl_freya->is_cache_freya1_info != true) {
			ret = get_freya_fw_info(cfg->port, cfg->target_addr,
						&accl_freya->freya1_fw_info);
			if ((ret == 0) || (ret == FREYA_NOT_SUPPORT_MODULE_IDENTIFIER_RET_CODE)) {
				accl_freya->is_cache_freya1_info = true;
				return true;
			}
		}
		break;
	case ACCL_FREYA_2_ADDR:
		if (accl_freya->is_cache_freya2_info != true) {
			ret = get_freya_fw_info(cfg->port, cfg->target_addr,
						&accl_freya->freya2_fw_info);
			if ((ret == 0) || (ret == FREYA_NOT_SUPPORT_MODULE_IDENTIFIER_RET_CODE)) {
				accl_freya->is_cache_freya2_info = true;
				return true;
			}
		}
		break;
	default:
		LOG_ERR("Invalid Freya sensor address, card id: 0x%x, sensor num: 0x%x", card_id,
			cfg->num);
		goto error_exit;
	}

	if (ret != 0) {
		if (ret != FREYA_NOT_READY_RET_CODE) {
			LOG_ERR("Get freya info fail, sensor num: 0x%x", cfg->num);
		}
		goto error_exit;
	}

	return true;

error_exit:
	k_mutex_unlock(mutex);
	return false;
}

bool post_accl_nvme_read(sensor_cfg *cfg, void *args, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(args, false);
	ARG_UNUSED(reading);

	int unlock_status = 0;
	uint8_t bus = cfg->port;
	accl_card_info *card_info_args = (accl_card_info *)args;

	struct k_mutex *mutex = get_i2c_mux_mutex(bus);
	if (mutex->lock_count != 0) {
		unlock_status = k_mutex_unlock(mutex);
	}

	if (unlock_status != 0) {
		LOG_ERR("Mutex unlock fail, status: %d, card id: 0x%x, sensor num: 0x%x",
			unlock_status, card_info_args->card_id, cfg->num);
		return false;
	}

	return true;
}
