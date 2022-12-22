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

LOG_MODULE_REGISTER(plat_hook);

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

ina233_init_arg ina233_init_args[] = {
	[0] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001 },
	[1] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001 },
	[2] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001 },
	[3] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001 },
	[4] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001 },
	[5] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001 },
	[6] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001 },
	[7] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001 },
	[8] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001 },
	[9] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001 },
	[10] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001 },
	[11] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001 },
};

pex89000_init_arg pex_sensor_init_args[] = {
	[0] = { .idx = 0, .is_init = false },
	[1] = { .idx = 1, .is_init = false },
};

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK ARGS
 **************************************************************************************************/
mux_config tca9543_configs[] = {
	[0] = { .target_addr = 0x70, .channel = TCA9543A_CHANNEL_1 },
	[1] = { .target_addr = 0x71, .channel = TCA9543A_CHANNEL_1 },
};

mux_config pi4msd5v9542_configs[] = {
	[0] = { .target_addr = 0x70, .channel = PI4MSD5V9542_CHANNEL_0 },
	[1] = { .target_addr = 0x70, .channel = PI4MSD5V9542_CHANNEL_1 },
};

vr_page_cfg xdpe15284_page[] = {
	[0] = { .vr_page = PMBUS_PAGE_0 },
	[1] = { .vr_page = PMBUS_PAGE_1 },
};

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK FUNC
 **************************************************************************************************/
bool pre_ina233_read(uint8_t sensor_num, void *args)
{
	CHECK_NULL_ARG_WITH_RETURN(args, false);

	// Select Channel
	bool ret = true;
	int mutex_status = 0;
	mux_config *pre_args = (mux_config *)args;
	pre_args->bus = sensor_config[sensor_config_index_map[sensor_num]].port;

	struct k_mutex *mutex = get_i2c_mux_mutex(pre_args->bus);
	mutex_status = k_mutex_lock(mutex, K_MSEC(MUTEX_LOCK_INTERVAL_MS));
	if (mutex_status != 0) {
		LOG_ERR("Mutex lock fail, status: %d", mutex_status);
		return false;
	}

	ret = set_mux_channel(*pre_args);
	return ret;
}

bool post_ina233_read(uint8_t sensor_num, void *args, int *reading)
{
	ARG_UNUSED(reading);
	ARG_UNUSED(args);

	int unlock_status = 0;
	uint8_t bus = sensor_config[sensor_config_index_map[sensor_num]].port;

	struct k_mutex *mutex = get_i2c_mux_mutex(bus);
	if (mutex->lock_count != 0) {
		unlock_status = k_mutex_unlock(mutex);
	}

	if (unlock_status != 0) {
		LOG_ERR("Mutex unlock fail, status: %d", unlock_status);
		return false;
	}

	return true;
}

bool pre_xdpe15284_read(uint8_t sensor_num, void *args)
{
	CHECK_NULL_ARG_WITH_RETURN(args, false);

	bool ret = false;
	int retry = 3;
	int mutex_status = 0;
	I2C_MSG msg = { 0 };
	vr_page_cfg *xdpe15284_page = (vr_page_cfg *)args;

	mutex_status = k_mutex_lock(&xdpe15284_mutex, K_MSEC(MUTEX_LOCK_INTERVAL_MS));
	if (mutex_status != 0) {
		LOG_ERR("Mutex lock fail, status: %d", mutex_status);
		return false;
	}

	msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
	msg.tx_len = 2;
	msg.data[0] = PMBUS_PAGE;
	msg.data[1] = xdpe15284_page->vr_page;

	ret = i2c_master_write(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Set xdpe15284 page fail, ret: %d", ret);
		return false;
	}

	return true;
}

bool post_xdpe15284_read(uint8_t sensor_num, void *args, int *reading)
{
	ARG_UNUSED(reading);
	ARG_UNUSED(args);

	int unlock_status = k_mutex_unlock(&xdpe15284_mutex);
	if (unlock_status != 0) {
		LOG_ERR("Mutex unlock fail, status: %d", unlock_status);
		return false;
	}

	return true;
}

bool pre_pex89000_read(uint8_t sensor_num, void *args)
{
	CHECK_NULL_ARG_WITH_RETURN(args, false);

	/* Can not access i2c mux and PEX89000 when DC off */
	if (is_mb_dc_on() == false) {
		return false;
	}

	bool ret = true;
	int mutex_status = 0;
	mux_config *pre_args = (mux_config *)args;
	pre_args->bus = sensor_config[sensor_config_index_map[sensor_num]].port;

	struct k_mutex *mutex = get_i2c_mux_mutex(pre_args->bus);
	mutex_status = k_mutex_lock(mutex, K_MSEC(MUTEX_LOCK_INTERVAL_MS));
	if (mutex_status != 0) {
		LOG_ERR("Mutex lock fail, status: %d", mutex_status);
		return false;
	}

	ret = set_mux_channel(*pre_args);
	return ret;
}

bool post_pex89000_read(uint8_t sensor_num, void *args, int *reading)
{
	ARG_UNUSED(reading);
	ARG_UNUSED(args);

	int unlock_status = 0;
	uint8_t bus = sensor_config[sensor_config_index_map[sensor_num]].port;

	struct k_mutex *mutex = get_i2c_mux_mutex(bus);
	if (mutex->lock_count != 0) {
		unlock_status = k_mutex_unlock(mutex);
	}

	if (unlock_status != 0) {
		LOG_ERR("Mutex unlock fail, status: %d", unlock_status);
		return false;
	}

	return true;
}
