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
#include "libutil.h"
#include "sensor.h"
#include "plat_hook.h"
#include "common_i2c_mux.h"
#include "i2c-mux-pca954x.h"
#include "plat_sensor_table.h"

LOG_MODULE_REGISTER(plat_hook);

/**************************************************************************************************
 * INIT ARGS
**************************************************************************************************/
mp5990_init_arg mp5990_init_args[] = {
	[0] = { .is_init = false,
		.iout_cal_gain = 0x0280,
		.iout_oc_fault_limit = 0x005F,
		.ocw_sc_ref = 0x05A0 },
};

adc_asd_init_arg adc_asd_init_args[] = { [0] = { .is_init = false } };

sq52205_init_arg sq52205_init_args[] = {
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
	[12] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001 },
	[13] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001 }
};

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK ARGS
 **************************************************************************************************/
mux_config bus_3_pca9546_configs[] = {
	[0] = { .target_addr = 0x70, .channel = PCA9546A_CHANNEL_0 },
	[1] = { .target_addr = 0x70, .channel = PCA9546A_CHANNEL_1 },
	[2] = { .target_addr = 0x70, .channel = PCA9546A_CHANNEL_2 },
	[3] = { .target_addr = 0x70, .channel = PCA9546A_CHANNEL_3 },
};

mux_config bus_4_pca9548_configs[] = {
	[0] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_0 },
	[1] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_1 },
	[2] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_2 },
	[3] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_3 },
};

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK FUNC
 **************************************************************************************************/
bool pre_nvme_read(uint8_t sensor_num, void *args)
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

bool post_nvme_read(uint8_t sensor_num, void *args, int *reading)
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

bool pre_sq52205_read(uint8_t sensor_num, void *args)
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

bool post_sq52205_read(uint8_t sensor_num, void *args, int *reading)
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
