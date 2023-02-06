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
#include "pmbus.h"
#include "plat_hook.h"
#include "common_i2c_mux.h"
#include "i2c-mux-pca954x.h"
#include "i2c-mux-pca984x.h"
#include "plat_sensor_table.h"
#include "ltc2991.h"
#include "plat_i2c.h"
#include "plat_ipmi.h"

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

ina233_init_arg ina233_init_args[] = {
	[0] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.005 },
	[1] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.005 },
};

ltc2991_init_arg ltc2991_init_args[] = {
	[0] = { .is_init = false,
		.v1_v4_control_operation.value = LTC2991_KEEP_DEFAULT_SETTING,
		.v5_v8_control_operation.value = LTC2991_KEEP_DEFAULT_SETTING },
	[1] = { .is_init = false,
		.v1_v4_control_operation.value = LTC2991_KEEP_DEFAULT_SETTING,
		.v5_v8_control_operation.value = LTC2991_KEEP_DEFAULT_SETTING },
};

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK ARGS
 **************************************************************************************************/
/** JCN 1~4, 9~12 CXL/E1.S 1 mux config **/
mux_config bus_2_pca9548_configs[] = {
	[0] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_0 },
	[1] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_1 },
	[2] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_2 },
	[3] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_3 },
	[4] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_4 },
	[5] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_5 },
	[6] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_6 },
	[7] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_7 },
};

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
	/** JCN 13~14 E1.S mux config **/
	[4] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_5 },
	[5] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_4 },
	[6] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_7 },
	[7] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_6 },
};

/** JCN 1~4, 9~12 E1.S 0 mux config **/
mux_config bus_8_pca9548_configs[] = {
	[0] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_0 },
	[1] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_1 },
	[2] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_2 },
	[3] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_3 },
	[4] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_4 },
	[5] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_5 },
	[6] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_6 },
	[7] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_7 },
};

mux_config cxl_mux_configs[] = {
	[0] = { .target_addr = 0x71, .channel = PCA9848_CHANNEL_0 },
	[1] = { .target_addr = 0x71, .channel = PCA9848_CHANNEL_1 },
	[2] = { .target_addr = 0x71, .channel = PCA9848_CHANNEL_2 },
	[3] = { .target_addr = 0x71, .channel = PCA9848_CHANNEL_3 },
	[4] = { .target_addr = 0x71, .channel = PCA9848_CHANNEL_4 },
	[5] = { .target_addr = 0x71, .channel = PCA9848_CHANNEL_5 },
};

vr_page_cfg vr_page_select[] = {
	[0] = { .vr_page = PMBUS_PAGE_0 },
	[1] = { .vr_page = PMBUS_PAGE_1 },
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

bool pre_e1s_switch_mux(uint8_t sensor_num, uint8_t card_id)
{
	mux_config e1s_mux = { 0 };
	bool ret = get_pcie_card_mux_config(PCIE_CARD_E1S, card_id, sensor_num, &e1s_mux, NULL);
	if (ret != true) {
		return ret;
	}

	int mutex_status = 0;
	struct k_mutex *mutex = get_i2c_mux_mutex(e1s_mux.bus);
	mutex_status = k_mutex_lock(mutex, K_MSEC(MUTEX_LOCK_INTERVAL_MS));
	if (mutex_status != 0) {
		LOG_ERR("Mutex lock fail, status: %d", mutex_status);
		return false;
	}

	ret = set_mux_channel(e1s_mux);
	if (ret != true) {
		LOG_ERR("Switch e1s mux fail");
		k_mutex_unlock(mutex);
		return false;
	}

	return true;
}

bool post_e1s_switch_mux(uint8_t sensor_num, uint8_t card_id)
{
	mux_config e1s_mux = { 0 };
	bool ret = get_pcie_card_mux_config(PCIE_CARD_E1S, card_id, sensor_num, &e1s_mux, NULL);
	if (ret != true) {
		return ret;
	}

	int unlock_status = 0;
	struct k_mutex *mutex = get_i2c_mux_mutex(e1s_mux.bus);
	unlock_status = k_mutex_unlock(mutex);
	if (unlock_status != 0) {
		LOG_ERR("Mutex unlock fail, status: %d", unlock_status);
		return false;
	}

	return true;
}

bool pre_cxl_switch_mux(uint8_t sensor_num, uint8_t card_id)
{
	mux_config card_mux = { 0 };
	mux_config cxl_mux = { 0 };
	bool ret =
		get_pcie_card_mux_config(PCIE_CARD_CXL, card_id, sensor_num, &card_mux, &cxl_mux);
	if (ret != true) {
		return ret;
	}

	int mutex_status = 0;
	struct k_mutex *mutex = get_i2c_mux_mutex(MEB_CXL_BUS);
	mutex_status = k_mutex_lock(mutex, K_MSEC(MUTEX_LOCK_INTERVAL_MS));
	if (mutex_status != 0) {
		LOG_ERR("Mutex lock fail, status: %d", mutex_status);
		return false;
	}

	// Switch card mux
	ret = set_mux_channel(card_mux);
	if (ret != true) {
		LOG_ERR("Switch card mux fail");
		k_mutex_unlock(mutex);
		return false;
	}

	// Switch cxl mux
	ret = set_mux_channel(cxl_mux);
	if (ret != true) {
		LOG_ERR("Switch cxl mux fail");
		k_mutex_unlock(mutex);
		return false;
	}

	return true;
}

bool post_cxl_switch_mux(uint8_t sensor_num, uint8_t card_id)
{
	int unlock_status = 0;
	struct k_mutex *mutex = get_i2c_mux_mutex(MEB_CXL_BUS);
	unlock_status = k_mutex_unlock(mutex);
	if (unlock_status != 0) {
		LOG_ERR("Mutex unlock fail, status: %d", unlock_status);
		return false;
	}

	return true;
}

bool pre_cxl_vr_read(uint8_t sensor_num, void *args)
{
	CHECK_NULL_ARG_WITH_RETURN(args, false);

	uint8_t index = 0;
	bool ret = get_cxl_sensor_config_index(sensor_num, &index);
	if (ret == false) {
		return false;
	}

	vr_page_cfg *vr_page_sel = (vr_page_cfg *)args;
	uint8_t retry = 5;
	I2C_MSG msg = { 0 };

	/* set page */
	msg.bus = plat_cxl_sensor_config[index].port;
	msg.target_addr = plat_cxl_sensor_config[index].target_addr;
	msg.tx_len = 2;
	msg.data[0] = 0x00;
	msg.data[1] = vr_page_sel->vr_page;

	if (i2c_master_write(&msg, retry)) {
		LOG_ERR("Set page fail");
		return false;
	}
	return true;
}

bool post_cxl_xdpe12284c_read(uint8_t sensor_num, void *args, int *reading)
{
	ARG_UNUSED(args);
	if (reading == NULL) {
		return check_reading_pointer_null_is_allowed(sensor_num);
	}

	sensor_val *sval = (sensor_val *)reading;
	float val = (float)sval->integer + (sval->fraction / 1000.0);
	uint8_t index = 0;
	bool ret = get_cxl_sensor_config_index(sensor_num, &index);
	if (ret == false) {
		return false;
	}

	switch (plat_cxl_sensor_config[index].offset) {
	case PMBUS_READ_IOUT:
		if (val < (-2)) {
			LOG_ERR("Sensor %x unexpected current reading", sensor_num);
			return false;
		}

		//the tolerance of current is -2 amps.
		if (val < 0) {
			sval->integer = 0;
			sval->fraction = 0;
		}
		break;
	case PMBUS_READ_POUT:
		if (val < (-4)) {
			LOG_ERR("Sensor %x unexpected power reading", sensor_num);
			return false;
		}

		//the tolerance of power is -4 watts.
		if (val < 0) {
			sval->integer = 0;
			sval->fraction = 0;
		}
		break;
	default:
		break;
	}

	return true;
}
