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
#include "ast_adc.h"
#include "sensor.h"
#include "hal_i2c.h"
#include "plat_sensor_table.h"
#include "plat_hook.h"
#include <logging/log.h>
K_MUTEX_DEFINE(wait_pm8702_mutex);

LOG_MODULE_REGISTER(plat_hook);
/**************************************************************************************************
 * INIT ARGS
**************************************************************************************************/
adc_asd_init_arg adc_asd_init_args[] = { [0] = { .is_init = false } };

pm8702_dimm_init_arg pm8702_dimm_init_args[] = { [0] = { .is_init = false, .dimm_id = 0x41 },
						 [1] = { .is_init = false, .dimm_id = 0x42 },
						 [2] = { .is_init = false, .dimm_id = 0x43 },
						 [3] = { .is_init = false, .dimm_id = 0x44 } };

ina233_init_arg ina233_init_args[] = {
	[0] = {
	.is_init = false,
	.current_lsb = 0.001,
	.r_shunt = 0.005,
	.mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011,	//set 64 average times
		.rsvd = 0b0100,
	},
	},
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
	},
};

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK ARGS
 **************************************************************************************************/
/*typedef struct _isl69254iraz_t_pre_arg_ {
	uint8_t vr_page;
} isl69254iraz_t_pre_arg;*/
isl69254iraz_t_pre_arg isl69254iraz_t_pre_read_args[] = {
	[0] = { 0x0 },
	[1] = { 0x1 },
};

vr_pre_proc_arg vr_page_select[] = {
	[0] = { 0x0 },
	[1] = { 0x1 },
};

ina230_init_arg SQ5220x_init_args[] = {
	[0] = {
	.is_init = false,
	.config = {
		.MODE = 0b111,		// Measure voltage of shunt resistor and bus(default).
		.VSH_CT = 0b100,	// The Vshunt conversion time is 1.1ms(default).
		.VBUS_CT = 0b100,	// The Vbus conversion time is 1.1ms(default).
		.AVG = 0b000,		// Average number is 1(default).
	},
	.alt_cfg = {
		.LEN = 1,			// Alert Latch enabled.
		.POL = 1,			// Enable the Over-Limit Power alert function.
	},
	.r_shunt = 0.005,
	.alert_value = 16.0,	// Unit: Watt
	.i_max = 4.8
	},
	[1] = {
	.is_init = false,
	.config = {
		.MODE = 0b111,		// Measure voltage of shunt resistor and bus(default).
		.VSH_CT = 0b100,	// The Vshunt conversion time is 1.1ms(default).
		.VBUS_CT = 0b100,	// The Vbus conversion time is 1.1ms(default).
		.AVG = 0b000,		// Average number is 1(default).
	},
	.alt_cfg = {
		.LEN = 1,			// Alert Latch enabled.
		.POL = 1,			// Enable the Over-Limit Power alert function.
	},
	.r_shunt = 0.005,
	.alert_value = 16.0,	// Unit: Watt
	.i_max = 4.8
	},
};

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK FUNC
 **************************************************************************************************/
/* VR pre read function
 *
 * set mux and VR page
 *
 * @param sensor_num sensor number
 * @param args pointer to vr_pre_proc_arg
 * @param reading pointer to reading from previous step
 * @retval true if setting mux and page is successful.
 * @retval false if setting mux or page fails.
 */
bool pre_vr_read(uint8_t sensor_num, void *args)
{
	if (args == NULL) {
		return false;
	}

	vr_pre_proc_arg *vr_page_sel = (vr_pre_proc_arg *)args;
	uint8_t retry = 5;
	I2C_MSG msg;
	int ret = 0;

	/* set page */
	msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
	msg.tx_len = 2;
	msg.data[0] = 0x00;
	msg.data[1] = vr_page_sel->vr_page;

	ret = i2c_master_write(&msg, retry);
	if (ret != 0) {
		LOG_ERR("I2C write fail  ret: %d", ret);
		return false;
	}

	return true;
}

bool pre_isl69254iraz_t_read(uint8_t sensor_num, void *args)
{
	if (args == NULL) {
		LOG_ERR("Input args is NULL");
		return false;
	}

	isl69254iraz_t_pre_arg *pre_args = (isl69254iraz_t_pre_arg *)args;
	uint8_t retry = 5;
	int ret = 0;
	I2C_MSG msg;
	memset(&msg, 0, sizeof(I2C_MSG));

	/* set page */
	msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
	msg.tx_len = 2;
	msg.data[0] = VR_PAGE_OFFSET;
	msg.data[1] = pre_args->vr_page;

	ret = i2c_master_write(&msg, retry);
	if (ret != 0) {
		LOG_ERR("I2C write fail  ret: %d", ret);
		return false;
	}
	return true;
}

bool pre_pm8702_read(uint8_t sensor_num, void *args)
{
	if (k_mutex_lock(&wait_pm8702_mutex, K_MSEC(3000))) {
		LOG_WRN("pm8702 mutex is locked over 3000 ms!!\n");
		return false;
	}
	k_msleep(20);
	return true;
}

bool post_pm8702_read(uint8_t sensor_num, void *args, int *reading)
{
	k_mutex_unlock(&wait_pm8702_mutex);
	return true;
}
