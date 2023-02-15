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
#include <sensor.h>
#include "plat_hook.h"
#include "plat_i2c.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(plat_hook);

#define VOLTAGE_SELECT_BIT (1 << 2)
#define ADC_16BIT_MODE_BIT (1 << 0)

#define ADC_16BIT_MODE_DELAY_MS 1100
#define ADC_12BIT_MODE_DELAY_MS 100

/**************************************************************************************************
 * INIT ARGS
**************************************************************************************************/
adc_asd_init_arg adc_asd_init_args[] = { [0] = { .is_init = false } };

adm1278_init_arg adm1278_init_args[] = {
	[0] = { .is_init = false, .config = { 0x3F1C }, .r_sense = 0.25 }
};

ltc4282_init_arg ltc4282_init_args[] = { [0] = { .is_init = false, .r_sense_mohm = 0.1 } };

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK ARGS
 **************************************************************************************************/
ltc4282_pre_proc_arg ltc4282_pre_read_args[] = {
	[0] = { "VIN" },
	[1] = { "VOUT" },
};

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK FUNC
 **************************************************************************************************/
/* LTC4282 pre read function
 *
 * set Vsource
 *
 * @param sensor_num sensor number
 * @param args pointer to ltc4282_pre_proc_arg
 * @param reading pointer to reading from previous step
 * @retval true if setting is successful.
 * @retval false if setting failed.
 */
bool pre_ltc4282_read(uint8_t sensor_num, void *args)
{
	if (args == NULL) {
		return false;
	}

	ltc4282_pre_proc_arg *pre_proc_args = (ltc4282_pre_proc_arg *)args;
	uint8_t retry = 5;
	I2C_MSG msg;
	int val = 0;

	/* get adjust */
	msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
	msg.tx_len = 1;
	msg.data[0] = LTC4282_ILIM_ADJUST_OFFSET;
	msg.rx_len = 1;
	if (i2c_master_read(&msg, retry) != 0) {
		LOG_ERR("Get voltage adjust register fail");
		return false;
	}

	if (strcmp(pre_proc_args->vsource_status, "VIN") == 0) {
		val = msg.data[0] | VOLTAGE_SELECT_BIT;

	} else if (strcmp(pre_proc_args->vsource_status, "VOUT") == 0) {
		val = msg.data[0] & (~VOLTAGE_SELECT_BIT);

	} else {
		LOG_ERR("Unexpected vsource %s", log_strdup(pre_proc_args->vsource_status));
		return false;
	}

	msg.tx_len = 2;
	msg.data[0] = sensor_config[sensor_config_index_map[sensor_num]].offset;
	msg.data[1] = val;

	if (i2c_master_write(&msg, retry) != 0) {
		LOG_ERR("Set vsource fail");
		return false;
	}

	if (val & ADC_16BIT_MODE_BIT) {
		k_msleep(ADC_16BIT_MODE_DELAY_MS);
	} else {
		k_msleep(ADC_12BIT_MODE_DELAY_MS);
	}

	return true;
}
