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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "sensor.h"
#include "hal_i2c.h"
#include "libutil.h"
#include "pmbus.h"

#define I2C_DATA_SIZE 5

#define MP5990_EIN_ROLLOVER_CNT_MAX 0x100
#define MP5990_EIN_SAMPLE_CNT_MAX 0x1000000
#define MP5990_EIN_ENERGY_CNT_MAX 0x8000

LOG_MODULE_REGISTER(dev_mp5990);

int mp5990_read_ein(double *val, uint8_t sensor_num)
{
	if ((val == NULL) || (sensor_num > SENSOR_NUM_MAX)) {
		return -1;
	}
	I2C_MSG msg;
	uint8_t retry = 5;
	uint32_t energy = 0, rollover = 0, sample = 0;
	uint32_t pre_energy = 0, pre_rollover = 0, pre_sample = 0;
	uint32_t sample_diff = 0;
	double energy_diff = 0;
	static uint32_t last_energy = 0, last_rollover = 0, last_sample = 0;
	static bool pre_ein = false;

	msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
	msg.tx_len = 1;
	msg.data[0] = sensor_config[sensor_config_index_map[sensor_num]].offset;
	msg.rx_len = 7;

	if (i2c_master_read(&msg, retry)) {
		LOG_WRN("i2c read failed.\n");
		return SENSOR_FAIL_TO_ACCESS;
	}

	//record the previous data
	pre_energy = last_energy;
	pre_rollover = last_rollover;
	pre_sample = last_sample;

	//record the current data
	last_energy = energy = (msg.data[2] << 8) | msg.data[1];
	last_rollover = rollover = msg.data[3];
	last_sample = sample = (msg.data[6] << 16) | (msg.data[5] << 8) | msg.data[4];

	//return since data isn't enough
	if (pre_ein == false) {
		pre_ein = true;
		return -1;
	}

	if ((pre_rollover > rollover) || ((pre_rollover == rollover) && (pre_energy > energy))) {
		rollover += MP5990_EIN_ROLLOVER_CNT_MAX;
	}

	if (pre_sample > sample) {
		sample += MP5990_EIN_SAMPLE_CNT_MAX;
	}

	energy_diff = (double)(rollover - pre_rollover) * MP5990_EIN_ENERGY_CNT_MAX +
		      (double)energy - (double)pre_energy;
	if (energy_diff < 0) {
		LOG_DBG("Energy difference is less than zero.\n");
		return -1;
	}

	sample_diff = sample - pre_sample;
	if (sample_diff == 0) {
		LOG_DBG("Sample difference is less than zero.\n");
		return -1;
	}

	*val = (double)(energy_diff / sample_diff);

	return 0;
}

uint8_t mp5990_read(uint8_t sensor_num, int *reading)
{
	if ((reading == NULL) || (sensor_num > SENSOR_NUM_MAX) ||
	    (sensor_config[sensor_config_index_map[sensor_num]].init_args == NULL)) {
		return SENSOR_UNSPECIFIED_ERROR;
	}

	mp5990_init_arg *init_arg =
		(mp5990_init_arg *)sensor_config[sensor_config_index_map[sensor_num]].init_args;
	if (init_arg->is_init == false) {
		printf("[%s], device isn't initialized\n", __func__);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	uint8_t retry = 5;
	double val;
	I2C_MSG msg = { 0 };
	int ret = 0;

	sensor_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = cfg->offset;

	if (i2c_master_read(&msg, retry))
		return SENSOR_FAIL_TO_ACCESS;

	switch (cfg->offset) {
	case PMBUS_READ_EIN:
		ret = mp5990_read_ein(&val, sensor_num);
		if (ret != 0) {
			return SENSOR_UNSPECIFIED_ERROR;
		}
		break;
	case PMBUS_READ_VIN:
	case PMBUS_READ_VOUT:
		/* 31.25 mv/LSB */
		val = ((msg.data[1] << 8) | msg.data[0]) * 31.25 / 1000;
		break;
	case PMBUS_READ_IOUT:
		/* 62.5 mA/LSB */
		val = ((msg.data[1] << 8) | msg.data[0]) * 62.5 / 1000;
		break;
	case PMBUS_READ_TEMPERATURE_1:
		/* 1 degree c/LSB */
		val = msg.data[0];
		break;
	case PMBUS_READ_POUT:
	case PMBUS_READ_PIN:
		/* 1 W/LSB */
		val = ((msg.data[1] << 8) | msg.data[0]);
		break;
	default:
		return SENSOR_NOT_FOUND;
	}

	sensor_val *sval = (sensor_val *)reading;
	memset(sval, 0, sizeof(*sval));

	sval->integer = (int32_t)val;
	sval->fraction = (int32_t)(val * 1000) % 1000;

	return SENSOR_READ_SUCCESS;
}

uint8_t mp5990_init(uint8_t sensor_num)
{
	if (sensor_num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	if (!sensor_config[sensor_config_index_map[sensor_num]].init_args) {
		printf("<error> MP5990 init args are not provided!\n");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	mp5990_init_arg *init_args =
		(mp5990_init_arg *)sensor_config[sensor_config_index_map[sensor_num]].init_args;
	if (init_args->is_init)
		goto skip_init;

	uint8_t retry = 5;
	I2C_MSG msg;
	char *data = (uint8_t *)malloc(I2C_DATA_SIZE * sizeof(uint8_t));
	if (data == NULL) {
		printf("[%s], Memory allocation failed!\n", __func__);
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}
	uint8_t bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	uint8_t target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
	uint8_t tx_len, rx_len;

	/* Skip setting iout_cal_gain if given 0xFFFF */
	if ((init_args->iout_cal_gain & 0xFFFF) != 0xFFFF) {
		tx_len = 3;
		rx_len = 0;
		data[0] = PMBUS_IOUT_CAL_GAIN;
		data[1] = init_args->iout_cal_gain & 0xFF;
		data[2] = (init_args->iout_cal_gain >> 8) & 0xFF;
		msg = construct_i2c_message(bus, target_addr, tx_len, data, rx_len);
		if (i2c_master_write(&msg, retry) != 0) {
			printf("Failed to write MP5990 register(0x%x)\n", data[0]);
			goto cleanup;
		}
	}

	/* Skip setting iout_oc_fault_limit if given 0xFFFF */
	if ((init_args->iout_oc_fault_limit & 0xFFFF) != 0xFFFF) {
		tx_len = 3;
		rx_len = 0;
		data[0] = PMBUS_IOUT_OC_FAULT_LIMIT;
		data[1] = init_args->iout_oc_fault_limit & 0xFF;
		data[2] = (init_args->iout_oc_fault_limit >> 8) & 0xFF;
		msg = construct_i2c_message(bus, target_addr, tx_len, data, rx_len);
		if (i2c_master_write(&msg, retry) != 0) {
			printf("Failed to write MP5990 register(0x%x)\n", data[0]);
			goto cleanup;
		}
	}

	init_args->is_init = true;

cleanup:
	SAFE_FREE(data);

skip_init:
	sensor_config[sensor_config_index_map[sensor_num]].read = mp5990_read;
	return SENSOR_INIT_SUCCESS;
}
