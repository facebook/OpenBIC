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

#include "hal_i2c.h"
#include "pmbus.h"
#include "sensor.h"
#include <logging/log.h>
#include <stdio.h>
#include <string.h>

#define REG_PWR_MT_CFG 0xD4

#define ADM1278_EIN_ROLLOVER_CNT_MAX 0x10000
#define ADM1278_EIN_SAMPLE_CNT_MAX 0x1000000
#define ADM1278_EIN_ENERGY_CNT_MAX 0x800000

LOG_MODULE_REGISTER(dev_adm1278);

int adm1278_read_ein_ext(float rsense, float *val, uint8_t sensor_num)
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
	msg.rx_len = 8;

	if (i2c_master_read(&msg, retry)) {
		LOG_WRN("i2c read failed.");
		return SENSOR_FAIL_TO_ACCESS;
	}

	//record the previous data
	pre_energy = last_energy;
	pre_rollover = last_rollover;
	pre_sample = last_sample;

	//record the current data
	last_energy = energy = (msg.data[2] << 16) | (msg.data[1] << 8) | msg.data[0];
	last_rollover = rollover = (msg.data[4] << 8) | msg.data[3];
	last_sample = sample = (msg.data[7] << 16) | (msg.data[6] << 8) | msg.data[5];

	//return since data isn't enough
	if (pre_ein == false) {
		pre_ein = true;
		return -1;
	}

	if ((pre_rollover > rollover) || ((pre_rollover == rollover) && (pre_energy > energy))) {
		rollover += ADM1278_EIN_ROLLOVER_CNT_MAX;
	}

	if (pre_sample > sample) {
		sample += ADM1278_EIN_SAMPLE_CNT_MAX;
	}

	energy_diff = (double)(rollover - pre_rollover) * ADM1278_EIN_ENERGY_CNT_MAX +
		      (double)energy - (double)pre_energy;
	if (energy_diff < 0) {
		LOG_DBG("Energy difference is less than zero.");
		return -1;
	}

	sample_diff = sample - pre_sample;
	if (sample_diff == 0) {
		LOG_DBG("Sample difference is less than zero.");
		return -1;
	}

	*val = (float)((energy_diff / sample_diff / 256) * 100) / (6123 * rsense);

	return 0;
}

uint8_t adm1278_read(uint8_t sensor_num, int *reading)
{
	if (reading == NULL) {
		LOG_ERR("Reading pointer passed in as NULL.");
		return SENSOR_UNSPECIFIED_ERROR;
	} else if (sensor_num > SENSOR_NUM_MAX) {
		LOG_ERR("Sensor number out of range.");
		return SENSOR_UNSPECIFIED_ERROR;
	} else if ((sensor_config[sensor_config_index_map[sensor_num]].init_args == NULL)) {
		LOG_ERR("Init args for sensor are NULL.");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	adm1278_init_arg *init_arg =
		(adm1278_init_arg *)sensor_config[sensor_config_index_map[sensor_num]].init_args;
	if (init_arg->is_init == false) {
		LOG_ERR("adm1278_read, device isn't initialized");
		return SENSOR_UNSPECIFIED_ERROR;
	}

	if (!init_arg->r_sense) {
		LOG_ERR("adm1278_read, Rsense not provided");
		return SENSOR_UNSPECIFIED_ERROR;
	}

	float Rsense = init_arg->r_sense;
	float val;
	uint8_t retry = 5;
	I2C_MSG msg;
	int ret = 0;
	uint8_t offset = sensor_config[sensor_config_index_map[sensor_num]].offset;

	if (offset != ADM1278_EIN_EXT_OFFSET) {
		msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
		msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
		msg.tx_len = 1;
		msg.data[0] = offset;
		msg.rx_len = 2;

		if (i2c_master_read(&msg, retry))
			return SENSOR_FAIL_TO_ACCESS;
	}

	switch (offset) {
	case PMBUS_READ_VIN:
		// m = +19599, b = 0, R = -2
		val = (float)((msg.data[1] << 8) | msg.data[0]) * 100 / 19599;
		break;

	case PMBUS_READ_IOUT:
	case ADM1278_PEAK_IOUT_OFFSET:
		// m = +800 * Rsense(mohm), b = +20475, R = -1
		val = (float)(((msg.data[1] << 8) | msg.data[0]) * 10 - 20475) / (800 * Rsense);
		break;

	case PMBUS_READ_TEMPERATURE_1:
		// m = +42, b = +31880, R = -1
		val = (float)(((msg.data[1] << 8) | msg.data[0]) * 10 - 31880) / 42;
		break;

	case PMBUS_READ_PIN:
	case ADM1278_PEAK_PIN_OFFSET:
		// m = +6123 * Rsense(mohm), b = 0, R = -2
		val = (float)(((msg.data[1] << 8) | msg.data[0]) * 100) / (6123 * Rsense);
		break;

	case ADM1278_EIN_EXT_OFFSET:
		ret = adm1278_read_ein_ext(Rsense, &val, sensor_num);
		if (ret != 0) {
			return SENSOR_UNSPECIFIED_ERROR;
		}
		break;

	default:
		LOG_WRN("Invalid sensor 0x%x", sensor_num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	sensor_val *sval = (sensor_val *)reading;
	sval->integer = (int)val & 0xFFFF;
	sval->fraction = (val - sval->integer) * 1000;
	return SENSOR_READ_SUCCESS;
}

uint8_t adm1278_init(uint8_t sensor_num)
{
	if (sensor_num > SENSOR_NUM_MAX) {
		LOG_ERR("Sensor number out of range.");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	if (!sensor_config[sensor_config_index_map[sensor_num]].init_args) {
		LOG_ERR("ADM1278 init args not provide!");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	adm1278_init_arg *init_args =
		(adm1278_init_arg *)sensor_config[sensor_config_index_map[sensor_num]].init_args;
	if (init_args->is_init) {
		LOG_DBG("adm1278 already initialized.");
		goto skip_init;
	}

	uint8_t retry = 5;
	I2C_MSG msg;
	msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
	msg.tx_len = 3;
	msg.data[0] = REG_PWR_MT_CFG;
	msg.data[1] = init_args->config.value & 0xFF;
	msg.data[2] = (init_args->config.value >> 8) & 0xFF;

	if (i2c_master_write(&msg, retry)) {
		LOG_ERR("ADM1278 initial failed while i2c writing");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	memset(&msg, 0, sizeof(msg));
	msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
	msg.tx_len = 1;
	msg.data[0] = REG_PWR_MT_CFG;
	msg.rx_len = 2;

	if (i2c_master_read(&msg, retry)) {
		LOG_ERR("ADM1278 failed while reading sensor");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	if ((msg.data[0] != (init_args->config.value & 0xFF)) ||
	    (msg.data[1] != ((init_args->config.value >> 8) & 0xFF))) {
		LOG_ERR("ADM1278 initialization failed, invalid data returned");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}
	init_args->is_init = 1;

skip_init:
	sensor_config[sensor_config_index_map[sensor_num]].read = adm1278_read;
	return SENSOR_INIT_SUCCESS;
}
