/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *	 http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include <string.h>
#include "sensor.h"
#include "hal_i2c.h"
#include "pmbus.h"

#include <logging/log.h>

LOG_MODULE_REGISTER(adc128d818);

#define ADC128_REG_CONFIG 0x00
#define ADC128_REG_CONFIG_ADV 0x0b
#define ADC128_REG_IN(nr) (0x20 + (nr))

#define ADC128_VREF 2.56 // unit: volt(V)

uint8_t adc128d818_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	adc128d818_init_arg *init_arg = (adc128d818_init_arg *)cfg->init_args;
	int ret = 0;
	float val = 0;
	sensor_val *sval = (sensor_val *)reading;

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	uint8_t retry = 5;
	I2C_MSG msg;
	memset(&msg, 0, sizeof(I2C_MSG));

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = ADC128_REG_IN(cfg->offset);

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("i2c read fail  sensor number 0x%x  ret: %d", cfg->num, ret);
		return SENSOR_FAIL_TO_ACCESS;
	}

	/* Get reading */
	val = (msg.data[1] | msg.data[0] << 8) >> 4;
	val *= (init_arg->vref / 4096);
	val *= init_arg->scalefactor[cfg->offset];
	sval->integer = val;
	sval->fraction = (val - sval->integer) * 1000;

	return SENSOR_READ_SUCCESS;
}

uint8_t adc128d818_write(sensor_cfg *cfg, uint8_t reg, uint8_t value)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);

	int ret = 0;
	uint8_t retry = 5;
	I2C_MSG msg;

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	memset(&msg, 0, sizeof(I2C_MSG));

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 2;
	msg.data[0] = reg;
	msg.data[1] = value;

	ret = i2c_master_write(&msg, retry);
	if (ret != 0) {
		LOG_ERR("i2c write fail, sensor number 0x%x, register: 0x%x, ret: %d", cfg->num,
			reg, ret);
		return SENSOR_FAIL_TO_ACCESS;
	}

	return 0;
}

uint8_t adc128d818_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	adc128d818_init_arg *init_arg = (adc128d818_init_arg *)cfg->init_args;
	if (init_arg->is_init != true) {
		int ret = 0;
		uint8_t regval = 0x0;

		/* Reset chip to defaults */
		ret = adc128d818_write(cfg, ADC128_REG_CONFIG, 0x80);
		if (ret != 0) {
			return SENSOR_FAIL_TO_ACCESS;
		}
		/* TODO support different operation mode */

		/* If external vref is selected, configure the chip to use it */
		if (init_arg->external_vref) {
			init_arg->vref = init_arg->vref / 1000;
			regval |= 0x01;
		} else {
			init_arg->vref = ADC128_VREF;
		}

		if (regval != 0x0) {
			ret = adc128d818_write(cfg, ADC128_REG_CONFIG_ADV, regval);
			if (ret != 0) {
				return SENSOR_FAIL_TO_ACCESS;
			}
		}

		/* Start monitoring */
		ret = adc128d818_write(cfg, ADC128_REG_CONFIG, 0x1);
		if (ret != 0) {
			return SENSOR_FAIL_TO_ACCESS;
		}

		init_arg->is_init = true;
	}

	cfg->read = adc128d818_read;
	return SENSOR_INIT_SUCCESS;
}
