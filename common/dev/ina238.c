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
#include "ina238.h"
#include "sensor.h"
#include "hal_i2c.h"
#include <logging/log.h>

#define I2C_RETRY 5
#define MSB_MASK BIT(15)
#define ADCRANGE_SET_TO_1 BIT(4)
#define SHUNT_CAL_MAX_VAL 0x7FFF
#define INTERNAL_FIXED_VALUE 819200000
#define ADCRANGE_1S_1 0x10
#define MAX_CURRENT_LSB 0x80
#define ADCRANGE_0_CONVERSION_FACTOR 0.000005 // 5uV/LSB
#define ADCRANGE_1_CONVERSION_FACTOR 0.00000125 // 1.25uV/LSB
#define VBUS_CONVERSION_FACTOR 0.000003125 // 3.125mV/LSB
#define DIETEMP_CONVERSION_FACTOR 0.125 // 125 m°C/LSB

LOG_MODULE_REGISTER(dev_ina238);

static uint16_t twoscomplement_to_decimal(uint16_t twoscomplement_val)
{
	if (twoscomplement_val & MSB_MASK) { // Check if MSB is 1 (negative number)
		twoscomplement_val =
			~twoscomplement_val + 1; // Two's complement operation for negative numbers
		return -twoscomplement_val; // Return negative number
	}

	return twoscomplement_val; // Return positive number as is
}

uint8_t ina238_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, SENSOR_INIT_UNSPECIFIED_ERROR);

	ina238_init_arg *init_args = (ina238_init_arg *)cfg->init_args;

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	uint16_t vshunt_val = 0;
	uint16_t vbus_val = 0;
	uint16_t dietemp_val = 0;
	uint16_t cur_val = 0;
	uint32_t pwr_val = 0;
	double val = 0.0;
	I2C_MSG msg = { 0 };
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	switch (cfg->offset) {
	case INA238_VSHUNT_OFFSET:
		/* Read shunt voltage */
		msg.tx_len = 1;
		msg.rx_len = 2;
		msg.data[0] = INA238_VSHUNT_OFFSET;
		if (i2c_master_read(&msg, I2C_RETRY)) {
			LOG_ERR("Failed to read shunt voltage from INA238");
			return SENSOR_FAIL_TO_ACCESS;
		}

		vshunt_val = (msg.data[0] << 8) | msg.data[1];
		if (init_args->adc_range)
			val = twoscomplement_to_decimal(vshunt_val) * ADCRANGE_1_CONVERSION_FACTOR;
		else
			val = twoscomplement_to_decimal(vshunt_val) * ADCRANGE_0_CONVERSION_FACTOR;

		break;
	case INA238_VBUS_OFFSET:
		/* Read bus voltage */
		msg.tx_len = 1;
		msg.rx_len = 2;
		msg.data[0] = INA238_VBUS_OFFSET;
		if (i2c_master_read(&msg, I2C_RETRY)) {
			LOG_ERR("Failed to read bus voltage from INA238");
			return SENSOR_FAIL_TO_ACCESS;
		}

		vbus_val = (msg.data[0] << 8) | msg.data[1];
		val = twoscomplement_to_decimal(vbus_val) * VBUS_CONVERSION_FACTOR;

		break;
	case INA238_DIETEMP_OFFSET:
		/* Read temperature  */
		msg.tx_len = 1;
		msg.rx_len = 2;
		msg.data[0] = INA238_DIETEMP_OFFSET;
		if (i2c_master_read(&msg, I2C_RETRY)) {
			LOG_ERR("Failed to read temperature from INA238");
			return SENSOR_FAIL_TO_ACCESS;
		}

		dietemp_val =
			(msg.data[0] << 4) | (msg.data[1] >> 4); // 15~4 total 12bits temperature
		val = twoscomplement_to_decimal(dietemp_val) * DIETEMP_CONVERSION_FACTOR;

		break;
	case INA238_CUR_OFFSET:
		/* Read current */
		// Current [A] = CURRENT_LSB x CURRENT
		msg.tx_len = 1;
		msg.rx_len = 2;
		msg.data[0] = INA238_CUR_OFFSET;
		if (i2c_master_read(&msg, I2C_RETRY)) {
			LOG_ERR("Failed to read current from INA238");
			return SENSOR_FAIL_TO_ACCESS;
		}

		cur_val = (msg.data[0] << 8) | msg.data[1];
		val = twoscomplement_to_decimal(cur_val) * (init_args->cur_lsb);
		break;
	case INA238_PWR_OFFSET:
		/* Read power */
		msg.tx_len = 1;
		msg.rx_len = 3;
		msg.data[0] = INA238_PWR_OFFSET;
		if (i2c_master_read(&msg, I2C_RETRY)) {
			LOG_ERR("Failed to read power from INA238");
			return SENSOR_FAIL_TO_ACCESS;
		}

		pwr_val = ((msg.data[0] << 16) | msg.data[1] << 8) | msg.data[2];
		val = 0.2 * twoscomplement_to_decimal(pwr_val) * (init_args->cur_lsb);

		break;
	default:
		LOG_ERR("Unknown offset: 0x%x", cfg->offset);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	sensor_val *sval = (sensor_val *)reading;
	sval->integer = (int)val & 0xFFFF;
	sval->fraction = (val - sval->integer) * 1000;

	return SENSOR_READ_SUCCESS;
}

uint8_t ina238_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	ina238_init_arg *init_args = (ina238_init_arg *)cfg->init_args;

	if (init_args->is_init)
		goto skip_init;

	I2C_MSG msg = { 0 };
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;

	/* read config reg */
	uint16_t config_val = 0x0000;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = INA238_CFG_OFFSET;
	config_val = msg.data[0] << 8 | msg.data[1];
	if (i2c_master_read(&msg, I2C_RETRY)) {
		LOG_ERR("Failed to read config from INA238 ");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	/* Configure the chip using default values */
	if (init_args->adc_range) {
		WRITE_BIT(config_val, 4, 1);
		msg.tx_len = 3;
		msg.data[0] = INA238_CFG_OFFSET;
		msg.data[1] = config_val >> 8;
		msg.data[2] = config_val & BIT_MASK(8);
		if (i2c_master_write(&msg, I2C_RETRY)) {
			LOG_ERR("Failed to init INA238 ");
			return SENSOR_INIT_UNSPECIFIED_ERROR;
		}
	}
	/* read alert reg */
	uint16_t alert_val = 0x0000;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = INA238_DIAG_ALRT_OFFSET;
	alert_val = msg.data[0] << 8 | msg.data[1];
	if (i2c_master_read(&msg, I2C_RETRY)) {
		LOG_ERR("Failed to read alert from INA238 ");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	/* enable alert latch */
	if (init_args->alert_latch) {
		WRITE_BIT(alert_val, 15, 1);
		msg.tx_len = 3;
		msg.data[0] = INA238_DIAG_ALRT_OFFSET;
		msg.data[1] = alert_val >> 8;
		msg.data[2] = alert_val & BIT_MASK(8);
		if (i2c_master_write(&msg, I2C_RETRY)) {
			LOG_ERR("Failed to enable alert latch in INA238 ");
			return SENSOR_INIT_UNSPECIFIED_ERROR;
		}
	}

	/* calculate Current_LSB */
	//Current_LSB = Maximum Expected Current/(2^15)
	init_args->cur_lsb = init_args->i_max / 32768.0;
	if (init_args->cur_lsb > MAX_CURRENT_LSB) {
		LOG_ERR("Current LSB is out of range");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	/* Write the calibration value */
	// SHUNT_CAL = 819.2 x 10^6 x CURRENT_LSB x R_SHUNT
	uint16_t shunt_cal = INTERNAL_FIXED_VALUE * (init_args->cur_lsb) * (init_args->r_shunt);

	// set conversion factor based on ADCRANGE setting
	if (init_args->adc_range)
		shunt_cal *= 4;

	if (shunt_cal > SHUNT_CAL_MAX_VAL) {
		LOG_ERR("Shunt calibration value is out of range");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	msg.tx_len = 3;
	msg.data[0] = INA238_SHUNT_CAL_OFFSET;
	msg.data[1] = shunt_cal >> 8; // high 8 bits
	msg.data[2] = shunt_cal & BIT_MASK(8); // low 8 bits
	if (i2c_master_write(&msg, I2C_RETRY)) {
		LOG_ERR("Failed to write the calibration value in INA238 ");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	init_args->is_init = true;

skip_init:
	cfg->read = ina238_read;
	return SENSOR_INIT_SUCCESS;
}
