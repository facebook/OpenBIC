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
#include <stdlib.h>
#include <logging/log.h>
#include "sensor.h"
#include "pmbus.h"
#include "libutil.h"
#include "hal_i2c.h"
#include "plat_dev.h"
#include "plat_hook.h"
#include "plat_class.h"
#include "common_i2c_mux.h"
#include "plat_sensor_table.h"

LOG_MODULE_REGISTER(plat_dev);

#define NVME_NOT_AVAILABLE 0x80
#define NVME_TEMP_SENSOR_FAILURE 0x81
#define NVME_DRIVE_NOT_READY_BIT BIT(6)

#define INA233_CALIBRATION_OFFSET 0xD4

bool pal_sensor_drive_init(uint8_t card_id, sensor_cfg *cfg, uint8_t *init_status)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(init_status, false);

	switch (cfg->type) {
	case sensor_dev_nvme:
		*init_status = pal_nvme_init(card_id, cfg);
		break;
	case sensor_dev_ina233:
		*init_status = pal_ina233_init(card_id, cfg);
		break;
	default:
		LOG_ERR("Invalid initial drive type: 0x%x", cfg->type);
		return false;
	}

	return true;
}

bool pal_sensor_drive_read(uint8_t card_id, sensor_cfg *cfg, int *reading, uint8_t *sensor_status)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(reading, false);
	CHECK_NULL_ARG_WITH_RETURN(sensor_status, false);

	switch (cfg->type) {
	case sensor_dev_nvme:
		*sensor_status = pal_nvme_read(card_id, cfg, reading);
		break;
	case sensor_dev_ina233:
		*sensor_status = pal_ina233_read(card_id, cfg, reading);
		break;
	default:
		LOG_ERR("Invalid reading drive type: 0x%x", cfg->type);
		return false;
	}

	return true;
}

uint8_t pal_nvme_read(uint8_t card_id, sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor nvme 0x%x input parameter is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	int ret = 0;
	int bus = 0;
	uint8_t retry = 5;
	uint8_t nvme_status = 0;
	uint16_t val = 0;
	I2C_MSG msg = { 0 };

	bus = get_accl_bus(card_id, cfg->num);
	if (bus < 0) {
		return SENSOR_UNSPECIFIED_ERROR;
	}

	msg.bus = bus;
	msg.target_addr = cfg->target_addr;
	msg.data[0] = cfg->offset;
	msg.tx_len = 1;

	if (cfg->offset == NVME_TEMP_OFFSET) {
		msg.rx_len = 4;
	} else {
		msg.rx_len = 2;
	}

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("nvme i2c read fail ret: %d, offset: 0x%x, addr: 0x%x", ret, cfg->offset,
			msg.target_addr);
		return SENSOR_FAIL_TO_ACCESS;
	}

	sensor_val *sval = (sensor_val *)reading;
	switch (cfg->offset) {
	case NVME_TEMP_OFFSET:
		nvme_status = msg.data[1];
		val = msg.data[3];

		/* Check SSD drive ready */
		if ((nvme_status & NVME_DRIVE_NOT_READY_BIT) != 0) {
			return SENSOR_NOT_ACCESSIBLE;
		}

		/* Check reading value */
		switch (val) {
		case NVME_NOT_AVAILABLE:
			return SENSOR_FAIL_TO_ACCESS;
		case NVME_TEMP_SENSOR_FAILURE:
			return SENSOR_UNSPECIFIED_ERROR;
		default:
			break;
		}

		sval->integer = (int8_t)val;
		sval->fraction = 0;
		return SENSOR_READ_SUCCESS;
	case NVME_VOLTAGE_RAIL_1_OFFSET:
	case NVME_VOLTAGE_RAIL_2_OFFSET:
		// 100 uV/LSB
		val = ((msg.data[0] << 8) | msg.data[1]) / 10;

		sval->integer = (val / 1000) & 0xFFFF;
		sval->fraction = (val - (sval->integer * 1000)) & 0xFFFF;
		return SENSOR_READ_SUCCESS;
	default:
		LOG_ERR("Invalid offset: 0x%x", cfg->offset);
		return SENSOR_PARAMETER_NOT_VALID;
	}
}

uint8_t pal_nvme_init(uint8_t card_id, sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	return SENSOR_INIT_SUCCESS;
}

uint8_t pal_ina233_read(uint8_t card_id, sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor 0x%x input parameter is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	ina233_init_arg *init_arg = NULL;
	if (cfg->init_args == NULL) {
		init_arg = get_accl_init_sensor_config(card_id, cfg->num);
		CHECK_NULL_ARG_WITH_RETURN(init_arg, SENSOR_UNSPECIFIED_ERROR);
	} else {
		init_arg = cfg->init_args;
	}

	if (init_arg->is_init != true) {
		LOG_ERR("device isn't initialized");
		return SENSOR_UNSPECIFIED_ERROR;
	}

	uint8_t retry = 5;
	int bus = 0;
	int ret = 0;
	int16_t val = 0;
	float parameter = 0;
	I2C_MSG msg;
	memset(&msg, 0, sizeof(I2C_MSG));
	*reading = 0;

	bus = get_accl_bus(card_id, cfg->num);
	if (bus < 0) {
		return SENSOR_UNSPECIFIED_ERROR;
	}

	msg.bus = bus;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = cfg->offset;

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("i2c read fail ret: %d", ret);
		return SENSOR_FAIL_TO_ACCESS;
	}
	uint8_t offset = cfg->offset;
	val = (msg.data[1] << 8) | msg.data[0];
	sensor_val *sval = (sensor_val *)reading;
	switch (offset) {
	case PMBUS_READ_VOUT:
		// 1 mV/LSB, unsigned integer
		// m = 8 , b = 0 , r = 2
		// voltage convert formula = ((val / 100) - 0) / 8
		parameter = 800;
		break;
	case PMBUS_READ_IOUT:
		if (GETBIT(msg.data[1], 7)) {
			// If raw value is negative, set it zero.
			val = 0;
		}
		// 1 mA/LSB, 2's complement
		// current convert formula = val / (1 / current_lsb)
		parameter = (1 / init_arg->current_lsb);
		break;
	case PMBUS_READ_POUT:
		// 1 Watt/LSB, 2's complement
		// power convert formula = val / (1 / (current_lsb * 25))
		parameter = (1 / (init_arg->current_lsb * 25));
		break;
	default:
		LOG_ERR("Offset not supported: 0x%x", offset);
		return SENSOR_FAIL_TO_ACCESS;
		break;
	}

	sval->integer = val / parameter;
	sval->fraction = ((val / parameter) - sval->integer) * 1000;
	return SENSOR_READ_SUCCESS;
}

uint8_t pal_ina233_init(uint8_t card_id, sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	ina233_init_arg *init_arg = NULL;

	if (cfg->init_args == NULL) {
		init_arg = get_accl_init_sensor_config(card_id, cfg->num);
		CHECK_NULL_ARG_WITH_RETURN(init_arg, SENSOR_INIT_UNSPECIFIED_ERROR);
	} else {
		init_arg = cfg->init_args;
	}

	if (init_arg->is_init != true) {
		int ret = 0;
		int bus = 0;
		int retry = 5;
		uint16_t calibration = 0;
		I2C_MSG msg = { 0 };

		bus = get_accl_bus(card_id, cfg->num);
		if (bus < 0) {
			return SENSOR_INIT_UNSPECIFIED_ERROR;
		}

		msg.bus = bus;
		msg.target_addr = cfg->target_addr;
		msg.tx_len = 3;
		msg.data[0] = INA233_CALIBRATION_OFFSET;

		// Calibration formula = (0.00512 / (current_lsb * r_shunt))
		calibration =
			(uint16_t)((0.00512 / (init_arg->current_lsb * init_arg->r_shunt)) + 0.5);
		memcpy(&msg.data[1], &calibration, sizeof(uint16_t));

		ret = i2c_master_write(&msg, retry);
		if (ret != 0) {
			LOG_ERR("i2c write fail ret: %d", ret);
			return SENSOR_INIT_UNSPECIFIED_ERROR;
		}
		init_arg->is_init = true;
	}

	return SENSOR_INIT_SUCCESS;
}
