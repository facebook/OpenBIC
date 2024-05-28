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
#include "sensor.h"
#include "hal_i2c.h"
#include "ads112c.h"

LOG_MODULE_REGISTER(dev_ads112c);

uint8_t ads112c_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_UNSPECIFIED_ERROR;
	}

	uint8_t retry = 10;
	uint8_t i2c_retry = 3;
	I2C_MSG msg;

	int i = 0;
	for (i = 0; i < retry; i++) {
		msg.bus = cfg->port;
		msg.target_addr = cfg->target_addr;
		msg.tx_len = 1;
    
		msg.data[0] = CMD_RREG | CFG_REG_OFFSET2; //RREG command: 0010 rrxx (rr register address = 10 (0x02), DRDY -> Conversion result ready flag.)
		msg.rx_len = 1;

		int ret = i2c_master_read(&msg, i2c_retry);
		if (ret != 0) {
			LOG_ERR("Read register on ads112c failed");
			return SENSOR_UNSPECIFIED_ERROR;
		}

		if (msg.data[0] & 0x80) { //the 7th (DRDY) bit in Configuration Register 2
			break;
		} else {
			k_msleep(10);
			memset(&msg, 0, sizeof(msg));
			if (i == (retry - 1)) {
				LOG_ERR("Read register on ads112c failed");
				return SENSOR_UNSPECIFIED_ERROR;
			}
		}
	}

	memset(&msg, 0, sizeof(msg));

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.data[0] = CMD_RDATA; // Send the RDATA command (10h)
	msg.rx_len = 2; // read 2 bytes of conversion data

	if (i2c_master_read(&msg, i2c_retry)) {
		return SENSOR_FAIL_TO_ACCESS;
	}

	*reading = (msg.data[1] << 8) | msg.data[0];

	return SENSOR_READ_SUCCESS;
}

uint8_t ads112c_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	uint8_t i2c_retry = 3;
	I2C_MSG msg;
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.data[0] = CMD_RESET; // RESET Command 000 011x(06h)

	if (i2c_master_write(&msg, i2c_retry)) {
		LOG_ERR("Reset ADS112C failed to the default state");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	memset(&msg, 0, sizeof(msg));

	ads112c_init_arg *init_arg = (ads112c_init_arg *)cfg->init_args;

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 4;

	msg.data[0] =
		CMD_WREG | CFG_REG_OFFSET0; //WREG command: 0100 rrxx (rr register address = 00)
	msg.data[1] = init_arg->reg0_input | init_arg->reg0_gain |
		      init_arg->reg0_pga; //Configuration Register 0
	msg.data[2] =
		CMD_WREG |
		CFG_REG_OFFSET1; //WREG command: 0100 rrxx (rr register address = 01)(spec sample is 0x42)
	msg.data[3] =
		init_arg->reg1_conversion |
		init_arg->reg1_vol_refer; //Configuration Register 1: continuous conversion mode.

	if (i2c_master_write(&msg, i2c_retry)) {
		LOG_ERR("Write the respective register configurations failed");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	memset(&msg, 0, sizeof(msg));

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.data[0] = CMD_START_SYNC; // START/SYNC Command 000 100x(08h)
	if (i2c_master_write(&msg, i2c_retry)) {
		LOG_ERR("Failed to start conversion mode on ADS112C");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	cfg->read = ads112c_read;
	return SENSOR_INIT_SUCCESS;
}