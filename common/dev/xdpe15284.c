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
#include "libutil.h"
#include "hal_i2c.h"
#include "util_pmbus.h"
#include "pmbus.h"

LOG_MODULE_REGISTER(xdpe15284);

#define XDPE15284_MFR_DISABLE_SECURITY_ONCE 0xCB
#define XDPE15284_MFR_FW_CMD_REG 0xFE
#define XDPE15284_MFR_FW_CMD_DATA_REG 0xFD
#define XDPE15284_MFR_FW_CMD_DATA_LEN 4
#define XDPE15284_REMAINING_WRITE_DATA_LEN 2
#define XDPE15284_NOP_CMD 0x00
#define XDPE15284_GET_REMAINING_WR_CMD 0x10
#define XDPE15284_CALCULATE_CRC_CMD 0x2D
#define XDPE15284_WAIT_DATA_DELAY_MS 10
#define XDPE15284C_CONF_SIZE 1344 // Config(604) + PMBus(504) + SVID(156) + PMBusPartial(80)

const uint32_t REG_LOCK_PASSWORD = 0x7F48680C;

bool xdpe15284_mfr_fw_operation(uint8_t bus, uint8_t addr, uint8_t cmd, uint8_t rx_len,
				uint8_t *buf)
{
	CHECK_NULL_ARG_WITH_RETURN(buf, false);

	uint8_t retry = 3;
	I2C_MSG i2c_msg = { 0 };

	/* Reset the MFR data register to default value */
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.tx_len = 2;
	i2c_msg.data[0] = XDPE15284_MFR_FW_CMD_REG;
	i2c_msg.data[1] = XDPE15284_NOP_CMD;

	if (i2c_master_write(&i2c_msg, retry)) {
		LOG_ERR("Reset data register fail, cmd: 0x%x", cmd);
		return false;
	}

	/* Send the command to get request data */
	memset(&i2c_msg, 0, sizeof(I2C_MSG));
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.tx_len = 2;
	i2c_msg.data[0] = XDPE15284_MFR_FW_CMD_REG;
	i2c_msg.data[1] = cmd;

	if (i2c_master_write(&i2c_msg, retry)) {
		LOG_ERR("Write command to register fail, cmd: 0x%x", cmd);
		return false;
	}

	k_msleep(XDPE15284_WAIT_DATA_DELAY_MS);

	/* Read data in MFR firmware command data register */
	memset(&i2c_msg, 0, sizeof(I2C_MSG));
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = rx_len + 1;
	i2c_msg.data[0] = XDPE15284_MFR_FW_CMD_DATA_REG;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Read data from register fail, cmd: 0x%x", cmd);
		return false;
	}

	memcpy(buf, &i2c_msg.data[1], rx_len);
	return true;
}

bool xdpe15284_get_checksum(uint8_t bus, uint8_t addr, uint8_t *checksum)
{
	CHECK_NULL_ARG_WITH_RETURN(checksum, false);

	bool ret = xdpe15284_mfr_fw_operation(bus, addr, XDPE15284_CALCULATE_CRC_CMD,
					      XDPE15284_MFR_FW_CMD_DATA_LEN, checksum);
	if (ret != true) {
		LOG_ERR("Get checksum fail, bus: 0x%x, addr: 0x%x", bus, addr);
		return ret;
	}

	reverse_array(checksum, XDPE15284_MFR_FW_CMD_DATA_LEN);
	return ret;
}

bool xdpe15284_get_remaining_wr(uint8_t bus, uint8_t addr, uint8_t *data)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);

	uint8_t buf[XDPE15284_REMAINING_WRITE_DATA_LEN] = { 0 };
	bool ret = xdpe15284_mfr_fw_operation(bus, addr, XDPE15284_GET_REMAINING_WR_CMD,
					      XDPE15284_REMAINING_WRITE_DATA_LEN, buf);
	if (ret != true) {
		LOG_ERR("Get remaining write count fail, bus: 0x%x, addr: 0x%x", bus, addr);
		return ret;
	}

	*data = ((buf[1] << 8 | buf[0]) / XDPE15284C_CONF_SIZE) & 0xFF;
	return ret;
}

uint8_t xdpe15284_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	uint8_t retry = 5;
	sensor_val *sval = (sensor_val *)reading;
	I2C_MSG msg;
	memset(sval, 0, sizeof(sensor_val));

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = cfg->offset;

	if (i2c_master_read(&msg, retry))
		return SENSOR_FAIL_TO_ACCESS;

	uint8_t offset = cfg->offset;
	if (offset == PMBUS_READ_VOUT) {
		/* ULINEAR16, get exponent from VOUT_MODE */
		float exponent;
		if (!get_exponent_from_vout_mode(cfg, &exponent))
			return SENSOR_FAIL_TO_ACCESS;

		float actual_value = ((msg.data[1] << 8) | msg.data[0]) * exponent;
		sval->integer = actual_value;
		sval->fraction = (actual_value - sval->integer) * 1000;
	} else if (offset == PMBUS_READ_IOUT || offset == PMBUS_READ_TEMPERATURE_1 ||
		   offset == PMBUS_READ_POUT) {
		/* SLINEAR11 */
		uint16_t read_value = (msg.data[1] << 8) | msg.data[0];
		float actual_value = slinear11_to_float(read_value);
		if (offset == PMBUS_READ_IOUT && actual_value < 0) {
			/* In the case POUT is 0, IOUT may read small negative value, replace this case with 0 */
			sval->integer = 0;
			sval->fraction = 0;
		} else {
			sval->integer = actual_value;
			sval->fraction = (actual_value - sval->integer) * 1000;
		}
	} else {
		return SENSOR_FAIL_TO_ACCESS;
	}

	return SENSOR_READ_SUCCESS;
}

bool xdpe15284_lock_reg(uint8_t bus, uint8_t addr)
{
	I2C_MSG i2c_msg = { 0 };
	int retry = 3;

	/* Reset the MFR data register to default value */
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.tx_len = 6;
	i2c_msg.data[0] = XDPE15284_MFR_DISABLE_SECURITY_ONCE;
	i2c_msg.data[1] = 0x04;
	i2c_msg.data[2] = 0x00;
	i2c_msg.data[3] = 0x00;
	i2c_msg.data[4] = 0x00;

	if (i2c_master_write(&i2c_msg, retry)) {
		LOG_ERR("Lock register fail");
		return false;
	}
	return true;
}

bool xdpe15284_unlock_reg(uint8_t bus, uint8_t addr)
{
	I2C_MSG i2c_msg = { 0 };
	int retry = 3;

	/* Reset the MFR data register to default value */
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.tx_len = 6;
	i2c_msg.data[0] = XDPE15284_MFR_DISABLE_SECURITY_ONCE;
	i2c_msg.data[1] = 0x04;
	memcpy(&(i2c_msg.data[2]), (uint8_t *)&REG_LOCK_PASSWORD, 4);

	if (i2c_master_write(&i2c_msg, retry)) {
		LOG_ERR("Unlock register fail");
		return false;
	}
	k_usleep(300);
	return true;
}

uint8_t xdpe15284_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	cfg->read = xdpe15284_read;
	return SENSOR_INIT_SUCCESS;
}
