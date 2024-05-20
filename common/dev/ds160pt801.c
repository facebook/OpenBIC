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
#include <stdlib.h>
#include <logging/log.h>
#include "ds160pt801.h"
#include "hal_i2c.h"
#include "libutil.h"
#include "sensor.h"

LOG_MODULE_REGISTER(dev_ds160pt801);

K_MUTEX_DEFINE(ds106pt801_mutex);

/* ADDR MODE SWITCH */
#define ADDR_MODE_16_TO_8 0x0168
#define ADDR_MODE_8_TO_16 0xF9

enum {
	FROM_16_TO_8 = 0, //default
	FROM_8_TO_16 = 1,
};

/* --------- GLOBAL ---------- */
#define GLOBAL_REG_DEV_REV 0xF0
#define GLOBAL_REG_BLOCK_SEL 0xFF

/* --------- SHARE ---------- */
#define SHARE_REG_TEMP_CTL 0x78
#define SHARE_REG_TEMP 0x79
#define SHARE_REG_TEMP_NOM 0x7E

static bool is_update_ongoing = false;

static bool ds160pt801_switch_addr_mode(I2C_MSG *i2c_msg, uint8_t mode)
{
	CHECK_NULL_ARG_WITH_RETURN(i2c_msg, false);

	switch (mode) {
	case FROM_16_TO_8:
		i2c_msg->data[0] = ADDR_MODE_16_TO_8 & 0xFF;
		i2c_msg->data[1] = (ADDR_MODE_16_TO_8 >> 8) & 0xFF;
		i2c_msg->tx_len = 3;
		break;

	case FROM_8_TO_16:
		i2c_msg->data[0] = ADDR_MODE_8_TO_16;
		i2c_msg->data[1] = 0x01;
		i2c_msg->tx_len = 2;
		break;

	default:
		break;
	}

	if (i2c_master_write(i2c_msg, 3)) {
		LOG_ERR("Failed to switch ADDR mode %d", mode);
		return false;
	}

	return true;
}

bool ds160pt801_get_fw_version(I2C_MSG *i2c_msg, uint8_t *version)
{
	CHECK_NULL_ARG_WITH_RETURN(i2c_msg, false);
	CHECK_NULL_ARG_WITH_RETURN(version, false);

	bool ret = false;

	if (is_update_ongoing) {
		return false;
	}

	if (k_mutex_lock(&ds106pt801_mutex, K_MSEC(1000))) {
		LOG_ERR("Mutex lock failed");
		return false;
	}

	i2c_msg->data[0] = GLOBAL_REG_DEV_REV;
	i2c_msg->tx_len = 1;
	i2c_msg->rx_len = 1;

	if (i2c_master_read(i2c_msg, 3)) {
		LOG_ERR("Failed to read REVISION register");
		goto unlock_exit;
	}

	*version = i2c_msg->data[0];

	ret = true;
unlock_exit:
	if (k_mutex_unlock(&ds106pt801_mutex)) {
		LOG_ERR("Mutex unlock failed");
	}

	return ret;
}

bool ds160pt801_read_junction_temp(I2C_MSG *i2c_msg, double *avg_temperature)
{
	CHECK_NULL_ARG_WITH_RETURN(i2c_msg, SENSOR_UNSPECIFIED_ERROR);
	uint8_t tmp_byte = 0;
	bool ret = false;

	if (k_mutex_lock(&ds106pt801_mutex, K_MSEC(1000))) {
		LOG_ERR("Mutex lock failed");
		return false;
	}

	/* Step1. Enable DIE0 shared register set */
	i2c_msg->data[0] = GLOBAL_REG_BLOCK_SEL;
	i2c_msg->tx_len = 1;
	i2c_msg->rx_len = 1;

	if (i2c_master_read(i2c_msg, 3)) {
		LOG_ERR("Failed to read BLOCK SEL register");
		goto unlock_exit;
	}
	tmp_byte = i2c_msg->data[0];

	tmp_byte |= BIT(6);
	i2c_msg->data[0] = GLOBAL_REG_BLOCK_SEL;
	i2c_msg->data[1] = tmp_byte;
	i2c_msg->tx_len = 2;
	i2c_msg->rx_len = 0;

	if (i2c_master_write(i2c_msg, 3)) {
		LOG_ERR("Failed to enable die0");
		goto unlock_exit;
	}

	/* Step2. Freeze capture */
	i2c_msg->data[0] = SHARE_REG_TEMP_CTL;
	i2c_msg->tx_len = 1;
	i2c_msg->rx_len = 1;

	if (i2c_master_read(i2c_msg, 3)) {
		LOG_ERR("Failed to read TEMP CTL register");
		goto unlock_exit;
	}
	tmp_byte = i2c_msg->data[0];

	tmp_byte &= ~BIT(1);
	i2c_msg->data[0] = SHARE_REG_TEMP_CTL;
	i2c_msg->data[1] = tmp_byte;
	i2c_msg->tx_len = 2;
	i2c_msg->rx_len = 0;

	if (i2c_master_write(i2c_msg, 3)) {
		LOG_ERR("Failed to freeze capture");
		goto unlock_exit;
	}

	/* Step3. Disable temp sensor */
	tmp_byte &= ~BIT(2);
	i2c_msg->data[0] = SHARE_REG_TEMP_CTL;
	i2c_msg->data[1] = tmp_byte;
	i2c_msg->tx_len = 2;
	i2c_msg->rx_len = 0;

	if (i2c_master_write(i2c_msg, 3)) {
		LOG_ERR("Failed to disable temp sensor");
		goto unlock_exit;
	}

	/* Step4. Enable temp sensor */
	tmp_byte |= BIT(2);
	i2c_msg->data[0] = SHARE_REG_TEMP_CTL;
	i2c_msg->data[1] = tmp_byte;
	i2c_msg->tx_len = 2;
	i2c_msg->rx_len = 0;

	if (i2c_master_write(i2c_msg, 3)) {
		LOG_ERR("Failed to enable temp sensor");
		goto unlock_exit;
	}

	/* Step5. Reset temp sensor */
	tmp_byte &= ~BIT(0);
	i2c_msg->data[0] = SHARE_REG_TEMP_CTL;
	i2c_msg->data[1] = tmp_byte;
	i2c_msg->tx_len = 2;
	i2c_msg->rx_len = 0;

	if (i2c_master_write(i2c_msg, 3)) {
		LOG_ERR("Failed to reset temp sensor");
		goto unlock_exit;
	}

	/* Step6. Unfreeze capture */
	tmp_byte |= BIT(1);
	i2c_msg->data[0] = SHARE_REG_TEMP_CTL;
	i2c_msg->data[1] = tmp_byte;
	i2c_msg->tx_len = 2;
	i2c_msg->rx_len = 0;

	if (i2c_master_write(i2c_msg, 3)) {
		LOG_ERR("Failed to unfreeze capture");
		goto unlock_exit;
	}

	/* Step7. Read TEMP_NOM */
	i2c_msg->data[0] = SHARE_REG_TEMP_NOM;
	i2c_msg->tx_len = 1;
	i2c_msg->rx_len = 1;

	if (i2c_master_read(i2c_msg, 3)) {
		LOG_ERR("Failed to unfreeze capture");
		goto unlock_exit;
	}

	*avg_temperature = i2c_msg->data[0];

	/* Step8. Read TEMP */
	i2c_msg->data[0] = SHARE_REG_TEMP;
	i2c_msg->tx_len = 1;
	i2c_msg->rx_len = 1;

	if (i2c_master_read(i2c_msg, 3)) {
		LOG_ERR("Failed to unfreeze capture");
		goto unlock_exit;
	}

	*avg_temperature = 1.56 * (i2c_msg->data[0] - *avg_temperature) + 28.5;

	ret = true;
unlock_exit:
	if (k_mutex_unlock(&ds106pt801_mutex)) {
		LOG_ERR("Mutex unlock failed");
	}

	return ret;
}

uint8_t ds160pt801_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	if (is_update_ongoing) {
		return SENSOR_NOT_ACCESSIBLE;
	}

	I2C_MSG msg = { 0 };
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;

	if (ds160pt801_switch_addr_mode(&msg, FROM_16_TO_8) == false) {
		LOG_ERR("Failed to switch ADDR mode from 16bit to 8bit");
		return SENSOR_FAIL_TO_ACCESS;
	}

	double val = 0;

	switch (cfg->offset) {
	case DS160PT801_READ_TEMP:
		if (ds160pt801_read_junction_temp(&msg, &val) == false) {
			LOG_ERR("Failed to read junction temperature");
			return SENSOR_FAIL_TO_ACCESS;
		}

		break;
	default:
		LOG_ERR("Invalid sensor 0x%x offset 0x%x", cfg->num, cfg->offset);
		return SENSOR_NOT_FOUND;
	}

	sensor_val *sval = (sensor_val *)reading;
	memset(sval, 0, sizeof(*sval));

	sval->integer = (int)val & 0xFFFF;
	sval->fraction = (val - sval->integer) * 1000;

	return SENSOR_READ_SUCCESS;
}

uint8_t ds160pt801_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	cfg->read = ds160pt801_read;
	return SENSOR_INIT_SUCCESS;
}
