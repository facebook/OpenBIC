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
#include "sensor.h"
#include "hal_i2c.h"
#include "libutil.h"
#include <logging/log.h>
#include "iris_smbus.h"

LOG_MODULE_REGISTER(iris_smbus);

bool iris_smbus_i2c_read(uint8_t bus, uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = len;
	i2c_msg.data[0] = reg;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Failed to read iris_smbus, bus: %d, addr: 0x%x, reg: 0x%x", bus, addr,
			reg);
		return false;
	}

	memcpy(data, i2c_msg.data, len);
	return true;
}

uint8_t iris_smbus_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	iris_priv_data_t *priv = (iris_priv_data_t *)cfg->priv_data;
	if (!priv) {
		return SENSOR_UNSPECIFIED_ERROR;
	}

	if (cfg->offset == ASIC_MONITOR_HBM_TEMP_REG) {
		uint8_t hbm_temp_data[ASIC_MONITOR_HBM_TEMP_REG_LEN] = { 0 };
		if (!iris_smbus_i2c_read(cfg->port, cfg->target_addr, cfg->offset,
					 (uint8_t *)hbm_temp_data, ASIC_MONITOR_HBM_TEMP_REG_LEN)) {
			LOG_ERR("Can't get max asic temp data from ASIC, reg: 0x%02x", cfg->offset);
			return SENSOR_UNSPECIFIED_ERROR;
		}
		memcpy(priv->hbm_temp, hbm_temp_data, ASIC_MONITOR_HBM_TEMP_REG_LEN);
		*reading = priv->hbm_temp[0];

		return SENSOR_READ_SUCCESS;
	}
	if (cfg->offset == ASIC_MONITOR_TEMP_REG) {
		uint8_t temp_data[ASIC_MONITOR_TEMP_REG_LEN] = { 0 };
		if (!iris_smbus_i2c_read(cfg->port, cfg->target_addr, cfg->offset,
					 (uint8_t *)temp_data, ASIC_MONITOR_TEMP_REG_LEN)) {
			LOG_ERR("Can't get max asic temp data from ASIC, reg: 0x%02x", cfg->offset);
			return SENSOR_UNSPECIFIED_ERROR;
		}

		memcpy(priv->temp, temp_data, ASIC_MONITOR_TEMP_REG_LEN);
		*reading = priv->temp[0];

		return SENSOR_READ_SUCCESS;
	}

	return SENSOR_READ_SUCCESS;
}

uint8_t iris_smbus_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	iris_priv_data_t *priv = k_malloc(sizeof(iris_priv_data_t));
	if (!priv) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	memset(priv, 0, sizeof(*priv));

	cfg->priv_data = priv;
	cfg->read = iris_smbus_read;
	return SENSOR_INIT_SUCCESS;
}
