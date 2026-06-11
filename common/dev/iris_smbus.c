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

// bool mp29816a_i2c_write(uint8_t bus, uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len)
// {
// 	CHECK_NULL_ARG_WITH_RETURN(data, false);

// 	I2C_MSG i2c_msg = { 0 };
// 	uint8_t retry = 5;
// 	i2c_msg.bus = bus;
// 	i2c_msg.target_addr = addr;
// 	i2c_msg.tx_len = len + 1;

// 	i2c_msg.data[0] = reg;

// 	if (len > 0)
// 		memcpy(&i2c_msg.data[1], data, len);

// 	if (i2c_master_write(&i2c_msg, retry)) {
// 		LOG_ERR("Failed to write mp29816a, bus: %d, addr: 0x%x, reg: 0x%x", bus, addr, reg);
// 		return false;
// 	}

// 	return true;
// }

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

// /*
//  * Copyright (c) Meta Platforms, Inc. and affiliates.
//  *
//  * Licensed under the Apache License, Version 2.0 (the "License");
//  * you may not use this file except in compliance with the License.
//  * You may obtain a copy of the License at
//  *
//  *     http://www.apache.org/licenses/LICENSE-2.0
//  *
//  * Unless required by applicable law or agreed to in writing, software
//  * distributed under the License is distributed on an "AS IS" BASIS,
//  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  * See the License for the specific language governing permissions and
//  * limitations under the License.
//  */

// #include <stdio.h>
// #include <stdint.h>
// #include <string.h>
// #include <logging/log.h>
// #include "libutil.h"
// #include "sensor.h"
// #include "pldm.h"
// #include "hal_i2c.h"
// #include "athena.h"

// LOG_MODULE_REGISTER(athena);

// static bool athena_set_sideband(uint8_t bus, uint8_t addr, uint32_t region)
// {
// 	I2C_MSG i2c_msg = { 0 };
// 	uint8_t retry = 3;

// 	i2c_msg.bus = bus;
// 	i2c_msg.target_addr = addr;
// 	i2c_msg.tx_len = 9;
// 	i2c_msg.rx_len = 0;

// 	// // Pack the command and offset address based on your example
// 	// // Example: i2c write I2C_5 6A D3 06 ED 4E 7C D0 FF 10 FF
// 	// i2c_msg.data[0] = 0xD3;
// 	// i2c_msg.data[1] = 0x06;
// 	// i2c_msg.data[2] = (full_offset >> 0) & 0xFF; // ED (LSB)
// 	// i2c_msg.data[3] = (full_offset >> 8) & 0xFF; // 4E
// 	// i2c_msg.data[4] = (full_offset >> 16) & 0xFF; // 7C
// 	// i2c_msg.data[5] = (full_offset >> 24) & 0xFF; // D0 (MSB)
// 	// i2c_msg.data[6] = 0xFF;
// 	// i2c_msg.data[7] = 0x10;
// 	// i2c_msg.data[8] = 0xFF;

// 	uint8_t region_bytes[4] = {
// 		(uint8_t)(region & 0xFF),
// 		(uint8_t)((region >> 8) & 0xFF),
// 		(uint8_t)((region >> 16) & 0xFF),
// 		(uint8_t)((region >> 24) & 0xFF),
// 	};

// 	i2c_msg.data[0] = 0xD3;
// 	i2c_msg.data[1] = 0x06;
// 	i2c_msg.data[2] = 0x00;
// 	i2c_msg.data[3] = region_bytes[0];
// 	i2c_msg.data[4] = region_bytes[1];
// 	i2c_msg.data[5] = region_bytes[2];
// 	i2c_msg.data[6] = region_bytes[3];
// 	i2c_msg.data[7] = 0x10;
// 	i2c_msg.data[8] = 0xFF;

// 	if (i2c_master_write(&i2c_msg, retry)) {
// 		LOG_ERR("Failed to set sideband region 0x%08X", region);
// 		return false;
// 	}

// 	return true;
// }

// bool athena_get_indirect(uint8_t bus, uint8_t addr, uint8_t *data, uint8_t len)
// {
// 	CHECK_NULL_ARG_WITH_RETURN(data, false);

// 	memset(data, 0, len);

// 	I2C_MSG i2c_msg = { 0 };
// 	uint8_t retry = 5;
// 	i2c_msg.bus = bus;
// 	i2c_msg.target_addr = addr;
// 	i2c_msg.tx_len = 2;
// 	i2c_msg.rx_len = len;
// 	i2c_msg.data[0] = 0x00; //offset
// 	i2c_msg.data[0] = len;

// 	// // Setup I2C message for reading data
// 	// // Based on your example: i2c read I2C_5 6C 00 08
// 	// // This means: bus I2C_5, addr 6C, offset 0, read 8 bytes
// 	// i2c_msg.bus = HBM_I2C_BUS;
// 	// i2c_msg.target_addr = HBM_I2C_ADDR_READ;
// 	// i2c_msg.tx_len = 1;
// 	// i2c_msg.rx_len = HBM_CHANNEL_DATA_SIZE; // Read 8 bytes
// 	// i2c_msg.data[0] = 0x00; // Offset 0

// 	if (i2c_master_read(&i2c_msg, retry)) {
// 		LOG_ERR("Failed to get indirect");
// 		return false;
// 	}

// 	memcpy(data, i2c_msg.data, len);
// 	return true;
// }

// uint8_t athena_read(sensor_cfg *cfg, int *reading)
// {
// 	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
// 	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

// 	uint8_t athena_sideband_addr, athena_indirect_addr = 0;

// 	if (cfg->init_args != NULL) {
// 		athena_init_arg *init_arg = (athena_init_arg *)cfg->init_args;
// 		athena_sideband_addr = init_arg->sideband_addr;
// 		athena_indirect_addr = init_arg->indirect_addr;
// 	}

// 	if (cfg->num > SENSOR_NUM_MAX) {
// 		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
// 		return SENSOR_UNSPECIFIED_ERROR;
// 	}

// 	sensor_val *sval = (sensor_val *)reading;
// 	int val = 0;

// 	if (athena_set_sideband(cfg->port, athena_sideband_addr, cfg->offset)) {
// 		LOG_ERR("Failed to set sideband offset 0x%08X", cfg->offset);
// 		return SENSOR_UNSPECIFIED_ERROR;
// 	}

// 	if (athena_get_indirect(cfg->port, athena_indirect_addr(uint8_t *) sval, 2)) {
// 		LOG_ERR("Failed to get indirect");
// 		return SENSOR_UNSPECIFIED_ERROR;
// 	}

// 	sval->integer = (int)val;
// 	sval->fraction = (val - sval->integer) * 1000;

// 	return SENSOR_READ_SUCCESS;
// }

// // uint8_t athena_read(sensor_cfg *cfg, int *reading)
// // {
// // 	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
// // 	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

// // 	if (cfg->num > SENSOR_NUM_MAX) {
// // 		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
// // 		return SENSOR_UNSPECIFIED_ERROR;
// // 	}

// // 	sensor_val *sval = (sensor_val *)reading;
// // 	uint16_t val = 0x0064;

// // 	// I2C_MSG i2c_msg = { 0 };
// // 	// uint8_t retry = 5;
// // 	// sensor_val *sval = (sensor_val *)reading;
// // 	// i2c_msg.bus = cfg->port;
// // 	// i2c_msg.target_addr = cfg->target_addr;
// // 	// i2c_msg.tx_len = 1;
// // 	// i2c_msg.rx_len = 1;
// // 	// i2c_msg.data[0] = cfg->offset;

// // 	// if (i2c_master_read(&i2c_msg, retry)) {
// // 	// 	LOG_WRN("Athena[0x%x] I2C read length failed.", cfg->num);
// // 	// 	return SENSOR_FAIL_TO_ACCESS;
// // 	// }

// // 	// i2c_msg.rx_len = i2c_msg.data[0] + 1;
// // 	// i2c_msg.data[0] = cfg->offset;

// // 	// if (i2c_master_read(&i2c_msg, retry)) {
// // 	// 	LOG_WRN("Athena[0x%x] I2C read failed.", cfg->num);
// // 	// 	return SENSOR_FAIL_TO_ACCESS;
// // 	// }

// // 	// uint16_t val;
// // 	// if (cfg->offset == ASIC_SMBUS_CORE_1_VTMON_VOLT_V_OFFSET ||
// // 	//     cfg->offset == ASIC_SMBUS_CORE_2_VTMON_VOLT_V_OFFSET ||
// // 	//     cfg->offset == ASIC_SMBUS_RAIL_1_VTMON_VOLT_V_OFFSET ||
// // 	//     cfg->offset == ASIC_SMBUS_RAIL_2_VTMON_VOLT_V_OFFSET) {
// // 	// 	val = (i2c_msg.data[1] << 8) | i2c_msg.data[0];
// // 	// } else {
// // 	// 	return SENSOR_FAIL_TO_ACCESS;
// // 	// }

// // 	sval->integer = (int)val;
// // 	sval->fraction = (val - sval->integer) * 1000;

// // 	return SENSOR_READ_SUCCESS;
// // }

// uint8_t athena_init(sensor_cfg *cfg)
// {
// 	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

// 	if (cfg->num > SENSOR_NUM_MAX) {
// 		return SENSOR_INIT_UNSPECIFIED_ERROR;
// 	}

// 	cfg->read = athena_read;
// 	return SENSOR_INIT_SUCCESS;
// }
