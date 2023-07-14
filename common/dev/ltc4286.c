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
#include <stdlib.h>
#include "sensor.h"
#include "hal_i2c.h"
#include "libutil.h"
#include "pmbus.h"

#include <logging/log.h>

LOG_MODULE_REGISTER(ltc4286);

#define LTC4286_MFR_CONFIG1 0xF2
#define I2C_DATA_SIZE 5

enum VRANGE_SELECT {
	VRANGE_SELECT_25_6_V = 0x0,
	VRANGE_SELECT_102_4_V = 0x1,
	BOTH = 0x2,
};

enum SENSOR_TYPE {
	VOLTAGE = 0x0,
	CURRENT = 0x1,
	POWER = 0x2,
	TEMPERATURE = 0x3,
};

struct LTC4286_MBR_TABLE {
	uint8_t type;
	uint8_t voltage_range;
	float M;
	bool is_multiplied_by_rsense;
	float R;
	float B;
};

/* Following the datasheet (table 3 in page 16),
   the real sensor value is calculated as
   (1 / M) * raw value * (10^(-R)) - B
   And the M, B and R vaues are depended on different sensor type and voltage range.
   The table is defined in ltc4286_mbr_table.
    (1) type - sensor type(voltage, current and etc.)
    (2) voltage_range - voltage range for VIN and VOUT measurements
    (3) M - PMBus M value
    (4) is_multiplied_by_rsense - if M value should be multiplied by r_sense or not.
    (5) R - PMBus R value
    (6) B - PMBus B value
 */
struct LTC4286_MBR_TABLE ltc4286_mbr_table[] = {
	{ VOLTAGE, VRANGE_SELECT_102_4_V, 32, false, 0.1, 0 },
	{ VOLTAGE, VRANGE_SELECT_25_6_V, 128, false, 0.1, 0 },
	{ CURRENT, BOTH, 1024, true, 0.001, 0 },
	{ POWER, VRANGE_SELECT_102_4_V, 1, true, 0.0001, 0 },
	{ POWER, VRANGE_SELECT_25_6_V, 4, true, 0.0001, 0 },
	{ TEMPERATURE, BOTH, 1, false, 1, 273.15 },
};

int ltc4286_convert_real_value(uint8_t type, uint8_t vrange_select, float rsense, uint16_t tmp_val,
			       double *val)
{
	CHECK_NULL_ARG_WITH_RETURN(val, -1);

	uint8_t index = 0;
	float M = 0;

	for (index = 0; index < ARRAY_SIZE(ltc4286_mbr_table); index++) {
		if (type == ltc4286_mbr_table[index].type) {
			if ((ltc4286_mbr_table[index].voltage_range == BOTH) ||
			    (ltc4286_mbr_table[index].voltage_range == vrange_select)) {
				// Multiplied by r_sense value if it needs
				M = ltc4286_mbr_table[index].M;
				if (ltc4286_mbr_table[index].is_multiplied_by_rsense) {
					M *= rsense;
				}

				// real value = (1 / M) * raw_value * R - B;
				*val = (1 / M) * tmp_val * ltc4286_mbr_table[index].R -
				       ltc4286_mbr_table[index].B;
				return 0;
			}
		}
	}

	return -1;
}

int ltc4286_read_iin(sensor_cfg *cfg, double *val)
{
	CHECK_NULL_ARG_WITH_RETURN(val, -1);
	CHECK_NULL_ARG_WITH_RETURN(cfg, -1);
	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, -1);

	ltc4286_init_arg *init_arg = (ltc4286_init_arg *)cfg->init_args;

	int ret = 0;
	uint8_t retry = 5;
	double vin = 0;
	double pin = 0;
	float rsense = init_arg->r_sense_mohm / 1000;
	I2C_MSG msg = { 0 };

	// Read voltage in
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = PMBUS_READ_VIN;

	if (i2c_master_read(&msg, retry)) {
		LOG_ERR("Read vin fail, ret: %d", ret);
		return -1;
	}

	ret = ltc4286_convert_real_value(VOLTAGE, init_arg->mfr_config_1.fields.vrange_select,
					 rsense, ((msg.data[1] << 8) | msg.data[0]), &vin);
	if (ret != 0) {
		LOG_ERR("Convert vin value fail");
		return ret;
	}

	if (vin == 0) {
		LOG_ERR("Vin is zero");
		return -1;
	}

	// Read power in
	memset(&msg, 0, sizeof(I2C_MSG));
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = PMBUS_READ_PIN;

	if (i2c_master_read(&msg, retry)) {
		LOG_ERR("Read pin fail, ret: %d", ret);
		return -1;
	}

	ret = ltc4286_convert_real_value(POWER, init_arg->mfr_config_1.fields.vrange_select, rsense,
					 ((msg.data[1] << 8) | msg.data[0]), &pin);
	if (ret != 0) {
		LOG_ERR("Convert pin value fail");
		return ret;
	}

	*val = pin / vin;
	return 0;
}

int ltc4286_read_pout(sensor_cfg *cfg, double *val)
{
	CHECK_NULL_ARG_WITH_RETURN(val, -1);
	CHECK_NULL_ARG_WITH_RETURN(cfg, -1);
	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, -1);

	ltc4286_init_arg *init_arg = (ltc4286_init_arg *)cfg->init_args;

	int ret = 0;
	uint8_t retry = 5;
	double vout = 0;
	double iout = 0;
	float rsense = init_arg->r_sense_mohm / 1000;
	I2C_MSG msg = { 0 };

	// Read voltage out
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = PMBUS_READ_VOUT;

	if (i2c_master_read(&msg, retry)) {
		LOG_ERR("Read vout fail, ret: %d", ret);
		return -1;
	}

	ret = ltc4286_convert_real_value(VOLTAGE, init_arg->mfr_config_1.fields.vrange_select,
					 rsense, ((msg.data[1] << 8) | msg.data[0]), &vout);
	if (ret != 0) {
		LOG_ERR("Convert vout value fail");
		return ret;
	}

	// Read current out
	memset(&msg, 0, sizeof(I2C_MSG));
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = PMBUS_READ_IOUT;

	if (i2c_master_read(&msg, retry)) {
		LOG_ERR("Read iout fail, ret: %d", ret);
		return -1;
	}

	ret = ltc4286_convert_real_value(CURRENT, init_arg->mfr_config_1.fields.vrange_select,
					 rsense, ((msg.data[1] << 8) | msg.data[0]), &iout);
	if (ret != 0) {
		LOG_ERR("Convert iout value fail");
		return ret;
	}

	*val = vout * iout;
	return 0;
}

uint8_t ltc4286_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	ltc4286_init_arg *init_arg = (ltc4286_init_arg *)cfg->init_args;
	if (init_arg->is_init == false) {
		LOG_ERR("Device isn't initialized");
		return SENSOR_UNSPECIFIED_ERROR;
	}

	float rsense = init_arg->r_sense_mohm / 1000;

	int ret = 0;
	uint8_t retry = 5;
	double val;
	I2C_MSG msg;

	if ((cfg->offset != PMBUS_READ_IIN) && (cfg->offset != PMBUS_READ_POUT)) {
		msg.bus = cfg->port;
		msg.target_addr = cfg->target_addr;
		msg.tx_len = 1;
		msg.rx_len = 2;
		msg.data[0] = cfg->offset;
		if (i2c_master_read(&msg, retry)) {
			return SENSOR_FAIL_TO_ACCESS;
		}
	}

	sensor_val *sval = (sensor_val *)reading;
	memset(sval, 0, sizeof(*sval));

	// Choose the corresponding sensor type
	uint8_t type = 0xFF;
	switch (cfg->offset) {
	case PMBUS_READ_VIN:
	case PMBUS_READ_VOUT:
		type = VOLTAGE;
		break;
	case PMBUS_READ_IIN:
		ret = ltc4286_read_iin(cfg, &val);
		if (ret != 0) {
			return SENSOR_UNSPECIFIED_ERROR;
		}
		goto return_val;
	case PMBUS_READ_IOUT:
		type = CURRENT;
		break;
	case PMBUS_READ_TEMPERATURE_1:
		type = TEMPERATURE;
		break;
	case PMBUS_READ_PIN:
		type = POWER;
		break;
	case PMBUS_READ_POUT:
		ret = ltc4286_read_pout(cfg, &val);
		if (ret != 0) {
			return SENSOR_UNSPECIFIED_ERROR;
		}
		goto return_val;
	default:
		return SENSOR_NOT_FOUND;
	}

	/* Iterate the LTC4286 MBR table to
	 * find the corresponding MBR values by the sensor type and voltage range selection
	 */
	uint8_t index = 0;
	for (int cnt = 0; cnt < ARRAY_SIZE(ltc4286_mbr_table); cnt++) {
		if (type == ltc4286_mbr_table[cnt].type) {
			if ((ltc4286_mbr_table[cnt].voltage_range == BOTH) ||
			    (ltc4286_mbr_table[cnt].voltage_range ==
			     init_arg->mfr_config_1.fields.vrange_select)) {
				index = cnt;
			} else {
				continue;
			}
		}
		if (cnt == ARRAY_SIZE(ltc4286_mbr_table)) {
			LOG_ERR("Undifined the type(0x%x) in LTC4286 MBR table", type);
			return SENSOR_UNSPECIFIED_ERROR;
		}
	}

	if (index == 0xff) {
		LOG_ERR("Index in ltc4286_mbr_table is %hhu, out of range", index);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	// Multiplied by r_sense value if it needs
	float M = ltc4286_mbr_table[index].M;
	if (ltc4286_mbr_table[index].is_multiplied_by_rsense) {
		M *= rsense;
	}

	// real value = (1 / M) * raw_value * R - B;
	val = (1 / M) * ((msg.data[1] << 8) | msg.data[0]) * ltc4286_mbr_table[index].R -
	      ltc4286_mbr_table[index].B;

return_val:
	sval->integer = (int32_t)val;
	sval->fraction = (int32_t)(val * 1000) % 1000;

	return SENSOR_READ_SUCCESS;
}

uint8_t ltc4286_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	I2C_MSG msg;
	memset(&msg, 0, sizeof(msg));
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	uint8_t retry = 5;

	ltc4286_init_arg *init_args = (ltc4286_init_arg *)cfg->init_args;
	if (init_args->is_init) {
		goto init_param;
	}

	/* if the value is equal to 0xFFFF, use the default value on the chip */
	if (init_args->mfr_config_1.value != 0xFFFF) {
		// Set MFR_CONFIG_1 register value
		msg.tx_len = 3;
		msg.rx_len = 0;
		msg.data[0] = LTC4286_MFR_CONFIG1;
		msg.data[1] = init_args->mfr_config_1.value & 0xFF;
		msg.data[2] = (init_args->mfr_config_1.value >> 8) & 0xFF;
		if (i2c_master_write(&msg, retry) != 0) {
			LOG_ERR("Failed to set LTC4286 register(0x%x)", msg.data[0]);
			goto cleanup;
		}
	}

init_param:
	// Read MFR_CONFIG_1 register value
	msg.tx_len = 1;
	msg.rx_len = 3;
	msg.data[0] = LTC4286_MFR_CONFIG1;
	if (i2c_master_read(&msg, retry) != 0) {
		LOG_ERR("Failed to read LTC4286 register(0x%x)", msg.data[0]);
		init_args->is_init = false;
		goto cleanup;
	}
	if (msg.data[0] == LTC4286_MFR_CONFIG1) {
		init_args->mfr_config_1.value = (msg.data[1] | (msg.data[2] << 8));
	}

	init_args->is_init = true;

cleanup:
	cfg->read = ltc4286_read;
	return SENSOR_INIT_SUCCESS;
}
