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
#include "sensor.h"
#include "hal_i2c.h"
#include "pmbus.h"

static bool adjust_of_twos_complement(uint8_t offset, int *val)
{
	if (val == NULL) {
		printf("[%s] input value is NULL\n", __func__);
		return false;
	}

	if ((offset == PMBUS_READ_IOUT) || (offset == PMBUS_READ_POUT)) {
		bool is_negative_val = ((*val & BIT(15)) == 0 ? false : true);
		if (is_negative_val) {
			*val = 0;
		}
		return true;
	}
	return false;
}

uint8_t raa229621_read(uint8_t sensor_num, int *reading)
{
	if (reading == NULL || (sensor_num > SENSOR_NUM_MAX)) {
		return SENSOR_UNSPECIFIED_ERROR;
	}

	sensor_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];
	uint8_t retry = 5;
	uint8_t offset = cfg->offset;

	I2C_MSG msg;

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = offset;

	if (i2c_master_read(&msg, retry)) {
		return SENSOR_FAIL_TO_ACCESS;
	}

	sensor_val *sval = (sensor_val *)reading;
	memset(sval, 0, sizeof(sensor_val));
	bool ret = false;
	int val = 0;
	val = (msg.data[1] << 8) | msg.data[0];

	switch (offset) {
	case PMBUS_READ_VOUT:
		/* 1 mV/LSB, unsigned integer */
		sval->integer = val / 1000;
		sval->fraction = val % 1000;
		break;
	case PMBUS_READ_IOUT:
		/* 0.1 A/LSB, 2's complement */
		ret = adjust_of_twos_complement(offset, &val);
		if (ret == false) {
			printf("[%s] adjust reading IOUT value failed - sensor number: 0x%x\n",
			       __func__, sensor_num);
			return SENSOR_UNSPECIFIED_ERROR;
		}
		sval->integer = (int16_t)val / 10;
		sval->fraction = ((int16_t)val - (sval->integer * 10)) * 100;
		break;
	case PMBUS_READ_TEMPERATURE_1:
		/* 1 Degree C/LSB, 2's complement */
		sval->integer = val;
		sval->fraction = 0;
		break;
	case PMBUS_READ_POUT:
		/* 1 Watt/LSB, 2's complement */
		ret = adjust_of_twos_complement(offset, &val);
		if (ret == false) {
			printf("[%s] adjust reading POUT value failed - sensor number: 0x%x\n",
			       __func__, sensor_num);
			return SENSOR_UNSPECIFIED_ERROR;
		}
		sval->integer = val;
		sval->fraction = 0;
		break;
	default:
		printf("[%s] not support offset: 0x%x\n", __func__, offset);
		return SENSOR_FAIL_TO_ACCESS;
		break;
	}

	return SENSOR_READ_SUCCESS;
}

uint8_t raa229621_init(uint8_t sensor_num)
{
	if (sensor_num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	sensor_config[sensor_config_index_map[sensor_num]].read = raa229621_read;
	return SENSOR_INIT_SUCCESS;
}
