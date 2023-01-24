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
#include <logging/log.h>
#include "sensor.h"
#include "intel_dimm.h"
#include "pmic.h"
#include "hal_i3c.h"

LOG_MODULE_REGISTER(dev_dimm);

__weak int pal_get_pmic_pwr(uint8_t sensor_num, uint8_t *data)
{
	return -1;
}

__weak int pal_get_spd_temp(uint8_t sensor_num, uint8_t *data)
{
	return -1;
}

uint8_t i3c_dimm_read(uint8_t sensor_num, int *reading)
{
	if (reading == NULL || (sensor_num > SENSOR_NUM_MAX)) {
		return SENSOR_UNSPECIFIED_ERROR;
	}

	int ret = 0;
	uint8_t data[2] = { 0 };

	memset(data, 0, sizeof(data));
	memset(reading, 0, sizeof(int));
	sensor_val *sval = (sensor_val *)reading;

	switch (sensor_config[sensor_config_index_map[sensor_num]].offset) {
	case DIMM_PMIC_SWA_PWR:
		ret = pal_get_pmic_pwr(sensor_num, data);
		if (ret != 0) {
			return SENSOR_FAIL_TO_ACCESS;
		}

		sval->integer = (data[0] * PMIC_TOTAL_POWER_MW / 1000) & 0xFFFF;
		sval->fraction = (data[0] * PMIC_TOTAL_POWER_MW % 1000) & 0xFFFF;
		break;
	case DIMM_SPD_TEMP:
		ret = pal_get_spd_temp(sensor_num, data);
		if (ret != 0) {
			return SENSOR_FAIL_TO_ACCESS;
		}

		/* According to SPEC 3.2.2 Thermal Sensor Registers Read Out Mechanism (p.105)
		Thermal Register - Low Byte & High Byte
		B |7   |6   |5   |4   |3   |2   |1   |0
		LB|8   |4   |2   |1   |0.5 |0.25|RSVD|RSVD
		HB|RSVD|RSVD|RSVD|Sign|128 |64  |32  |16
		*/
		sval->integer = ((data[1] & 0xF) << 4) | (data[0] & 0xF0) >> 4;
		sval->fraction = (GETBIT(data[0], 3) * 0.5 + GETBIT(data[0], 2) * 0.25) * 1000;

		if (GETBIT(data[1], 4)) {
			sval->integer = -sval->integer;
			sval->fraction = -sval->fraction;
		}
		break;
	default:
		return SENSOR_FAIL_TO_ACCESS;
	}

	return SENSOR_READ_SUCCESS;
}

uint8_t i3c_dimm_init(uint8_t sensor_num)
{
	if (sensor_num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	sensor_config[sensor_config_index_map[sensor_num]].read = i3c_dimm_read;
	return SENSOR_INIT_SUCCESS;
}
