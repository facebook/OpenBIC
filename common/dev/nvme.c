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
#include "libutil.h"
#include "sensor.h"
#include "hal_i2c.h"

#include <logging/log.h>
#include <sys/crc.h>

#define NVMe_NOT_AVAILABLE 0x80
#define NVMe_TMP_SENSOR_FAILURE 0x81

#define NVMe_READ_LEN 8

LOG_MODULE_REGISTER(nvme);

static uint8_t nvme_cal_pec(uint8_t addr, uint8_t oft, uint8_t *buf, uint32_t len, uint8_t *pec)
{
	CHECK_NULL_ARG_WITH_RETURN(buf, 1);
	CHECK_NULL_ARG_WITH_RETURN(pec, 1);

	uint8_t pec_buf[len + 3];
	pec_buf[0] = addr << 1;
	pec_buf[1] = oft;
	pec_buf[2] = (addr << 1) + 1;

	memcpy(pec_buf + 3, buf, len);

	*pec = crc8(pec_buf, sizeof(pec_buf), 0x07, 0x00, false);

	return 0;
}

uint8_t nvme_read(uint8_t sensor_num, int *reading)
{
	if (!reading || (sensor_num > SENSOR_NUM_MAX)) {
		return SENSOR_UNSPECIFIED_ERROR;
	}

	uint8_t retry = 5;
	bool is_drive_ready;
	I2C_MSG msg;

	msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
	msg.data[0] = sensor_config[sensor_config_index_map[sensor_num]].offset;
	msg.tx_len = 1;
	msg.rx_len = NVMe_READ_LEN;

	if (!i2c_master_read(&msg, retry)) {
		/* Check PEC */
		uint8_t pec = 0;
		if (nvme_cal_pec(sensor_config[sensor_config_index_map[sensor_num]].target_addr,
				 sensor_config[sensor_config_index_map[sensor_num]].offset,
				 msg.data, NVMe_READ_LEN - 1, &pec)) {
			LOG_ERR("sensor_num 0x%02x cal nvme pec fail!\n", sensor_num);
			return SENSOR_UNSPECIFIED_ERROR;
		}
		if (msg.data[7] != pec) {
			LOG_ERR("sensor_num 0x%02x check nvme pec error! (%02x/%02x)\n", sensor_num,
				msg.data[7], pec);
			return SENSOR_PEC_ERROR;
		}

		/* Check SSD drive ready */
		is_drive_ready = ((msg.data[1] & 0x40) == 0 ? true : false);
		if (!is_drive_ready)
			return SENSOR_NOT_ACCESSIBLE;

		/* Check reading value */
		if (msg.data[3] == NVMe_NOT_AVAILABLE) {
			return SENSOR_FAIL_TO_ACCESS;
		}
		if (msg.data[3] == NVMe_TMP_SENSOR_FAILURE) {
			return SENSOR_UNSPECIFIED_ERROR;
		}

	} else {
		return SENSOR_FAIL_TO_ACCESS;
	}

	sensor_val *sval = (sensor_val *)reading;
	sval->integer = (int8_t)msg.data[3];
	sval->fraction = 0;

	return SENSOR_READ_SUCCESS;
}

uint8_t nvme_init(uint8_t sensor_num)
{
	if (sensor_num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	sensor_config[sensor_config_index_map[sensor_num]].read = nvme_read;
	return SENSOR_INIT_SUCCESS;
}
