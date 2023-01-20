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

#include <zephyr.h>
#include <stdlib.h>
#include <stdio.h>
#include "sensor.h"
#include "hal_i2c.h"
#include "mctp.h"
#include <logging/log.h>
#include "plat_def.h"
#include "libutil.h"

#ifdef ENABLE_PM8702
#include "pm8702.h"
#include "cci.h"

LOG_MODULE_REGISTER(pm8702);

bool pm8702_get_dimm_temp(void *mctp_p, mctp_ext_params ext_params, uint16_t address,
			  int16_t *interger, int16_t *fraction)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_p, false);
	CHECK_NULL_ARG_WITH_RETURN(interger, false);
	CHECK_NULL_ARG_WITH_RETURN(fraction, false);

	i2c_offset_read_req req = { 0 };
	mctp_cci_msg msg = { 0 };
	memcpy(&msg.ext_params, &ext_params, sizeof(mctp_ext_params));

	msg.hdr.op = pm8702_I2C_OFFSET_READ;
	msg.hdr.pl_len = sizeof(req);

	msg.pl_data = (uint8_t *)malloc(sizeof(req));
	if (msg.pl_data == NULL) {
		LOG_ERR("Failed to allocate payload data.");
		return false;
	}

	req.addr_size = ADDR_SIZE_7_BIT;
	req.address = address; //dimm temp register address  /*Refer to JEDEC SPD*/
	req.offset_size = OFFSET_SIZE_8_BIT;
	req.offset = DIMM_TEMP_REG_OFFSET; // temperature register offset of the DIMM
	req.timeout_offset = 1;
	req.read_bytes = DIMM_TEMP_READ_RESP_PL_LEN;
	req.timeout_ms = I2C_READ_TIMEOUT_MS;

	memcpy(msg.pl_data, &req, sizeof(req));

	uint8_t rbuf[DIMM_TEMP_READ_RESP_PL_LEN] = { 0 };
	if (mctp_cci_read(mctp_p, &msg, rbuf, DIMM_TEMP_READ_RESP_PL_LEN) !=
	    DIMM_TEMP_READ_RESP_PL_LEN) {
		LOG_ERR("[%s] mctp_cci_read fail", __func__);
		SAFE_FREE(msg.pl_data);
		return false;
	}

	int8_t dimm_int = (rbuf[0] << 4) | (rbuf[1] >> 4);
	int8_t dimm_frac = 0;
	if (rbuf[0] & BIT(4)) { // negative  /*Refer to JEDEC SPD*/
		dimm_int = -(~dimm_int);
		dimm_frac = -((((~rbuf[1]) & 0x0F) >> 2) + 1) * 25;
	} else {
		dimm_frac = ((rbuf[1] & 0x0F) >> 2) * 25;
	}
	*interger = dimm_int;
	*fraction = dimm_frac * 10;

	SAFE_FREE(msg.pl_data);
	return true;
}

uint8_t pm8702_read(uint8_t sensor_num, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);
	if (sensor_num > SENSOR_NUM_MAX) {
		return SENSOR_UNSPECIFIED_ERROR;
	}
	uint8_t port = sensor_config[sensor_config_index_map[sensor_num]].port;
	uint8_t address = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
	uint8_t pm8702_access = sensor_config[sensor_config_index_map[sensor_num]].offset;

	mctp *mctp_inst = NULL;
	mctp_ext_params ext_params = { 0 };
	sensor_val *sval = (sensor_val *)reading;

	if (get_mctp_info_by_eid(port, &mctp_inst, &ext_params) == false) {
		return SENSOR_UNSPECIFIED_ERROR;
	}
	if (!mctp_inst) {
		return SENSOR_UNSPECIFIED_ERROR;
	}
	switch (pm8702_access) {
	case chip_temp:
		if (cci_get_chip_temp(mctp_inst, ext_params, &sval->integer) == false) {
			return SENSOR_NOT_ACCESSIBLE;
		}
		sval->fraction = 0;
		break;
	case dimm_temp:
		if (pm8702_get_dimm_temp(mctp_inst, ext_params, address, &sval->integer,
					 &sval->fraction) == false) {
			return SENSOR_NOT_ACCESSIBLE;
		}
		break;
	default:
		LOG_ERR("Invalid access offset %d", pm8702_access);
		break;
	}

	return SENSOR_READ_SUCCESS;
}

uint8_t pm8702_init(uint8_t sensor_num)
{
	if (sensor_num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}
	sensor_config[sensor_config_index_map[sensor_num]].read = pm8702_read;
	return SENSOR_INIT_SUCCESS;
}

#endif
