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

pm8702_command_info pm8702_cmd_table[] = {
	{ .cmd_opcode = CCI_GET_FW_INFO,
	  .payload_len = GET_FW_INFO_REQ_PL_LEN,
	  .response_len = sizeof(cci_fw_info_resp) },
	{ .cmd_opcode = PM8702_HBO_STATUS,
	  .payload_len = HBO_STATUS_REQ_PL_LEN,
	  .response_len = sizeof(pm8702_hbo_status_resp) },
	{ .cmd_opcode = PM8702_HBO_TRANSFER_FW,
	  .payload_len = HBO_TRANSFER_FW_REQ_PL_LEN,
	  .response_len = TRANSFER_FW_RESP_PL_LEN },
	{ .cmd_opcode = PM8702_HBO_ACTIVATE_FW,
	  .payload_len = HBO_ACTIVATE_FW_REQ_PL_LEN,
	  .response_len = ACTIVATE_FW_RESP_PL_LEN },
};

bool pm8702_cmd_handler(void *mctp_inst, mctp_ext_params ext_params, uint16_t opcode,
			uint8_t *data_buf, int data_len, uint8_t *response, uint8_t *response_len)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, false);
	CHECK_NULL_ARG_WITH_RETURN(response, false);
	CHECK_NULL_ARG_WITH_RETURN(response_len, false);

	if (data_len != 0) {
		CHECK_NULL_ARG_WITH_RETURN(data_buf, false);
	}

	mctp_cci_msg msg = { 0 };
	memcpy(&msg.ext_params, &ext_params, sizeof(mctp_ext_params));

	uint8_t index = 0;

	for (index = 0; index < ARRAY_SIZE(pm8702_cmd_table); ++index) {
		if (opcode == pm8702_cmd_table[index].cmd_opcode) {
			msg.hdr.op = opcode;
			if (data_len > pm8702_cmd_table[index].payload_len) {
				LOG_ERR("Transfer data len: 0x%x is over payload len: 0x%x",
					data_len, msg.hdr.pl_len);
				msg.hdr.pl_len = pm8702_cmd_table[index].payload_len;
			} else {
				msg.hdr.pl_len = data_len;
			}

			int resp_len = pm8702_cmd_table[index].response_len;
			uint8_t resp_buf[resp_len];
			memset(resp_buf, 0, resp_len);

			if (msg.hdr.pl_len != 0) {
				msg.pl_data = (uint8_t *)malloc(sizeof(uint8_t) * msg.hdr.pl_len);
				CHECK_NULL_ARG_WITH_RETURN(msg.pl_data, false);

				memcpy(msg.pl_data, data_buf, sizeof(uint8_t) * msg.hdr.pl_len);
			}

			if (mctp_cci_read(mctp_inst, &msg, resp_buf, resp_len) != resp_len) {
				LOG_ERR("MCTP cci command: 0x%x read fail", opcode);
				SAFE_FREE(msg.pl_data);
				return false;
			}

			memcpy(response, resp_buf, sizeof(uint8_t) * resp_len);
			*response_len = resp_len;
			SAFE_FREE(msg.pl_data);

			return true;
		}
	}

	LOG_ERR("Command opcode: 0x%x is not support", opcode);
	return false;
}

bool pm8702_read_dimm_temp_from_pioneer(void *mctp_p, mctp_ext_params ext_params, int dimm_id,
					int16_t *temp_int, int16_t *temp_dec)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_p, false);
	CHECK_NULL_ARG_WITH_RETURN(temp_int, false);
	CHECK_NULL_ARG_WITH_RETURN(temp_dec, false);

	bool ret = false;

	mctp_cci_msg msg = { 0 };
	memcpy(&msg.ext_params, &ext_params, sizeof(mctp_ext_params));

	msg.hdr.op = PM8702_READ_DIMM_TEMP;
	msg.hdr.pl_len = READ_DIMM_TEMP_REQ_PL_LEN;

	uint8_t rbuf[DIMM_TEMP_RESP_PL_LEN] = { 0 };

	if (!mctp_cci_read(mctp_p, &msg, rbuf, DIMM_TEMP_RESP_PL_LEN)) {
		LOG_ERR("pm8702_get_dimm_temp fail");
		SAFE_FREE(msg.pl_data);
		return ret;
	}

	LOG_HEXDUMP_DBG(rbuf, DIMM_TEMP_RESP_PL_LEN, "pm8702_get_ddr_temp");
	dimm_info_header *dimm_info_header_p = (dimm_info_header *)rbuf;
	int dimm_num = dimm_info_header_p->dimm_num;
	dimm_slot_info *dimm_slot_info_p = (dimm_slot_info *)(rbuf + sizeof(dimm_info_header));

	for (int i = 0; i < dimm_num; i++) {
		dimm_slot_info_p = (dimm_slot_info *)(rbuf + sizeof(dimm_info_header) +
						      (sizeof(dimm_slot_info) * i));
		if (dimm_id == dimm_slot_info_p->dimm_id) {
			LOG_DBG("dimm #%d temp: %d.%d\n", dimm_id, *temp_int, *temp_dec);
			if (dimm_slot_info_p->temp_int == DIMM_ERROR_VALUE) {
				LOG_ERR("Failed to get the dimm data.");
				ret = false;
			} else {
				*temp_int = dimm_slot_info_p->temp_int;
				*temp_dec = dimm_slot_info_p->temp_dec * 10;
				ret = true;
			}
		}
	}
	SAFE_FREE(msg.pl_data);
	return ret;
}

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
		LOG_ERR("mctp_cci_read fail");
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

uint8_t pm8702_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	uint8_t port = cfg->port;
	uint8_t address = cfg->target_addr;
	uint8_t pm8702_access = cfg->offset;
	pm8702_dimm_init_arg *init_arg = (pm8702_dimm_init_arg *)cfg->init_args;

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
			return SENSOR_FAIL_TO_ACCESS;
		}
		sval->fraction = 0;
		break;
	case dimm_temp:
		if (pm8702_get_dimm_temp(mctp_inst, ext_params, address, &sval->integer,
					 &sval->fraction) == false) {
			return SENSOR_FAIL_TO_ACCESS;
		}
		break;
	case dimm_temp_from_pioneer:
		CHECK_NULL_ARG_WITH_RETURN(init_arg, SENSOR_UNSPECIFIED_ERROR);
		if (pm8702_read_dimm_temp_from_pioneer(mctp_inst, ext_params, init_arg->dimm_id,
						       &sval->integer, &sval->fraction) == false) {
			return SENSOR_FAIL_TO_ACCESS;
		}
		break;
	default:
		LOG_ERR("Invalid access offset %d", pm8702_access);
		break;
	}

	return SENSOR_READ_SUCCESS;
}

uint8_t pm8702_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	cfg->read = pm8702_read;
	return SENSOR_INIT_SUCCESS;
}

#endif
