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
#include <logging/log.h>
#include "libutil.h"
#include "mctp.h"
#include "sensor.h"
#ifdef ENABLE_VISTARA
#include "cci.h"
#include "vistara.h"

LOG_MODULE_REGISTER(vistara);

__weak uint8_t plat_get_cxl_eid(uint8_t cxl_id)
{
	return 0;
}

bool vistara_read_ddr_temp(uint8_t cxl_eid, uint8_t *resp)
{
	CHECK_NULL_ARG_WITH_RETURN(resp, false);

	mctp_cci_msg cci_msg = { 0 };
	cci_msg.hdr.op = CCI_OEM_OP_READ_DDR_TEMP;
	cci_msg.hdr.pl_len = READ_DDR_TEMP_REQ_LEN;

	if (!vistara_cci_command(cxl_eid, cci_msg, resp, READ_DDR_TEMP_RESP_LEN)) {
		LOG_DBG("Failed to read Vistara DDR temperature from EID %d", cxl_eid);
		return false;
	}

	return true;
}

bool vistara_cci_command(uint8_t cxl_eid, mctp_cci_msg cci_msg, uint8_t *resp, uint8_t resp_len)
{
	CHECK_NULL_ARG_WITH_RETURN(resp, false);

	mctp *mctp_inst = NULL;
	mctp_ext_params ext_params = { 0 };

	if (!get_mctp_info_by_eid(cxl_eid, &mctp_inst, &ext_params)) {
		LOG_ERR("Fail to get mctp info via eid: %d", cxl_eid);
		return false;
	}

	memcpy(&cci_msg.ext_params, &ext_params, sizeof(mctp_ext_params));
	if (mctp_cci_read(mctp_inst, &cci_msg, resp, resp_len) != resp_len) {
		LOG_ERR("CCI command 0x%x read fail", cci_msg.hdr.op);
		return false;
	}

	k_msleep(50);

	return true;
}

uint8_t vistara_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	uint8_t cxl_id = cfg->port;
	uint8_t cxl_eid = plat_get_cxl_eid(cxl_id);
	uint8_t dimm_id = cfg->target_addr;
	uint8_t sensor_type = cfg->arg0;

	sensor_val *sval = (sensor_val *)reading;
	float f_val = 0;

	switch (sensor_type) {
	case DDR_TEMP: {
		uint8_t resp_buf[READ_DDR_TEMP_RESP_LEN];
		if (!vistara_read_ddr_temp(cxl_eid, resp_buf)) {
			return SENSOR_FAIL_TO_ACCESS;
		} else {
			read_ddr_temp_resp *ddr_temp =
				(read_ddr_temp_resp *)(resp_buf + dimm_id * 8);
			f_val = *((float *)&ddr_temp->dimm_temp);
			LOG_HEXDUMP_DBG(ddr_temp->dimm_temp, sizeof(float), "ddr temp");
		}
	} break;
	default:
		LOG_ERR("Invalid vistara sensor type 0x%x", sensor_type);
		return SENSOR_FAIL_TO_ACCESS;
		break;
	}

	sval->integer = (int)f_val & 0xFFFF;
	sval->fraction = (f_val - sval->integer) * 1000;

	return SENSOR_READ_SUCCESS;
}

uint8_t vistara_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	cfg->read = vistara_read;
	return SENSOR_INIT_SUCCESS;
}

#endif
