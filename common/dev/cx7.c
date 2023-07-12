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
#include <stdint.h>
#include <string.h>
#include <logging/log.h>
#include "libutil.h"
#include "sensor.h"
#include "pldm.h"
#include "hal_i2c.h"

LOG_MODULE_REGISTER(cx7);

uint8_t cx7_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	cx7_init_arg *init_arg = (cx7_init_arg *)cfg->init_args;

	uint8_t mctp_dest_eid = init_arg->endpoint;

	mctp *mctp_inst = NULL;
	mctp_ext_params ext_params = { 0 };
	if (get_mctp_info_by_eid(mctp_dest_eid, &mctp_inst, &ext_params) == false) {
		LOG_ERR("Failed to get mctp info by eid 0x%x", mctp_dest_eid);
		return SENSOR_FAIL_TO_ACCESS;
	}

	uint8_t req_buf[10] = { 0 };
	uint8_t resp_buf[10] = { 0 };
	pldm_msg pmsg = { 0 };
	pmsg.ext_params = ext_params;
	pmsg.hdr.msg_type = MCTP_MSG_TYPE_PLDM;
	pmsg.hdr.pldm_type = PLDM_TYPE_PLAT_MON_CTRL;
	pmsg.hdr.cmd = 0x11;
	pmsg.hdr.rq = PLDM_REQUEST;
	pmsg.len = sizeof(struct pldm_get_sensor_reading_req);

	pmsg.buf = req_buf;
	struct pldm_get_sensor_reading_req *cmd_req =
		(struct pldm_get_sensor_reading_req *)pmsg.buf;
	cmd_req->sensor_id = init_arg->sensor_id;
	cmd_req->rearm_event_state = 0;

	uint16_t resp_len = mctp_pldm_read(mctp_inst, &pmsg, resp_buf, sizeof(resp_buf));
	if (resp_len == 0) {
		LOG_ERR("Failed to get mctp info by eid 0x%x", mctp_dest_eid);
		return SENSOR_FAIL_TO_ACCESS;
	}

	if (resp_buf[0] != PLDM_SUCCESS) {
		LOG_ERR("Failed to get get sensor reading");
		return SENSOR_FAIL_TO_ACCESS;
	} else {
		sensor_val *sval = (sensor_val *)reading;
		sval->integer = (resp_buf[8] << 8) | resp_buf[7];
		sval->fraction = 0;
		return SENSOR_READ_SUCCESS;
	}
}

uint8_t cx7_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	cfg->read = cx7_read;
	return SENSOR_INIT_SUCCESS;
}
