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

	uint8_t resp_buf[10] = { 0 };
	uint8_t req_len = sizeof(struct pldm_get_sensor_reading_req);
	struct pldm_get_sensor_reading_req req = { 0 };
	req.sensor_id = init_arg->sensor_id;
	req.rearm_event_state = 0;

	uint16_t resp_len =
		pldm_platform_monitor_read(mctp_inst, ext_params,
					   PLDM_MONITOR_CMD_CODE_GET_SENSOR_READING,
					   (uint8_t *)&req, req_len, resp_buf, sizeof(resp_buf));

	if (resp_len == 0) {
		LOG_ERR("Failed to get CX7 sensor #%d reading", init_arg->sensor_id);

		LOG_INF("Attempting to re-initialize EID 0x%x for CX7 device due to sensor read failure",
			mctp_dest_eid);

		if (init_arg->re_init_eid_fn) {
			init_arg->re_init_eid_fn();
		}

		k_msleep(50); // Wait for EID re-initialization to complete
		resp_len = pldm_platform_monitor_read(mctp_inst, ext_params,
						      PLDM_MONITOR_CMD_CODE_GET_SENSOR_READING,
						      (uint8_t *)&req, req_len, resp_buf,
						      sizeof(resp_buf));

		if (resp_len == 0) {
			LOG_ERR("Failed to get CX7 sensor #%d reading after EID re-initialization",
				init_arg->sensor_id);
			return SENSOR_FAIL_TO_ACCESS;
		}
	}

	struct pldm_get_sensor_reading_resp *res = (struct pldm_get_sensor_reading_resp *)resp_buf;

	if ((res->completion_code != PLDM_SUCCESS)) {
		LOG_ERR("Failed to get get sensor reading, completion_code = 0x%x",
			res->completion_code);
		return SENSOR_FAIL_TO_ACCESS;
	}

	sensor_val *sval = (sensor_val *)reading;
	if (res->sensor_operational_state == PLDM_SENSOR_ENABLED) {
		if (res->sensor_data_size != PLDM_SENSOR_DATA_SIZE_SINT16) {
			sval->integer = (res->present_reading[1] << 8) | res->present_reading[0];
			sval->fraction = 0;
			return SENSOR_READ_SUCCESS;
		} else if (res->sensor_data_size != PLDM_SENSOR_DATA_SIZE_SINT8) {
			sval->integer = res->present_reading[0];
			sval->fraction = 0;
			return SENSOR_READ_SUCCESS;
		}
	} else if (res->sensor_operational_state == PLDM_SENSOR_UNAVAILABLE) {
		return SENSOR_UNAVAILABLE;
	}

	LOG_ERR("CX7 Failed to get get sensor reading, sensor operational state=0x%x",
		res->sensor_operational_state);
	return SENSOR_FAIL_TO_ACCESS;
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
