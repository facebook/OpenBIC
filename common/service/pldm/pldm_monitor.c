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

#include <logging/log.h>
#include <string.h>
#include <stdio.h>
#include "sensor.h"
#include "pldm_monitor.h"

uint8_t pldm_get_sensor_reading(void *mctp_inst, uint8_t *buf, uint16_t len, uint8_t *resp,
				uint16_t *resp_len, void *ext_params)
{
	if (!mctp_inst || !buf || !resp || !resp_len)
		return PLDM_ERROR;

	struct pldm_get_sensor_reading_req *req_p = (struct pldm_get_sensor_reading_req *)buf;
	struct pldm_get_sensor_reading_resp *res_p = (struct pldm_get_sensor_reading_resp *)resp;

	if (len != PLDM_GET_SENSOR_READING_REQ_BYTES) {
		res_p->completion_code = PLDM_ERROR_INVALID_LENGTH;
		res_p->sensor_operational_state = PLDM_SENSOR_STATUSUNKOWN;
		goto ret;
	}
	/* Only support one byte range of sensor number */
	if (req_p->sensor_id > PLDM_MONITOR_SENSOR_SUPPORT_MAX) {
		res_p->completion_code = PLDM_PLATFORM_INVALID_SENSOR_ID;
		res_p->sensor_operational_state = PLDM_SENSOR_STATUSUNKOWN;
		goto ret;
	}

	uint8_t sensor_number = (uint8_t)req_p->sensor_id;
	uint8_t status = -1;
	int reading = 0;

	status = get_sensor_reading(sensor_number, &reading, GET_FROM_CACHE);

	switch (status) {
	case SENSOR_READ_SUCCESS:
	case SENSOR_READ_ACUR_SUCCESS:
	case SENSOR_READ_4BYTE_ACUR_SUCCESS:
		res_p->completion_code = PLDM_SUCCESS;
		res_p->sensor_operational_state = PLDM_SENSOR_ENABLED;
		break;
	case SENSOR_NOT_ACCESSIBLE:
	case SENSOR_INIT_STATUS:
		res_p->completion_code = PLDM_SUCCESS;
		res_p->sensor_operational_state = PLDM_SENSOR_INITIALIZING;
		break;
	case SENSOR_POLLING_DISABLE:
		res_p->completion_code = PLDM_SUCCESS;
		res_p->sensor_operational_state = PLDM_SENSOR_STATUSUNKOWN;
		break;
	case SENSOR_NOT_FOUND:
		// request sensor number not found
		res_p->completion_code = PLDM_PLATFORM_INVALID_SENSOR_ID;
		res_p->sensor_operational_state = PLDM_SENSOR_STATUSUNKOWN;
		break;
	case SENSOR_FAIL_TO_ACCESS:
	case SENSOR_UNSPECIFIED_ERROR:
	default:
		res_p->completion_code = PLDM_SUCCESS;
		res_p->sensor_operational_state = PLDM_SENSOR_FAILED;
		break;
	}

ret:
	/* Only support 4-bytes unsinged sensor data */
	res_p->sensor_data_size = PLDM_SENSOR_DATA_SIZE_UINT32;
	res_p->sensor_event_message_enable = PLDM_EVENTS_DISABLED;
	res_p->previous_state =
		(res_p->completion_code == PLDM_SUCCESS) ? PLDM_SENSOR_NORMAL : PLDM_SENSOR_UNKNOWN;
	res_p->present_state =
		(res_p->completion_code == PLDM_SUCCESS) ? PLDM_SENSOR_NORMAL : PLDM_SENSOR_UNKNOWN;
	res_p->event_state =
		(res_p->completion_code == PLDM_SUCCESS) ? PLDM_SENSOR_NORMAL : PLDM_SENSOR_UNKNOWN;

	if ((res_p->completion_code != PLDM_SUCCESS) ||
	    (res_p->sensor_operational_state != PLDM_SENSOR_ENABLED))
		reading = -1;

	memcpy(res_p->present_reading, &reading, sizeof(reading));
	*resp_len = sizeof(struct pldm_get_sensor_reading_resp) + res_p->sensor_data_size - 1;
	return PLDM_SUCCESS;
}

static pldm_cmd_handler pldm_monitor_cmd_tbl[] = {
	{ PLDM_MONITOR_CMD_CODE_GET_SENSOR_READING, pldm_get_sensor_reading },
};

uint8_t pldm_monitor_handler_query(uint8_t code, void **ret_fn)
{
	if (!ret_fn)
		return PLDM_ERROR;

	pldm_cmd_proc_fn fn = NULL;
	uint8_t i;

	for (i = 0; i < ARRAY_SIZE(pldm_monitor_cmd_tbl); i++) {
		if (pldm_monitor_cmd_tbl[i].cmd_code == code) {
			fn = pldm_monitor_cmd_tbl[i].fn;
			break;
		}
	}

	*ret_fn = (void *)fn;
	return fn ? PLDM_SUCCESS : PLDM_ERROR;
}