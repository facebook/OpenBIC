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
#include "pldm.h"

LOG_MODULE_DECLARE(pldm);

static uint8_t get_sensor_data_size(pldm_sensor_readings_data_type_t data_type)
{
	switch (data_type) {
	case PLDM_SENSOR_DATA_SIZE_UINT8:
	case PLDM_SENSOR_DATA_SIZE_SINT8:
		return PLDM_MONITOR_SENSOR_DATA_SIZE_INT8;
	case PLDM_SENSOR_DATA_SIZE_UINT16:
	case PLDM_SENSOR_DATA_SIZE_SINT16:
		return PLDM_MONITOR_SENSOR_DATA_SIZE_INT16;
	case PLDM_SENSOR_DATA_SIZE_UINT32:
	case PLDM_SENSOR_DATA_SIZE_SINT32:
		return PLDM_MONITOR_SENSOR_DATA_SIZE_INT32;
	default:
		LOG_ERR("Unsupport data type, (%d)", data_type);
		return 0;
	}
}

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
	res_p->sensor_data_size = get_sensor_data_size(PLDM_SENSOR_DATA_SIZE_UINT32);
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

uint16_t pldm_platform_monitor_read(void *mctp_inst, mctp_ext_params ext_params,
				    pldm_platform_monitor_commands_t cmd, uint8_t *req,
				    uint16_t req_len, uint8_t *rbuf, uint16_t rbuf_len)
{
	/* return read length */
	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, 0);
	CHECK_NULL_ARG_WITH_RETURN(req, 0);
	CHECK_NULL_ARG_WITH_RETURN(rbuf, 0);

	pldm_msg msg = { 0 };

	memcpy(&msg.ext_params, &ext_params, sizeof(mctp_ext_params));

	msg.hdr.pldm_type = PLDM_TYPE_PLAT_MON_CTRL;
	msg.hdr.cmd = cmd;
	msg.hdr.rq = 1;

	msg.buf = req;
	msg.len = req_len;

	LOG_HEXDUMP_DBG(msg.buf, msg.len, "Buf: ");

	return mctp_pldm_read(mctp_inst, &msg, rbuf, rbuf_len);
}

static uint8_t pldm_encode_sensor_event_data(struct pldm_sensor_event_data *sensor_event,
					     uint16_t sensor_id,
					     pldm_sensor_event_class_t sensor_event_class,
					     const uint8_t *sensor_event_data,
					     uint8_t event_data_length)
{
	CHECK_NULL_ARG_WITH_RETURN(sensor_event, PLDM_ERROR_INVALID_DATA);
	CHECK_NULL_ARG_WITH_RETURN(sensor_event_data, PLDM_ERROR_INVALID_DATA);

	if ((sensor_event_class == PLDM_SENSOR_OP_STATE)) {
		if (event_data_length != PLDM_MONITOR_SENSOR_EVENT_SENSOR_OP_STATE_DATA_LENGTH)
			return PLDM_ERROR_INVALID_LENGTH;
	} else if (sensor_event_class == PLDM_STATE_SENSOR_STATE) {
		if (event_data_length != PLDM_MONITOR_SENSOR_EVENT_STATE_SENSOR_STATE_DATA_LENGTH)
			return PLDM_ERROR_INVALID_LENGTH;
	} else if (sensor_event_class == PLDM_NUMERIC_SENSOR_STATE) {
		if (event_data_length <
			    PLDM_MONITOR_SENSOR_EVENT_NUMERIC_SENSOR_STATE_MIN_DATA_LENGTH ||
		    event_data_length >
			    PLDM_MONITOR_SENSOR_EVENT_NUMERIC_SENSOR_STATE_MAX_DATA_LENGTH)
			return PLDM_ERROR_INVALID_LENGTH;
	} else {
		return PLDM_ERROR_INVALID_DATA;
	}

	sensor_event->sensor_id = sensor_id;
	sensor_event->sensor_event_class_type = sensor_event_class;
	memcpy(sensor_event->event_class_data, sensor_event_data, event_data_length);

	return PLDM_SUCCESS;
}

uint8_t pldm_send_sensor_event(void *mctp_inst, mctp_ext_params ext_params, uint16_t sensor_id,
			       pldm_sensor_event_class_t sensor_event_class,
			       const uint8_t *sensor_event_data, uint8_t event_data_length)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(sensor_event_data, PLDM_ERROR_INVALID_DATA);

	struct pldm_sensor_event_data sensor_event = { 0 };

	if (pldm_encode_sensor_event_data(&sensor_event, sensor_id, sensor_event_class,
					  sensor_event_data, event_data_length) != PLDM_SUCCESS) {
		LOG_ERR("Encode event data failed");
		return PLDM_ERROR;
	}

	return pldm_platform_event_request(
		mctp_inst, ext_params, PLDM_SENSOR_EVENT, (uint8_t *)&sensor_event,
		sizeof(struct pldm_sensor_event_data) + event_data_length - 1);
}

uint8_t pldm_send_effecter_event(void *mctp_inst, mctp_ext_params ext_params,
				 struct pldm_effecter_event_data event_data)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, PLDM_ERROR);

	if (event_data.effecter_event_class != PLDM_EFFECTER_OP_STATE) {
		LOG_ERR("Unsupport effecter event class, (%d)", event_data.effecter_event_class);
		return PLDM_ERROR;
	}

	return pldm_platform_event_request(mctp_inst, ext_params, PLDM_EFFECTER_EVENT,
					   (uint8_t *)&event_data,
					   sizeof(struct pldm_effecter_event_data));
}

uint8_t pldm_platform_event_request(void *mctp_inst, mctp_ext_params ext_params,
				    uint8_t event_class, const uint8_t *event_data,
				    uint8_t event_data_length)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(event_data, PLDM_ERROR_INVALID_DATA);

	uint8_t req_len = sizeof(struct pldm_platform_event_message_req) + event_data_length - 1;
	uint8_t resp_len = sizeof(struct pldm_platform_event_message_resp);
	uint8_t rbuf[resp_len];

	struct pldm_platform_event_message_req req = { 0 };
	struct pldm_platform_event_message_resp *resp_p =
		(struct pldm_platform_event_message_resp *)rbuf;

	req.event_class = event_class;
	req.format_version = 0x01;
	req.tid = DEFAULT_TID;

	memcpy(&req.event_data, event_data, event_data_length);

	uint16_t read_len = pldm_platform_monitor_read(mctp_inst, ext_params,
						       PLDM_MONITOR_CMD_CODE_PLATFORM_EVENT_MESSAGE,
						       (uint8_t *)&req, req_len, rbuf, resp_len);

	if ((!read_len) || (resp_p->completion_code != PLDM_SUCCESS)) {
		LOG_ERR("Send event message failed, read_len (%d) comp_code (0x%x)", read_len,
			resp_p->completion_code);
		return PLDM_ERROR;
	}

	if (resp_p->platform_event_status != PLDM_EVENT_LOGGED) {
		LOG_ERR("Event not logged, status (0x%x)", resp_p->platform_event_status);
		return PLDM_ERROR;
	}

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
