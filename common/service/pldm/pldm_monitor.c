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
#include <stdlib.h>
#include <drivers/spi_nor.h>
#include <drivers/flash.h>
#include "sensor.h"
#include "plat_def.h"
#ifdef ENABLE_PLDM_SENSOR
#include "pldm_sensor.h"
#endif
#include "pldm.h"
#include "pdr.h"
#include "hal_gpio.h"

#ifndef PLDM_MONITOR_EVENT_QUEUE_MSG_NUM_MAX
#define PLDM_MONITOR_EVENT_QUEUE_MSG_NUM_MAX PLDM_MONITOR_EVENT_QUEUE_MSG_NUM_MAX_DEFAULT
#endif

LOG_MODULE_DECLARE(pldm);

K_FIFO_DEFINE(send_event_pkt_fifo);

struct pldm_event_pkt {
	void *fifo_reserved; /* 1st word reserved for use by fifo */
	uint8_t event_class;
	uint16_t id;
	uint8_t ext_class;
	uint8_t event_data[PLDM_MONITOR_EVENT_DATA_SIZE_MAX];
	uint8_t event_data_length;
};

static struct pldm_event_receiver_info {
	mctp *mctp_inst_p;
	mctp_ext_params ext_params;
} event_receiver_info = {
	.mctp_inst_p = NULL,
	.ext_params = { 0 },
};

struct pldm_state_effecter_info *state_effecter_table;

uint16_t pldm_state_effecter_index = 0;

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
		LOG_ERR("Unsupported data type, (%d)", data_type);
		return 0;
	}
}

float pldm_sensor_cal(uint8_t *buf, uint8_t len, pldm_sensor_readings_data_type_t data_type,
		      pldm_sensor_pdr_parm parm)
{
	CHECK_NULL_ARG_WITH_RETURN(buf, 0);

	float val = 0;

	switch (data_type) {
	case PLDM_SENSOR_DATA_SIZE_UINT8:
	case PLDM_SENSOR_DATA_SIZE_SINT8:
		if (len != PLDM_MONITOR_SENSOR_DATA_SIZE_INT8) {
			LOG_ERR("Buffer length not mach with given type");
			goto exit;
		}
		if (data_type == PLDM_SENSOR_DATA_SIZE_UINT8)
			val = (uint8_t)*buf;
		else
			val = (int8_t)*buf;
		break;
	case PLDM_SENSOR_DATA_SIZE_UINT16:
	case PLDM_SENSOR_DATA_SIZE_SINT16:
		if (len != PLDM_MONITOR_SENSOR_DATA_SIZE_INT16) {
			LOG_ERR("Buffer length not mach with given type");
			goto exit;
		}
		if (data_type == PLDM_SENSOR_DATA_SIZE_UINT16)
			val = (uint16_t)(buf[0] | (buf[1] << 8));
		else
			val = (int16_t)(buf[0] | (buf[1] << 8));
		break;
	case PLDM_SENSOR_DATA_SIZE_UINT32:
	case PLDM_SENSOR_DATA_SIZE_SINT32:
		if (len != PLDM_MONITOR_SENSOR_DATA_SIZE_INT32) {
			LOG_ERR("Buffer length not mach with given type");
			goto exit;
		}
		if (data_type == PLDM_SENSOR_DATA_SIZE_UINT32)
			val = (uint32_t)(buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24));
		else
			val = (int32_t)(buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24));
		break;
	default:
		LOG_ERR("Unsupported data type, (%d)", data_type);
		goto exit;
	}

	val = (parm.resolution * val + parm.ofst) * power(10, parm.unit_modifier);

exit:
	return val;
}

uint8_t pldm_get_sensor_reading(void *mctp_inst, uint8_t *buf, uint16_t len, uint8_t instance_id,
				uint8_t *resp, uint16_t *resp_len, void *ext_params)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(buf, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp_len, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(ext_params, PLDM_ERROR);

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
	uint8_t status;
	int reading = 0;

#ifdef ENABLE_PLDM_SENSOR
	uint8_t sensor_operational_state = PLDM_SENSOR_STATUSUNKOWN;

	status = pldm_sensor_get_reading_from_cache(sensor_number, &reading,
						    &sensor_operational_state);
	res_p->completion_code = status;
	res_p->sensor_operational_state = sensor_operational_state;
#else
	status = get_sensor_reading(sensor_config, sensor_config_count, sensor_number, &reading,
				    GET_FROM_CACHE);

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
	case SENSOR_UNAVAILABLE:
		res_p->completion_code = PLDM_SUCCESS;
		res_p->sensor_operational_state = PLDM_SENSOR_UNAVAILABLE;
		break;
	case SENSOR_FAIL_TO_ACCESS:
	case SENSOR_UNSPECIFIED_ERROR:
	default:
		res_p->completion_code = PLDM_SUCCESS;
		res_p->sensor_operational_state = PLDM_SENSOR_FAILED;
		break;
	}
#endif

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

static uint8_t pldm_send_sensor_event_message(void *mctp_inst, mctp_ext_params ext_params,
					      uint16_t sensor_id,
					      pldm_sensor_event_class_t sensor_event_class,
					      const uint8_t *sensor_event_data,
					      uint8_t event_data_length)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(sensor_event_data, PLDM_ERROR_INVALID_DATA);

	uint8_t ret = PLDM_ERROR;
	struct pldm_sensor_event_data *sensor_event_p = (struct pldm_sensor_event_data *)malloc(
		sizeof(struct pldm_sensor_event_data) + event_data_length);

	if (!sensor_event_p) {
		LOG_ERR("pldm_sensor_event_data malloc failed");
		return PLDM_ERROR;
	}
	if (pldm_encode_sensor_event_data(sensor_event_p, sensor_id, sensor_event_class,
					  sensor_event_data, event_data_length) != PLDM_SUCCESS) {
		LOG_ERR("Encode event data failed");
		goto exit;
	}

	ret = pldm_platform_event_message_req(
		mctp_inst, ext_params, PLDM_SENSOR_EVENT, (uint8_t *)sensor_event_p,
		sizeof(struct pldm_sensor_event_data) + event_data_length);

exit:
	SAFE_FREE(sensor_event_p);
	return ret;
}

static uint8_t pldm_send_effecter_event_message(void *mctp_inst, mctp_ext_params ext_params,
						uint16_t effecter_id,
						pldm_effecter_event_class_t effecter_event_class,
						const uint8_t *effecter_event_data,
						uint8_t event_data_length)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(effecter_event_data, PLDM_ERROR_INVALID_DATA);

	if (effecter_event_class != PLDM_EFFECTER_OP_STATE) {
		LOG_ERR("Unsupport effecter event class, (%d)", effecter_event_class);
		return PLDM_ERROR;
	}

	if (event_data_length != sizeof(struct pldm_effeter_event_op_state)) {
		LOG_ERR("Invalid event data length, (%d)", event_data_length);
		return PLDM_ERROR_INVALID_LENGTH;
	}

	struct pldm_effecter_event_data effecter_event = { 0 };
	effecter_event.effecter_id = effecter_id;
	effecter_event.effecter_event_class = effecter_event_class;
	memcpy(effecter_event.event_class_data, effecter_event_data, event_data_length);

	return pldm_platform_event_message_req(
		mctp_inst, ext_params, PLDM_EFFECTER_EVENT, (uint8_t *)&effecter_event,
		sizeof(struct pldm_effecter_event_data) + event_data_length - 1);
}

uint8_t pldm_platform_event_message_req(void *mctp_inst, mctp_ext_params ext_params,
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
	req.tid = plat_pldm_get_tid();

	memcpy(&req.event_data, event_data, event_data_length);

	resp_p->completion_code = PLDM_ERROR;
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

__weak void plat_send_event_pre_work()
{
	return;
}

static void process_event_message_queue(struct k_work *work)
{
	CHECK_NULL_ARG(work);
	struct pldm_event_pkt *pkt;

	plat_send_event_pre_work();

	if ((pkt = k_fifo_get(&send_event_pkt_fifo, K_NO_WAIT)) != NULL) {
		if (pldm_send_platform_event(pkt->event_class, pkt->id, pkt->ext_class,
					     pkt->event_data,
					     pkt->event_data_length) != PLDM_SUCCESS) {
			LOG_ERR("Send event failed, event_class (0x%x) id (0x%x) ext_class (%x)",
				pkt->event_class, pkt->id, pkt->ext_class);
			LOG_HEXDUMP_ERR(pkt->event_data, pkt->event_data_length, "Event data:");
		} else {
			LOG_DBG("Send event succeeded, event_class (0x%x) id (0x%x) ext_class (%x)",
				pkt->event_class, pkt->id, pkt->ext_class);
			LOG_HEXDUMP_DBG(pkt->event_data, pkt->event_data_length, "Event data:");
		}

		SAFE_FREE(pkt);
		k_work_schedule((struct k_work_delayable *)work, K_SECONDS(1));
	} else {
		LOG_INF("The event packet queue is empty, send the event work complete.");
	}
}

K_WORK_DELAYABLE_DEFINE(send_event_pkt_work, process_event_message_queue);

static uint8_t send_event_to_queue(uint8_t event_class, uint16_t id, uint8_t ext_class,
				   const uint8_t *event_data, uint8_t event_data_length)
{
	CHECK_NULL_ARG_WITH_RETURN(event_data, PLDM_ERROR);

	static uint8_t count = 0;
	struct pldm_event_pkt *pkt;

	if (event_data_length > PLDM_MONITOR_EVENT_DATA_SIZE_MAX) {
		LOG_ERR("Invalid event data length, (%d)", event_data_length);
		return PLDM_ERROR_INVALID_LENGTH;
	}

	if (count > PLDM_MONITOR_EVENT_QUEUE_MSG_NUM_MAX - 1) {
		LOG_ERR("Number of messages in the queue has reached maximum");
		return PLDM_ERROR;
	}

	pkt = (struct pldm_event_pkt *)malloc(sizeof(struct pldm_event_pkt));

	if (!pkt) {
		LOG_ERR("Event packet memory allocate failed");
		return PLDM_ERROR;
	}

	pkt->event_class = event_class;
	pkt->id = id;
	pkt->ext_class = ext_class;
	pkt->event_data_length = event_data_length;

	memcpy(pkt->event_data, event_data, event_data_length);

	k_fifo_put(&send_event_pkt_fifo, pkt);
	count++;

	return PLDM_SUCCESS;
}

uint8_t pldm_send_platform_event(uint8_t event_class, uint16_t id, uint8_t ext_class,
				 const uint8_t *event_data, uint8_t event_data_length)
{
	CHECK_NULL_ARG_WITH_RETURN(event_data, PLDM_ERROR);

	struct pldm_event_receiver_info *event_receiver_info_p = &event_receiver_info;

	if (!event_receiver_info_p->mctp_inst_p) {
		return send_event_to_queue(event_class, id, ext_class, event_data,
					   event_data_length);
	}

	switch (event_class) {
	case PLDM_SENSOR_EVENT:
		return pldm_send_sensor_event_message(event_receiver_info_p->mctp_inst_p,
						      event_receiver_info_p->ext_params, id,
						      ext_class, event_data, event_data_length);
	case PLDM_EFFECTER_EVENT:
		return pldm_send_effecter_event_message(event_receiver_info_p->mctp_inst_p,
							event_receiver_info_p->ext_params, id,
							ext_class, event_data, event_data_length);
	default:
		LOG_ERR("Unsupported event class, (%d)", event_class);
		return PLDM_ERROR;
	}
}

uint8_t pldm_set_event_receiver(void *mctp_inst, uint8_t *buf, uint16_t len, uint8_t instance_id,
				uint8_t *resp, uint16_t *resp_len, void *ext_params)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(buf, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp_len, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(ext_params, PLDM_ERROR);

	struct pldm_set_event_receiver_req *req_p = (struct pldm_set_event_receiver_req *)buf;
	uint8_t *completion_code_p = resp;
	mctp_ext_params *ext_params_p = (mctp_ext_params *)ext_params;
	*resp_len = 1;

	if (len != sizeof(struct pldm_set_event_receiver_req)) {
		*completion_code_p = PLDM_ERROR_INVALID_LENGTH;
		return PLDM_SUCCESS;
	}

	if (req_p->event_message_global_enable >
	    PLDM_EVENT_MESSAGE_GLOBAL_ENABLE_ASYNC_KEEP_ALIVE) {
		LOG_ERR("Unsupport event message global enable, (%d)",
			req_p->event_message_global_enable);
		*completion_code_p = PLDM_ERROR_INVALID_DATA;
		return PLDM_SUCCESS;
	}

	/* Only support MCTP protocol type currently */
	if (req_p->transport_protocol_type != PLDM_TRANSPORT_PROTOCOL_TYPE_MCTP) {
		LOG_ERR("Unsupport transport protocol type, (%d)", req_p->transport_protocol_type);
		*completion_code_p = PLDM_PLATFORM_INVALID_PROTOCOL_TYPE;
		return PLDM_SUCCESS;
	}

	if (req_p->event_receiver_address_info != ext_params_p->ep) {
		LOG_ERR("Require set event receiver EID (0x%x) not match command sender (0x%x)",
			req_p->event_receiver_address_info, ext_params_p->ep);
		*completion_code_p = PLDM_ERROR_INVALID_DATA;
		return PLDM_SUCCESS;
	}

	event_receiver_info.mctp_inst_p = (mctp *)mctp_inst;
	memcpy(&event_receiver_info.ext_params, ext_params_p, sizeof(*ext_params_p));

	*completion_code_p = PLDM_SUCCESS;

	k_work_schedule(&send_event_pkt_work, K_MSEC(1000));

	return PLDM_SUCCESS;
}

/**
 * @brief PLDM event length check function
 *
 * Check given event data length meet with spec.
 *
 * @param buf message data start from event class type
 * @param len Length of buf in bytes
 *
 * @return PLDM_BASE return value 
 */
uint8_t pldm_event_len_check(uint8_t *buf, uint16_t len)
{
	CHECK_NULL_ARG_WITH_RETURN(buf, PLDM_ERROR);

	if (len == 0)
		return PLDM_ERROR_INVALID_LENGTH;

	uint8_t *p = buf;

	uint8_t event_class = *p;
	p++;
	len--;

	switch (event_class) {
	case PLDM_SENSOR_EVENT:
		/* exclude sensorID, sensorEventClass */
		if ((len - 3) <= 0) {
			return PLDM_ERROR_INVALID_LENGTH;
		}

		p += 2;
		len -= 2;

		uint8_t sensor_event_class = *p;

		if (sensor_event_class == PLDM_SENSOR_OP_STATE) {
			if (len != sizeof(struct pldm_sensor_event_op_exp_data))
				return PLDM_ERROR_INVALID_LENGTH;
		} else if (sensor_event_class == PLDM_STATE_SENSOR_STATE) {
			if (len != sizeof(struct pldm_sensor_event_state_exp_data))
				return PLDM_ERROR_INVALID_LENGTH;
		} else if (sensor_event_class == PLDM_NUMERIC_SENSOR_STATE) {
			if (len < sizeof(struct pldm_sensor_event_numeric_exp_data))
				return PLDM_ERROR_INVALID_LENGTH;
		} else {
			LOG_ERR("Received invalid sensor event class 0x%x", sensor_event_class);
			return PLDM_ERROR;
		}
		break;

	default:
		LOG_WRN("Given event class 0x%x not supported", event_class);
		return PLDM_ERROR;
	}

	return PLDM_SUCCESS;
}

__weak uint8_t pldm_platform_event_message(void *mctp_inst, uint8_t *buf, uint16_t len,
					   uint8_t instance_id, uint8_t *resp, uint16_t *resp_len,
					   void *ext_params)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(buf, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp_len, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(ext_params, PLDM_ERROR);

	struct pldm_platform_event_message_req *req_p =
		(struct pldm_platform_event_message_req *)buf;
	struct pldm_platform_event_message_resp *res_p =
		(struct pldm_platform_event_message_resp *)resp;

	LOG_INF("Recieved event class 0x%x", req_p->event_class);
	LOG_HEXDUMP_WRN(req_p->event_data, len - 3, "event data:");

	res_p->completion_code = PLDM_ERROR_UNSUPPORTED_PLDM_CMD;
	res_p->platform_event_status = 0x00;

	*resp_len = 2;

	return PLDM_SUCCESS;
}

void set_effecter_state_gpio_handler(const uint8_t *buf, uint16_t len, uint8_t *resp,
				     uint16_t *resp_len, uint8_t gpio_pin)
{
	CHECK_NULL_ARG(buf);
	CHECK_NULL_ARG(resp);
	CHECK_NULL_ARG(resp_len);

	struct pldm_set_state_effecter_states_req *req_p =
		(struct pldm_set_state_effecter_states_req *)buf;
	uint8_t *completion_code_p = resp;
	*resp_len = 1;

	/* Check whether the range of AST1030 gpio pins is correct */
	if (gpio_pin > PLDM_PLATFORM_OEM_AST1030_GPIO_PIN_NUM_MAX) {
		LOG_ERR("Unsupport GPIO pin number, (%d)", gpio_pin);
		*completion_code_p = PLDM_OEM_GPIO_UNSUPPORT_RANGE;
		return;
	}

	if (req_p->composite_effecter_count != PLDM_PLATFORM_OEM_GPIO_EFFECTER_STATE_FIELD_COUNT) {
		LOG_ERR("Unsupport GPIO effecter count, (%d)", req_p->composite_effecter_count);
		*completion_code_p = PLDM_ERROR_INVALID_DATA;
		return;
	}

	set_effecter_state_field_t *gpio_dir_state = &req_p->field[0];
	set_effecter_state_field_t *gpio_val_state = &req_p->field[1];

	/* Not support change GPIO direction and GPIO direciton value doesn't mater */
	if ((gpio_dir_state->set_request != PLDM_NO_CHANGE) ||
	    (gpio_val_state->set_request >= PLDM_SET_REQUEST_MAX)) {
		LOG_ERR("Unsupport GPIO effecter set request, direction (%d) value (%d)",
			gpio_dir_state->set_request, gpio_val_state->set_request);
		*completion_code_p = PLDM_PLATFORM_UNSUPPORTED_EFFECTERSTATE;
		return;
	}

	if ((gpio_dir_state->effecter_state >= EFFECTER_STATE_GPIO_DIRECTION_MAX) ||
	    (gpio_val_state->effecter_state >= EFFECTER_STATE_GPIO_VALUE_MAX)) {
		LOG_ERR("Unsupport GPIO effecter state, direction (%d) value (%d)",
			gpio_dir_state->effecter_state, gpio_val_state->effecter_state);
		*completion_code_p = PLDM_PLATFORM_INVALID_STATE_VALUE;
		return;
	}

	/* Set output pin value only */
	if (gpio_cfg[gpio_pin].direction == GPIO_INPUT) {
		LOG_ERR("Can't set input pin (%d) value", gpio_pin);
		*completion_code_p = PLDM_OEM_GPIO_EFFECTER_VALUE_UNKNOWN;
		return;
	} else {
		if (gpio_val_state->effecter_state == EFFECTER_STATE_GPIO_VALUE_UNKNOWN) {
			*completion_code_p = PLDM_OEM_GPIO_EFFECTER_VALUE_UNKNOWN;
			return;
		} else {
			uint8_t gpio_val =
				((gpio_val_state->effecter_state == EFFECTER_STATE_GPIO_VALUE_LOW) ?
					 GPIO_LOW :
					 GPIO_HIGH);
			gpio_set(gpio_pin, gpio_val);
			*completion_code_p = PLDM_SUCCESS;
			return;
		}
	}
}

void pldm_spi_reinit(const char *spi_dev_str, const uint8_t *buf, uint16_t len, uint8_t *resp,
		     uint16_t *resp_len)
{
	CHECK_NULL_ARG(buf);
	CHECK_NULL_ARG(resp);
	CHECK_NULL_ARG(resp_len);

	uint8_t *completion_code_p = resp;
	*resp_len = 1;

	struct pldm_set_state_effecter_states_req *req_p =
		(struct pldm_set_state_effecter_states_req *)buf;

	if (req_p->composite_effecter_count !=
	    PLDM_PLATFORM_OEM_SPI_REINIT_EFFECTER_STATE_FIELD_COUNT) {
		LOG_ERR("Unsupported SPI reinit effecter count, (%d)",
			req_p->composite_effecter_count);
		*completion_code_p = PLDM_ERROR_INVALID_DATA;
		return;
	}

	set_effecter_state_field_t *reinit_spi_state = &req_p->field[0];

	switch (reinit_spi_state->set_request) {
	case PLDM_NO_CHANGE:
		return;
	case PLDM_REQUEST_SET:
		break;
	default:
		LOG_ERR("Invalid reinit SPI reinit set request (%d)",
			reinit_spi_state->set_request);
		*completion_code_p = PLDM_PLATFORM_UNSUPPORTED_EFFECTERSTATE;
		return;
	}

	const struct device *flash_dev_spi;

	flash_dev_spi = device_get_binding(spi_dev_str);

	switch (reinit_spi_state->effecter_state) {
	case EFFECTER_STATE_SPI_REINIT:
		spi_nor_re_init(flash_dev_spi);
		break;
	default:
		LOG_ERR("Unsupported reinit i3c hub effecter state, (%d)",
			reinit_spi_state->effecter_state);
		*completion_code_p = PLDM_ERROR_INVALID_DATA;
	}
}

__weak uint8_t plat_pldm_set_state_effecter_state_handler(const uint8_t *buf, uint16_t len,
							  uint8_t *resp, uint16_t *resp_len,
							  struct pldm_state_effecter_info *info_p)
{
	CHECK_NULL_ARG_WITH_RETURN(buf, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp_len, PLDM_ERROR);
	CHECK_ARG_WITH_RETURN(!len, PLDM_ERROR);

	uint8_t *completion_code_p = resp;
	*resp_len = 1;
	LOG_WRN("Not implemented in the platform code");
	*completion_code_p = PLDM_ERROR_UNSUPPORTED_PLDM_CMD;

	return PLDM_SUCCESS;
}

__weak uint8_t plat_pldm_get_state_effecter_state_handler(const uint8_t *buf, uint16_t len,
							  uint8_t *resp, uint16_t *resp_len,
							  struct pldm_state_effecter_info *info_p)
{
	CHECK_NULL_ARG_WITH_RETURN(buf, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp_len, PLDM_ERROR);
	CHECK_ARG_WITH_RETURN(!len, PLDM_ERROR);

	uint8_t *completion_code_p = resp;
	*resp_len = 1;
	LOG_WRN("Not implemented in the platform code");
	*completion_code_p = PLDM_ERROR_UNSUPPORTED_PLDM_CMD;

	return PLDM_SUCCESS;
}

uint8_t pldm_set_state_effecter_states(void *mctp_inst, uint8_t *buf, uint16_t len,
				       uint8_t instance_id, uint8_t *resp, uint16_t *resp_len,
				       void *ext_params)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(buf, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp_len, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(ext_params, PLDM_ERROR);

	struct pldm_set_state_effecter_states_req *req_p =
		(struct pldm_set_state_effecter_states_req *)buf;
	uint8_t *completion_code_p = resp;
	*resp_len = 1;

	if (req_p->composite_effecter_count < 0x01 || req_p->composite_effecter_count > 0x08) {
		*completion_code_p = PLDM_ERROR_INVALID_DATA;
		return PLDM_SUCCESS;
	}

	if (len != (PLDM_SET_STATE_EFFECTER_REQ_NO_STATE_FIELD_BYTES +
		    sizeof(set_effecter_state_field_t) * req_p->composite_effecter_count)) {
		*completion_code_p = PLDM_ERROR_INVALID_LENGTH;
		return PLDM_SUCCESS;
	}

	struct pldm_state_effecter_info *info_p = find_state_effecter_info(req_p->effecter_id);

	if (!info_p) {
		LOG_ERR("Can't find effecter ID (0x%x) info", req_p->effecter_id);
		*completion_code_p = PLDM_ERROR_INVALID_DATA;
		return PLDM_SUCCESS;
	}

	return plat_pldm_set_state_effecter_state_handler(buf, len, resp, resp_len, info_p);
}

void get_effecter_state_gpio_handler(const uint8_t *buf, uint16_t len, uint8_t *resp,
				     uint16_t *resp_len, uint8_t gpio_pin)
{
	CHECK_NULL_ARG(buf);
	CHECK_NULL_ARG(resp);
	CHECK_NULL_ARG(resp_len);

	struct pldm_get_state_effecter_states_resp *res_p =
		(struct pldm_get_state_effecter_states_resp *)resp;

	if (gpio_pin > PLDM_PLATFORM_OEM_AST1030_GPIO_PIN_NUM_MAX) {
		LOG_ERR("Unsupport GPIO pin number, (%d)", gpio_pin);
		res_p->completion_code = PLDM_OEM_GPIO_UNSUPPORT_RANGE;
		return;
	}

	get_effecter_state_field_t *gpio_dir_state = &res_p->field[0];
	get_effecter_state_field_t *gpio_val_state = &res_p->field[1];

	if (!gpio_cfg[gpio_pin].is_init) {
		LOG_WRN("Pin %d is not configured as GPIO", gpio_pin);
		gpio_dir_state->effecter_op_state = gpio_val_state->effecter_op_state =
			PLDM_EFFECTER_DISABLED;
		gpio_dir_state->present_state = gpio_dir_state->pending_state =
			EFFECTER_STATE_GPIO_DIRECTION_UNKNOWN;
		gpio_val_state->present_state = gpio_val_state->pending_state =
			EFFECTER_STATE_GPIO_VALUE_UNKNOWN;
	} else {
		gpio_dir_state->effecter_op_state = gpio_val_state->effecter_op_state =
			PLDM_EFFECTER_ENABLED_NOUPDATEPENDING;
		gpio_dir_state->present_state = gpio_dir_state->pending_state =
			((gpio_cfg[gpio_pin].direction == GPIO_INPUT) ?
				 EFFECTER_STATE_GPIO_DIRECTION_INPUT :
				 EFFECTER_STATE_GPIO_DIRECTION_OUTPUT);
		gpio_val_state->present_state = gpio_val_state->pending_state =
			(!gpio_get(gpio_pin) ? EFFECTER_STATE_GPIO_VALUE_LOW :
					       EFFECTER_STATE_GPIO_VALUE_HIGH);
	}

	*resp_len = PLDM_GET_STATE_EFFECTER_RESP_NO_STATE_FIELD_BYTES +
		    (sizeof(get_effecter_state_field_t) *
		     PLDM_PLATFORM_OEM_GPIO_EFFECTER_STATE_FIELD_COUNT);
	res_p->composite_effecter_count = PLDM_PLATFORM_OEM_GPIO_EFFECTER_STATE_FIELD_COUNT;
	res_p->completion_code = PLDM_SUCCESS;
}

uint8_t pldm_get_state_effecter_states(void *mctp_inst, uint8_t *buf, uint16_t len,
				       uint8_t instance_id, uint8_t *resp, uint16_t *resp_len,
				       void *ext_params)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(buf, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp_len, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(ext_params, PLDM_ERROR);

	struct pldm_get_state_effecter_states_req *req_p =
		(struct pldm_get_state_effecter_states_req *)buf;
	struct pldm_get_state_effecter_states_resp *res_p =
		(struct pldm_get_state_effecter_states_resp *)resp;
	uint8_t *completion_code_p = resp;
	*resp_len = 1;

	if (len != sizeof(struct pldm_get_state_effecter_states_req)) {
		res_p->completion_code = PLDM_ERROR_INVALID_LENGTH;
		return PLDM_SUCCESS;
	}

	struct pldm_state_effecter_info *info_p = find_state_effecter_info(req_p->effecter_id);

	if (!info_p) {
		LOG_ERR("Can't find effecter ID (0x%x) info", req_p->effecter_id);
		*completion_code_p = PLDM_ERROR_INVALID_DATA;
		return PLDM_SUCCESS;
	}

	return plat_pldm_get_state_effecter_state_handler(buf, len, resp, resp_len, info_p);
}

uint8_t pldm_get_pdr_info(void *mctp_inst, uint8_t *buf, uint16_t len, uint8_t instance_id,
			  uint8_t *resp, uint16_t *resp_len, void *ext_params)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(buf, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp_len, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(ext_params, PLDM_ERROR);

	struct pldm_get_pdr_info_resp *res_p = (struct pldm_get_pdr_info_resp *)resp;

	PDR_INFO *pdr_info = get_pdr_info();
	if (pdr_info == NULL) {
		LOG_ERR("The PDR table is empty");
		return PLDM_ERROR;
	}

	res_p->repository_state = pdr_info->repository_state;
	res_p->record_count = pdr_info->record_count;
	res_p->repository_size = pdr_info->repository_size;
	res_p->largest_record_size = pdr_info->largest_record_size;
	*resp_len = sizeof(struct pldm_get_pdr_info_resp);
	res_p->completion_code = PLDM_SUCCESS;

	return PLDM_SUCCESS;
}

uint8_t pldm_get_pdr(void *mctp_inst, uint8_t *buf, uint16_t len, uint8_t instance_id,
		     uint8_t *resp, uint16_t *resp_len, void *ext_params)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(buf, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp_len, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(ext_params, PLDM_ERROR);

	int response_count = 0;
	uint32_t record_count = get_record_count();
	struct pldm_get_pdr_req *req_p = (struct pldm_get_pdr_req *)buf;
	struct pldm_get_pdr_resp *res_p = (struct pldm_get_pdr_resp *)resp;

	response_count =
		get_pdr_table_via_record_handle(&res_p->record_data[0], req_p->record_handle);
	if (response_count <= 0) {
		LOG_ERR("Failed to get PDR for record handle: %x\n", req_p->record_handle);
		return PLDM_ERROR;
	}

	if (req_p->record_handle + 1 >= record_count) {
		res_p->next_record_handle = 0x00000000;
	} else {
		res_p->next_record_handle = req_p->record_handle + 1;
	}

	res_p->transfer_flag = PLDM_TRANSFER_FLAG_START_AND_END;
	res_p->response_count = response_count;
	*resp_len = sizeof(struct pldm_get_pdr_resp);
	res_p->completion_code = PLDM_SUCCESS;
	return PLDM_SUCCESS;
}

static pldm_cmd_handler pldm_monitor_cmd_tbl[] = {
	{ PLDM_MONITOR_CMD_CODE_GET_SENSOR_READING, pldm_get_sensor_reading },
	{ PLDM_MONITOR_CMD_CODE_SET_EVENT_RECEIVER, pldm_set_event_receiver },
	{ PLDM_MONITOR_CMD_CODE_PLATFORM_EVENT_MESSAGE, pldm_platform_event_message },
	{ PLDM_MONITOR_CMD_CODE_SET_STATE_EFFECTER_STATES, pldm_set_state_effecter_states },
	{ PLDM_MONITOR_CMD_CODE_GET_STATE_EFFECTER_STATES, pldm_get_state_effecter_states },
	{ PLDM_MONITOR_CMD_CODE_GET_PDR_INFO, pldm_get_pdr_info },
	{ PLDM_MONITOR_CMD_CODE_GET_PDR, pldm_get_pdr },
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

__weak void plat_pldm_set_effecter_state_host_power_control(const uint8_t *buf, uint16_t len,
							    uint8_t *resp, uint16_t *resp_len)
{
	CHECK_NULL_ARG(buf);
	CHECK_NULL_ARG(resp);
	CHECK_NULL_ARG(resp_len);

	uint8_t *completion_code_p = resp;
	*resp_len = 1;
	LOG_WRN("Not implemented in the platform code");
	*completion_code_p = PLDM_ERROR_UNSUPPORTED_PLDM_CMD;

	return;
}

void pldm_assign_gpio_effecter_id(int plat_effecter_id_gpio_high_byte)
{
	for (uint8_t i = 0; i <= PLDM_PLATFORM_OEM_AST1030_GPIO_PIN_NUM_MAX; i++) {
		state_effecter_table[i].effecter_id = (plat_effecter_id_gpio_high_byte | i);
	}
	return;
}

void pldm_load_state_effecter_table(uint16_t state_effecter_index)
{
	pldm_state_effecter_index = state_effecter_index;

	state_effecter_table =
		malloc(state_effecter_index * sizeof(struct pldm_state_effecter_info));
	if (state_effecter_table == NULL) {
		LOG_ERR("Failed to allocate memory");
		return;
	}
	plat_pldm_load_state_effecter_table();
	return;
}

__weak void plat_pldm_load_state_effecter_table(void)
{
	return;
}

struct pldm_state_effecter_info *find_state_effecter_info(uint16_t effecter_id)
{
	for (uint8_t i = 0; i < pldm_state_effecter_index; i++) {
		if (state_effecter_table[i].effecter_id == effecter_id)
			return &state_effecter_table[i];
	}

	return NULL;
}

uint8_t pldm_fill_addsel_req(struct pldm_set_state_effecter_states_req *req, uint16_t effecter_id,
			     uint8_t device_type, uint8_t board_info, uint8_t event_type)
{
	CHECK_NULL_ARG_WITH_RETURN(req, PLDM_ERROR);

	req->effecter_id = effecter_id;
	req->composite_effecter_count = PLDM_COMPOSITE_EFFECTER_COUNT_ADDSEL;
	req->field[0].set_request = PLDM_REQUEST_SET;
	req->field[0].effecter_state = device_type;
	req->field[1].set_request = PLDM_REQUEST_SET;
	req->field[1].effecter_state = board_info;
	req->field[2].set_request = PLDM_REQUEST_SET;
	req->field[2].effecter_state = event_type;

	return PLDM_SUCCESS;
}

uint8_t pldm_send_set_state_effecter_states_req(struct pldm_set_state_effecter_states_req *req,
						void *mctp_inst, mctp_ext_params ext_params)
{
	CHECK_NULL_ARG_WITH_RETURN(req, PLDM_ERROR);

	if ((req->composite_effecter_count < PLDM_COMPOSITE_EFFECTER_COUNT_MIN) ||
	    (req->composite_effecter_count > PLDM_COMPOSITE_EFFECTER_COUNT_MAX)) {
		LOG_ERR("Invalid composite effecter count: 0x%x", req->composite_effecter_count);
		return PLDM_ERROR;
	}

	uint8_t completion_code = PLDM_ERROR;
	uint8_t req_len = sizeof(struct pldm_set_state_effecter_states_req);
	uint8_t resp_len = sizeof(completion_code);
	pldm_msg msg = { 0 };

	memcpy(&msg.ext_params, &ext_params, sizeof(mctp_ext_params));

	msg.hdr.pldm_type = PLDM_TYPE_PLAT_MON_CTRL;
	msg.hdr.cmd = PLDM_MONITOR_CMD_CODE_SET_STATE_EFFECTER_STATES;
	msg.hdr.rq = 1;
	msg.buf = (uint8_t *)req;
	msg.len = req_len;

	uint16_t read_len = mctp_pldm_read(mctp_inst, &msg, &completion_code, resp_len);
	if ((!read_len) || (completion_code != PLDM_SUCCESS)) {
		LOG_ERR("Send event message failed, read_len (%d) comp_code (0x%x)", read_len,
			completion_code);
		return PLDM_ERROR;
	}

	return PLDM_SUCCESS;
}
