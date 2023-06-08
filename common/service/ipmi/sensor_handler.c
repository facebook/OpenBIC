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

#include "sensor_handler.h"

#include "sensor.h"
#include <logging/log.h>
#include "libutil.h"

LOG_MODULE_DECLARE(ipmi);

__weak void SENSOR_GET_SENSOR_READING(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	uint8_t status = -1;
	uint8_t sensor_report_status = SENSOR_EVENT_MESSAGES_ENABLE;
	int reading = 0;
	if (msg->data_len != 1) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	if (enable_sensor_poll_thread) {
		// Set IPMI sensor status response
		sensor_report_status = SENSOR_EVENT_MESSAGES_ENABLE | SENSOR_SCANNING_ENABLE;

		//Get sensor reading from bic cache
		status = get_sensor_reading(sensor_config, sensor_config_count, msg->data[0],
					    &reading, GET_FROM_CACHE);
	} else {
		status = SENSOR_POLLING_DISABLE;
	}

	sensor_val *sval = (sensor_val *)(&reading);
	switch (status) {
	case SENSOR_READ_SUCCESS:
	case SENSOR_READ_ACUR_SUCCESS:
	case SENSOR_READ_4BYTE_ACUR_SUCCESS:
		msg->data[0] = calculate_MBR(msg->data[0],
					     (int)((sval->integer * 1000) + sval->fraction)) /
			       1000;
		msg->data[1] = sensor_report_status;
		// fix to threshold deassert status, BMC will compare with UCR/UNR itself
		msg->data[2] = SENSOR_THRESHOLD_STATUS;
		msg->data_len = 3;
		msg->completion_code = CC_SUCCESS;
		break;
	case SENSOR_NOT_ACCESSIBLE:
	case SENSOR_INIT_STATUS:
		msg->data[0] = 0;
		// notice BMC about sensor temporary in not accessible status
		msg->data[1] = (sensor_report_status | SENSOR_READING_STATE_UNAVAILABLE);
		// fix to threshold deassert status, BMC will compare with UCR/UNR itself
		msg->data[2] = SENSOR_THRESHOLD_STATUS;
		msg->data_len = 3;
		msg->completion_code = CC_SUCCESS;
		break;
	case SENSOR_POLLING_DISABLE:
		// getting sensor cache while sensor polling disable
		msg->completion_code = CC_SENSOR_NOT_PRESENT;
		break;
	case SENSOR_FAIL_TO_ACCESS:
		// transaction error
		msg->completion_code = CC_NODE_BUSY;
		break;
	case SENSOR_NOT_FOUND:
		// request sensor number not found
		msg->completion_code = CC_INVALID_DATA_FIELD;
		break;
	case SENSOR_UNSPECIFIED_ERROR:
	default:
		// unknown error
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		break;
	}
	return;
}

void IPMI_SENSOR_handler(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	switch (msg->cmd) {
	case CMD_SENSOR_GET_SENSOR_READING:
		SENSOR_GET_SENSOR_READING(msg);
		break;
	default:
		LOG_ERR("invalid sensor msg netfn: %x, cmd: %x", msg->netfn, msg->cmd);
		msg->data_len = 0;
		msg->completion_code = CC_INVALID_CMD;
		break;
	}
	return;
}
