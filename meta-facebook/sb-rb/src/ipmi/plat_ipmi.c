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

/* This file is generated for build code passing. */

#include "ipmi.h"
#include "app_handler.h"
#include "oem_1s_handler.h"
#include "libutil.h"
#include <logging/log.h>

LOG_MODULE_DECLARE(ipmi);

void IPMI_APP_handler(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);
	switch (msg->cmd) {
	case CMD_APP_GET_DEVICE_ID:
		APP_GET_DEVICE_ID(msg);
		break;
	case CMD_APP_MASTER_WRITE_READ:
		APP_MASTER_WRITE_READ(msg);
		break;
	default:
		LOG_ERR("Invalid APP msg netfn: %x, cmd: %x", msg->netfn, msg->cmd);
		msg->data_len = 0;
		msg->completion_code = CC_INVALID_CMD;
		break;
	}
	return;
}


void IPMI_OEM_1S_handler(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);
	switch (msg->cmd) {
	case CMD_OEM_1S_SENSOR_POLL_EN: // debug command
		LOG_DBG("Received 1S Sensor Poll Enable (Debug) command");
		OEM_1S_SENSOR_POLL_EN(msg);
		break;
	default:
		LOG_ERR("Invalid OEM message, netfn(0x%x) cmd(0x%x)", msg->netfn, msg->cmd);
		msg->data_len = 0;
		msg->completion_code = CC_INVALID_CMD;
		break;
	}
	return;
}
