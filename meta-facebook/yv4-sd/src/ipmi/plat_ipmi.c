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
#include <stdlib.h>
#include <logging/log.h>

#include "libutil.h"
#include "ipmb.h"
#include "ipmi.h"
#include "plat_ipmi.h"
#include "plat_sys.h"
#include "plat_class.h"

enum THREAD_STATUS {
	THREAD_SUCCESS = 0,
	// If k_work_schedule_for_queue() is running, return 1
	THREAD_RUNNING = 1,
};

enum SLOT_STATUS {
	SLOT_NOT_PRESENT = 0x00,
	SLOT_PRESENT = 0x80, // Bit 7 is set to 1
};

LOG_MODULE_REGISTER(plat_ipmi);

bool pal_request_msg_to_BIC_from_HOST(uint8_t netfn, uint8_t cmd)
{
	if (netfn == NETFN_APP_REQ) {
		if ((cmd == CMD_APP_SET_ACPI_POWER) || (cmd == CMD_APP_GET_DEVICE_GUID) ||
		    (cmd == CMD_APP_GET_BMC_GLOBAL_ENABLES) ||
		    (cmd == CMD_APP_CLEAR_MESSAGE_FLAGS) || (cmd == CMD_APP_GET_CAHNNEL_INFO) ||
		    (cmd == CMD_APP_GET_DEVICE_ID) || (cmd == CMD_APP_GET_SELFTEST_RESULTS) ||
		    (cmd == CMD_APP_COLD_RESET)) {
			return true;
		}
	} else if (netfn == NETFN_DCMI_REQ) {
		if (cmd == CMD_DCMI_GET_PICMG_PROPERTIES) {
			return true;
		}
	} else if (netfn == NETFN_OEM_REQ) {
		if (cmd == CMD_OEM_GET_CHASSIS_POSITION) {
			return true;
		}
	}

	else {
		return false;
	}

	return false;
}

void APP_GET_BMC_GLOBAL_ENABLES(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	msg->data[0] = ENABLE;
	msg->data_len = 1;

	msg->completion_code = CC_SUCCESS;
	return;
}

void APP_SET_ACPI_POWER(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	msg->completion_code = CC_SUCCESS;
	return;
}

void APP_CLEAR_MESSAGE_FLAGS(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	msg->completion_code = CC_SUCCESS;
	return;
}

// Reset BMC
void APP_COLD_RESET(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	switch (pal_submit_bmc_cold_reset()) {
	case -EBUSY:
		msg->completion_code = CC_NODE_BUSY;
		break;
	case -EINVAL:
		msg->completion_code = CC_INVALID_PARAM;
		break;
	case -ENODEV:
		msg->completion_code = CC_SENSOR_NOT_PRESENT;
		break;
	case THREAD_RUNNING:
	case THREAD_SUCCESS:
		msg->completion_code = CC_SUCCESS;
		break;
	default:
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		break;
	}

	msg->data_len = 0;
	return;
}

void OEM_GET_CHASSIS_POSITION(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	msg->completion_code = CC_SUCCESS;
	msg->data_len = 1;

	uint8_t slot_id = get_slot_id();

	/*   msg->data[0] format:
	 *
	 *   Slot Present (Bit 7):
	 *       0: Slot not present
	 *       1: Slot present
	 *
	 *   Reserved (Bits [6:4])
	 *
	 *   Slot ID (Bits [3:0]):
	 *       0001: Slot 1
	 *       0010: Slot 2
	 *       ...
	 *       1000: Slot 8
	 */

	msg->data[0] = SLOT_PRESENT + slot_id;
	return;
}

void APP_GET_SELFTEST_RESULTS(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	// Bios needs get self test and get system info before getting set system info
	// Using hardcode directly response
	msg->data[0] = 0x55;
	msg->data[1] = 0x00;
	msg->data_len = 2;
	msg->completion_code = CC_SUCCESS;

	return;
}
