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
#include "plat_isr.h"

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
	if (netfn == NETFN_STORAGE_REQ) {
		if (cmd == CMD_STORAGE_ADD_SEL) {
			return false;
		}
	}

	if (netfn == NETFN_APP_REQ) {
		if (cmd == CMD_APP_SET_SYS_INFO_PARAMS) {
			return false;
		}
	}
	// In YV4, all IPMI commands are all sent to BIC
	return true;
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

	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	msg->completion_code = CC_SUCCESS;
	msg->data_len = 1;

	uint8_t slot_id = get_slot_id();

	uint8_t blade_config = BLADE_CONFIG_UNKNOWN;
	if (get_blade_config(&blade_config) == false) {
		LOG_ERR("Failed to get the blade configuration");
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}

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

	msg->data[0] = SLOT_PRESENT + blade_config + slot_id;
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

void OEM_1S_DEBUG_GET_HW_SIGNAL(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	memcpy(&msg->data[0], &hw_event_register[0], sizeof(hw_event_register));

	//clear cache register after bmc read.
	memset(hw_event_register, 0, sizeof(hw_event_register));

	msg->data_len = sizeof(hw_event_register);
	msg->completion_code = CC_SUCCESS;
	return;
}
