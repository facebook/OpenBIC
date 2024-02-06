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

#include "libutil.h"
#include <logging/log.h>
#include "ipmi.h"
#include "plat_ipmi.h"

LOG_MODULE_REGISTER(plat_ipmi);

bool pal_request_msg_to_BIC_from_HOST(uint8_t netfn, uint8_t cmd)
{
	if (netfn == NETFN_APP_REQ) {
		if ((cmd == CMD_APP_SET_ACPI_POWER) || (cmd == CMD_APP_GET_DEVICE_GUID) ||
		    (cmd == CMD_APP_GET_BMC_GLOBAL_ENABLES) ||
		    (cmd == CMD_APP_CLEAR_MESSAGE_FLAGS) || (cmd == CMD_APP_GET_CAHNNEL_INFO) ||
		    (cmd == CMD_APP_GET_DEVICE_ID) || (cmd == CMD_APP_GET_SELFTEST_RESULTS)) {
			return true;
		}
	} else if (netfn == NETFN_DCMI_REQ) {
		if (cmd == CMD_DCMI_GET_PICMG_PROPERTIES) {
			return true;
		}
	} else {
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
