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
#include "ipmi.h"
#include "libutil.h"
#include "plat_ipmi.h"
#include "plat_ipmb.h"
#include "expansion_board.h"
#include <logging/log.h>
#include "oem_1s_handler.h"
#include "cci.h"
#include "mctp.h"
#include "plat_mctp.h"

LOG_MODULE_REGISTER(plat_ipmi);

void OEM_1S_GET_BOARD_ID(ipmi_msg *msg)
{
	if (msg == NULL) {
		LOG_ERR("Failed due to parameter *msg is NULL");
		return;
	}

	if (msg->data_len != 0) {
		msg->data_len = 0;
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	msg->data_len = 1;
	msg->data[0] = get_board_id();
	msg->completion_code = CC_SUCCESS;
	return;
}

void OEM_1S_GET_FW_VERSION(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);
	if (msg->data_len != 1) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}
	bool ret = false;
	uint8_t component = msg->data[0];
	mctp *mctp_inst = NULL;
	mctp_ext_params ext_params = { 0 };
	uint8_t read_len = 0;
	uint8_t resp_buf[GET_FW_INFO_REVISION_LEN] = { 0 };

	switch (component) {
	case RF_COMPNT_BIC:
		msg->data[0] = BIC_FW_YEAR_MSB;
		msg->data[1] = BIC_FW_YEAR_LSB;
		msg->data[2] = BIC_FW_WEEK;
		msg->data[3] = BIC_FW_VER;
		msg->data[4] = BIC_FW_platform_0;
		msg->data[5] = BIC_FW_platform_1;
		msg->data[6] = BIC_FW_platform_2;
		msg->data_len = 7;
		msg->completion_code = CC_SUCCESS;
		break;
	case RF_COMPNT_CXL:
		if (get_mctp_info_by_eid(CXL_EID, &mctp_inst, &ext_params) == false) {
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			return;
		}
		CHECK_NULL_ARG(mctp_inst);
		ret = cci_get_chip_fw_version(mctp_inst, ext_params, resp_buf, &read_len);
		if (ret == false) {
			msg->completion_code = CC_UNSPECIFIED_ERROR;
		} else {
			memcpy(&msg->data[0], resp_buf, read_len);
			msg->data_len = read_len;
			msg->completion_code = CC_SUCCESS;
		}
		break;
	default:
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		break;
	}

	return;
}
