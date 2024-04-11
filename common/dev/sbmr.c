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

#include "plat_def.h"
#ifdef ENABLE_SBMR

#include <zephyr.h>
#include <stdlib.h>
#include "libutil.h"
#include <logging/log.h>
#include "ipmi.h"
#include "ipmb.h"
#include "sensor.h"
#include "sbmr.h"

LOG_MODULE_REGISTER(sbmr);

#define SBMR_POSTCODE_LEN 128
#define PROCESS_POSTCODE_STACK_SIZE 2048

#define SMBR_CC_ERR 0x80
#define SMBR_GROUP_DEF_BODE_CODE 0xAE

static sbmr_boot_progress_code_t sbmr_read_buffer[SBMR_POSTCODE_LEN];
static uint16_t sbmr_read_len = 0, sbmr_read_index = 0;
static bool sbmr_proc_9byte_postcode_ok = false;

uint16_t copy_sbmr_read_buffer(uint16_t start, uint16_t length, uint8_t *buffer,
			       uint16_t buffer_len)
{
	CHECK_NULL_ARG_WITH_RETURN(buffer, 0);

	if (buffer_len < (length * SBMR_POSTCODE_SIZE))
		return 0;

	uint16_t current_index, i = 0;
	uint16_t current_read_len = sbmr_read_len;
	uint16_t current_read_index = sbmr_read_index;
	if (start < current_read_index) {
		current_index = current_read_index - start - 1;
	} else {
		current_index = current_read_index + SBMR_POSTCODE_LEN - start - 1;
	}

	for (; (i < length) && ((i + start) < current_read_len); i++) {
		memcpy(&buffer[SBMR_POSTCODE_SIZE * i], (uint8_t *)&sbmr_read_buffer[current_index],
		       SBMR_POSTCODE_SIZE);

		if (current_index == 0) {
			current_index = SBMR_POSTCODE_LEN - 1;
		} else {
			current_index--;
		}
	}
	return SBMR_POSTCODE_SIZE * i;
}

void sbmr_postcode_insert(sbmr_boot_progress_code_t boot_progress_code)
{
	sbmr_proc_9byte_postcode_ok = true;

	LOG_INF("* [POSTCODE] inst: %02d status:0x%04x code:0x%04x", boot_progress_code.inst,
		boot_progress_code.status_code, boot_progress_code.efi_status_code);

	sbmr_read_buffer[sbmr_read_index] = boot_progress_code;

	if (sbmr_read_len < SBMR_POSTCODE_LEN)
		sbmr_read_len++;

	sbmr_read_index++;
	if (sbmr_read_index == SBMR_POSTCODE_LEN)
		sbmr_read_index = 0;
}

void reset_sbmr_postcode_buffer()
{
	sbmr_read_len = 0;
	return;
}

bool sbmr_get_9byte_postcode_ok()
{
	return sbmr_proc_9byte_postcode_ok;
}

void sbmr_reset_9byte_postcode_ok()
{
	sbmr_proc_9byte_postcode_ok = false;
}

__weak bool SBMR_SEND_BOOT_PROGRESS_CODE(ipmi_msg *msg)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, false);

	msg->completion_code = SMBR_CC_ERR;

	if (msg->data_len != sizeof(struct sbmr_cmd_send_boot_progress_code_req)) {
		LOG_ERR("Get invalid data length 0x%x -- 0x%x", msg->data_len,
			sizeof(struct sbmr_cmd_send_boot_progress_code_req));
		goto exit;
	}

	struct sbmr_cmd_send_boot_progress_code_req *req =
		(struct sbmr_cmd_send_boot_progress_code_req *)msg->data;

	if (req->group_ext_def_body != SMBR_GROUP_DEF_BODE_CODE) {
		LOG_ERR("Get invalid group_ext_def_body 0x%x", req->group_ext_def_body);
		goto exit;
	}

	sbmr_postcode_insert(req->code);

	msg->data[0] = SMBR_GROUP_DEF_BODE_CODE;
	msg->data_len = 1;

	msg->completion_code = CC_SUCCESS;

exit:
	return true;
}

__weak bool SBMR_GET_BOOT_PROGRESS_CODE(ipmi_msg *msg)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, false);

	msg->completion_code = SMBR_CC_ERR;

	if (msg->data_len != sizeof(struct sbmr_cmd_get_boot_progress_code_req)) {
		msg->data_len = 0;
		LOG_ERR("Get invalid data length");
		goto exit;
	}

	struct sbmr_cmd_get_boot_progress_code_req *req =
		(struct sbmr_cmd_get_boot_progress_code_req *)msg->data;

	if (req->group_ext_def_body != SMBR_GROUP_DEF_BODE_CODE) {
		LOG_ERR("Get invalid group_ext_def_body 0x%x", req->group_ext_def_body);
		msg->data_len = 0;
		goto exit;
	}

	msg->data[0] = SMBR_GROUP_DEF_BODE_CODE;
	msg->data_len = 1;

	if (!sbmr_read_len)
		memset(&msg->data[1], 0, sizeof(sbmr_boot_progress_code_t));
	else
		memcpy(&msg->data[1], (uint8_t *)&sbmr_read_buffer[sbmr_read_len],
		       sizeof(sbmr_boot_progress_code_t));

	msg->data_len += sizeof(sbmr_boot_progress_code_t);
	msg->completion_code = CC_SUCCESS;

exit:
	return true;
}

bool smbr_cmd_handler(ipmi_msg *msg)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, false);

	switch (msg->cmd) {
	case CMD_DCMI_SEND_BOOT_PROGRESS_CODE:
		if (SBMR_SEND_BOOT_PROGRESS_CODE(msg) == false)
			return false;
		break;

	case CMD_DCMI_GET_BOOT_PROGRESS_CODE:
		if (SBMR_GET_BOOT_PROGRESS_CODE(msg) == false)
			return false;
		break;

	default:
		return false;
	}

	msg->netfn = (msg->netfn + 1) << 2;

	return true;
}

#endif
