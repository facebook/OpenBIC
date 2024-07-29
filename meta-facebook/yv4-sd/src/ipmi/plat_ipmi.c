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
#include <sys/crc.h>

#include "libutil.h"
#include "ipmb.h"
#include "ipmi.h"
#include "plat_ipmi.h"
#include "plat_sys.h"
#include "plat_class.h"
#include "plat_isr.h"
#include "util_sys.h"
#include "pldm_oem.h"
#include "plat_mctp.h"
#include "pldm.h"
#include "plat_pldm.h"
#include "pldm_base.h"

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

	// BMC COLD RESET event assert
	LOG_ERR("BMC COLD RESET event assert");
	struct pldm_addsel_data event_msg = { 0 };
	event_msg.event_type = BMC_COMES_OUT_COLD_RESET;
	event_msg.assert_type = EVENT_ASSERTED;
	if (PLDM_SUCCESS != send_event_log_to_bmc(event_msg)) {
		LOG_ERR("Failed to assert BMC COLD RESET event log.");
	};

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

void OEM_1S_INFORM_BMC_TO_CONTROL_POWER(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 1) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	pldm_msg translated_msg = { 0 };
	uint8_t bmc_bus = I2C_BUS_BMC;
	uint8_t bmc_interface = pal_get_bmc_interface();
	uint8_t completion_code = CC_UNSPECIFIED_ERROR;
	translated_msg.ext_params.ep = MCTP_EID_BMC;

	switch (bmc_interface) {
	case BMC_INTERFACE_I3C:
		bmc_bus = I3C_BUS_BMC;
		translated_msg.ext_params.type = MCTP_MEDIUM_TYPE_TARGET_I3C;
		translated_msg.ext_params.i3c_ext_params.addr = I3C_STATIC_ADDR_BMC;
		break;
	case BMC_INTERFACE_I2C:
		bmc_bus = I2C_BUS_BMC;
		translated_msg.ext_params.type = MCTP_MEDIUM_TYPE_SMBUS;
		translated_msg.ext_params.smbus_ext_params.addr = I2C_ADDR_BMC;
		break;
	default:
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}

	translated_msg.hdr.pldm_type = PLDM_TYPE_OEM;
	translated_msg.hdr.cmd = PLDM_OEM_WRITE_FILE_IO;
	translated_msg.hdr.rq = 1;

	uint8_t power_option = msg->data[0];
	if (power_option >= MAX_POWER_OPTION) {
		msg->completion_code = CC_INVALID_PARAM;
		return;
	}

	struct pldm_oem_write_file_io_req *ptr = (struct pldm_oem_write_file_io_req *)malloc(
		sizeof(struct pldm_oem_write_file_io_req) +
		sizeof(uint8_t) /* Minimum requried length */);

	if (!ptr) {
		LOG_ERR("Failed to allocate memory.");
		msg->completion_code = CC_OUT_OF_SPACE;
		msg->data_len = 0;
		return;
	}

	ptr->cmd_code = POWER_CONTROL;
	ptr->data_length = POWER_CONTROL_LEN;
	ptr->messages[0] = power_option;

	translated_msg.buf = (uint8_t *)ptr;
	translated_msg.len = sizeof(struct pldm_oem_write_file_io_req) + sizeof(uint8_t);

	uint8_t resp_len = sizeof(struct pldm_oem_write_file_io_resp);
	uint8_t rbuf[resp_len];

	if (!mctp_pldm_read(find_mctp_by_bus(bmc_bus), &translated_msg, rbuf, resp_len)) {
		LOG_ERR("mctp_pldm_read fail");
		completion_code = CC_CAN_NOT_RESPOND;
		goto exit;
	}

	struct pldm_oem_write_file_io_resp *resp = (struct pldm_oem_write_file_io_resp *)rbuf;
	if (resp->completion_code != PLDM_SUCCESS) {
		LOG_ERR("Check reponse completion code fail %x", resp->completion_code);
		completion_code = resp->completion_code;
		goto exit;
	}

	completion_code = CC_SUCCESS;
exit:
	SAFE_FREE(ptr);
	msg->completion_code = completion_code;
	msg->data_len = 0;
}

void OEM_GET_HTTP_BOOT_DATA(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 3) {
		LOG_ERR("Failed to get OEM http boot data because of invalid length: 0x%x",
			msg->data_len);
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t ret = 0;
	uint8_t length = msg->data[2];
	uint16_t offset = (uint16_t)(msg->data[1] << 8) | msg->data[0];

	uint8_t *httpbootdata = (uint8_t *)malloc(sizeof(uint8_t) * length);
	if (httpbootdata == NULL) {
		LOG_ERR("Failed to allocate http boot data buffer");
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}

	ret = plat_pldm_get_http_boot_data(offset, &length, msg->data[2], httpbootdata);
	if (ret != PLDM_SUCCESS) {
		LOG_ERR("Failed to get http boot data, ret: 0x%x", ret);
		SAFE_FREE(httpbootdata);
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}

	msg->data_len = 1 + length; // 1 byte length + length bytes Data
	msg->data[0] = (uint8_t)length;
	memcpy(&msg->data[1], httpbootdata, length);
	SAFE_FREE(httpbootdata);
	msg->completion_code = CC_SUCCESS;
	return;
}

void OEM_GET_HTTP_BOOT_ATTR(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 1) {
		LOG_ERR("Failed to get OEM http boot attribute because of invalid length: 0x%x",
			msg->data_len);
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t attr = msg->data[0];
	if (attr >= GET_HTTP_BOOT_MAX) {
		LOG_ERR("Failed to get OEM http boot attribute because of invalid command: 0x%x",
			attr);
		msg->completion_code = CC_INVALID_CMD;
		return;
	}

	uint8_t ret = 0;
	struct pldm_oem_read_file_attr_info info = { 0 };

	ret = plat_pldm_get_http_boot_attr(sizeof(struct pldm_oem_read_file_attr_info),
					   (uint8_t *)&info);
	if (ret != PLDM_SUCCESS) {
		LOG_ERR("Failed to get http boot attributes, ret: 0x%x", ret);
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}

	switch (attr) {
	case GET_HTTP_BOOT_SIZE:
		msg->data_len = 2;
		msg->data[0] = info.size_lsb;
		msg->data[1] = info.size_msb;
		break;
	case GET_HTTP_BOOT_CRC32:
		msg->data_len = 4;
		memcpy(&msg->data[0], &info.crc32, sizeof(uint32_t));
		break;
	default:
		msg->completion_code = CC_INVALID_CMD;
		return;
	}

	msg->completion_code = CC_SUCCESS;
	return;
}
