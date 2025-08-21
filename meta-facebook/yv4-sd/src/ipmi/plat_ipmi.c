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
#include "hal_i3c.h"
#include "hal_i2c.h"
#include "power_status.h"
#include "plat_i3c.h"
#include "plat_i2c.h"
#include "plat_dimm.h"
#include "util_worker.h"
#include "plat_pldm_monitor.h"

#define EVENT_RESEND_DELAY_MS 300000 // 5 minutes delay for resend
#define MAX_RESEND_ATTEMPTS 3

// Structure to hold event resend data
struct event_resend_data {
	struct pldm_addsel_data event_msg;
	int resend_count;
	struct k_work_delayable resend_work; // Work item with delay support
} resend_data;

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
	if (netfn == NETFN_OEM_REQ) {
		if (cmd == CMD_OEM_CRASH_DUMP || cmd == CMD_OEM_POST_START ||
		    cmd == CMD_OEM_POST_END) {
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

	msg->data_len = 0;
	msg->completion_code = CC_SUCCESS;
	return;
}

void APP_CLEAR_MESSAGE_FLAGS(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	msg->data_len = 0;
	msg->completion_code = CC_SUCCESS;
	return;
}

// Work handler to resend the event
void event_resend_work_handler(struct k_work *work)
{
	struct event_resend_data *data = CONTAINER_OF(work, struct event_resend_data, resend_work);

	if (data->resend_count >= MAX_RESEND_ATTEMPTS) {
		LOG_ERR("Max resend attempts reached for event log");
		return;
	}

	data->resend_count++;

	if (PLDM_SUCCESS != send_event_log_to_bmc(data->event_msg)) {
		LOG_ERR("Failed to re-send event log, attempt %d", data->resend_count);
		// Re-schedule another attempt if retries are not complete
		k_work_reschedule_for_queue(&plat_work_q, &data->resend_work,
					    K_MSEC(EVENT_RESEND_DELAY_MS));
	} else {
		LOG_INF("Successfully re-sent event log on attempt %d", data->resend_count);
		// Successfully sent, cancel any further retries
		k_work_cancel_delayable(&data->resend_work);
	}
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

	// Initialize resend data
	memset(&resend_data, 0, sizeof(resend_data));
	resend_data.event_msg = event_msg;
	k_work_init_delayable(&resend_data.resend_work, event_resend_work_handler);

	if (PLDM_SUCCESS != send_event_log_to_bmc(event_msg)) {
		LOG_ERR("Failed to assert BMC COLD RESET event log, scheduling retry.");
		resend_data.resend_count = 0;
		k_work_reschedule_for_queue(&plat_work_q, &resend_data.resend_work,
					    K_MSEC(EVENT_RESEND_DELAY_MS));
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

void frb2_wdt_timer_action(uint8_t action, uint8_t timer)
{
	struct pldm_addsel_data msg = { 0 };
	if (timer == 0x01) // BIOS FRB2
	{
		switch (action & 0x03) {
		case NO_ACTION:
			LOG_INF("frb2 no action");
			msg.event_type = BIOS_FRB2_WDT_EXPIRE;
			msg.assert_type = EVENT_ASSERTED;
			break;
		case HARD_RESET:
			LOG_INF("frb2 power reset");
			host_power_reset();
			msg.event_type = FRB2_WDT_HARD_RST;
			msg.assert_type = EVENT_ASSERTED;
			break;
		case POWER_DOWN:
			LOG_INF("frb2 power down");
			host_power_off();
			msg.event_type = FRB2_WDT_PWR_DOWN;
			msg.assert_type = EVENT_ASSERTED;
			break;
		case POWER_CYCLE:
			LOG_INF("frb2 power cycle");
			host_power_cycle();
			msg.event_type = FRB2_WDT_PWR_CYCLE;
			msg.assert_type = EVENT_ASSERTED;
			break;
		default:
			return;
		}
	} else if (timer == 0x03) // OS Load
	{
		switch (action & 0x03) {
		case NO_ACTION:
			LOG_INF("OS Load no action");
			msg.event_type = OS_LOAD_WDT_EXPIRED;
			msg.assert_type = EVENT_ASSERTED;
			break;
		case HARD_RESET:
			LOG_INF("frb2 power reset");
			host_power_reset();
			msg.event_type = OS_LOAD_WDT_HARD_RST;
			msg.assert_type = EVENT_ASSERTED;
			break;
		case POWER_DOWN:
			LOG_INF("frb2 power down");
			host_power_off();
			msg.event_type = OS_LOAD_WDT_PWR_DOWN;
			msg.assert_type = EVENT_ASSERTED;
			break;
		case POWER_CYCLE:
			LOG_INF("frb2 power cycle");
			host_power_cycle();
			msg.event_type = OS_LOAD_WDT_PWR_CYCLE;
			msg.assert_type = EVENT_ASSERTED;
			break;
		default:
			return;
		}
	} else {
		LOG_INF("Timer (%x) action not implement", timer);
		return;
	}

	if (PLDM_SUCCESS != send_event_log_to_bmc(msg)) {
		LOG_ERR("Failed to assert FRB2/OS Load timeout action event log.");
	}
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
		msg->data[0] = info.size & 0xFF;
		msg->data[1] = (info.size >> 8) & 0xFF;
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

/*
Byte 0 - DIMM location
  00h - A
  01h - B
  02h - C
  03h - D
  04h - E
  05h - F
  06h - G
  07h - H
  08h - I
  09h - J
  0Ah - K
  0Bh - L
Byte 1: Device type
  00h - SPD
  01h - SPD NVM
  02h - PMIC
Byte 2: Read/write data length
Byte 3:4: 2byte offset
Byte 5:~ write data
*/
void OEM_1S_WRITE_READ_DIMM(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	// At least include DIMM location, device type, write/read len, offset
	if (msg->data_len < 4) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	int ret = 0;
	uint8_t dimm_id = msg->data[0];
	uint8_t device_type = msg->data[1];

	// If host is DC on but not post complete, BIC can't read DIMM information via I3C
	if (get_DC_status() == true && get_post_status() == false) {
		msg->completion_code = CC_NOT_SUPP_IN_CURR_STATE;
		return;
	}

	I2C_MSG i2c_msg = { 0 };
	memset(&i2c_msg, 0, sizeof(I2C_MSG));
	i2c_msg.bus = I2C_BUS13;
	i2c_msg.tx_len = msg->data_len - 3;
	i2c_msg.rx_len = msg->data[2];

	// Check offset byte count: SPD_NVM and SPD_CACHE have 2 bytes offset
	if (device_type == DIMM_SPD_NVM || device_type == DIMM_SPD_CACHE) {
		if (i2c_msg.tx_len < 2) {
			msg->completion_code = CC_INVALID_DATA_FIELD;
			return;
		}
	} else {
		// One byte offset
		if (i2c_msg.tx_len < 1) {
			msg->completion_code = CC_INVALID_DATA_FIELD;
			return;
		}
	}

	memcpy(&i2c_msg.data[0], &msg->data[3], i2c_msg.tx_len);
	msg->data_len = i2c_msg.rx_len;
	if (k_mutex_lock(&i3c_dimm_mutex, K_MSEC(I3C_DIMM_MUTEX_TIMEOUT_MS))) {
		LOG_ERR("Failed to lock I3C dimm mux");
		msg->completion_code = CC_NODE_BUSY;
		return;
	}
	uint8_t i3c_ctrl_mux_data = (dimm_id / (DIMM_ID_MAX / 2)) ? I3C_MUX_BIC_TO_DIMMG_TO_L :
								    I3C_MUX_BIC_TO_DIMMA_TO_F;
	ret = switch_i3c_dimm_mux(i3c_ctrl_mux_data);
	if (ret < 0) {
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		goto exit;
	}

	switch (device_type) {
	case DIMM_SPD:
	case DIMM_SPD_NVM:
		i2c_msg.target_addr = spd_i3c_addr_list[dimm_id % (DIMM_ID_MAX / 2)];
		if (device_type == DIMM_SPD_NVM) {
			ret = i2c_spd_reg_read(&i2c_msg, true);
		} else {
			ret = i2c_spd_reg_read(&i2c_msg, false);
		}

		if (ret != 0) {
			LOG_ERR("Failed to read SPD addr0x%x offset0x%x, ret%d",
				i2c_msg.target_addr, i2c_msg.data[0], ret);
			msg->completion_code = CC_UNSPECIFIED_ERROR;
		} else {
			memcpy(&msg->data[0], &i2c_msg.data, i2c_msg.rx_len);
			msg->data_len = i2c_msg.rx_len;
			msg->completion_code = CC_SUCCESS;
		}
		break;
	case DIMM_PMIC:
		i2c_msg.target_addr = pmic_i3c_addr_list[dimm_id % (DIMM_ID_MAX / 2)];
		ret = i2c_master_read(&i2c_msg, 3);
		if (ret != 0) {
			LOG_ERR("Failed to read PMIC addr0x%x offset0x%x, ret%d bus%d",
				i2c_msg.target_addr, i2c_msg.data[0], ret, i2c_msg.bus);
			msg->completion_code = CC_UNSPECIFIED_ERROR;
		} else {
			memcpy(&msg->data[0], &i2c_msg.data, i2c_msg.rx_len);
			msg->data_len = i2c_msg.rx_len;
			msg->completion_code = CC_SUCCESS;
		}
		break;
	case DIMM_SPD_CACHE: {
		if (i2c_msg.tx_len < 2) {
			msg->completion_code = CC_INVALID_DATA_FIELD;
			break;
		}

		uint16_t offset = (uint16_t)i2c_msg.data[1] << 8 | i2c_msg.data[0];
		uint8_t read_len = i2c_msg.rx_len;

		// Basic bounds
		if (offset >= SPD_RAW_LEN || (offset + read_len) > SPD_RAW_LEN) {
			msg->completion_code = CC_PARAM_OUT_OF_RANGE;
			break;
		}

		// Read from cache, no mux/I2C
		uint8_t *buf = NULL;
		bool ready = false;
		int pret = plat_get_spd_raw(dimm_id, &buf, &ready);
		if (pret == -2) {
			msg->completion_code = CC_SENSOR_NOT_PRESENT;
			break;
		}
		if (pret != 0) {
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			break;
		}
		if (!ready) {
			msg->completion_code = CC_NOT_SUPP_IN_CURR_STATE;
			break;
		}

		memcpy(&msg->data[0], &buf[offset], read_len);
		msg->data_len = read_len;
		msg->completion_code = CC_SUCCESS;
		break;
	}
	default:
		msg->completion_code = CC_INVALID_DATA_FIELD;
		break;
	}
exit:
	// Switch I3C MUX to CPU after read finish
	switch_i3c_dimm_mux(I3C_MUX_CPU_TO_DIMM);
	if (k_mutex_unlock(&i3c_dimm_mutex)) {
		LOG_ERR("Failed to lock I3C dimm MUX");
	}
}

void OEM_GET_BOOT_ORDER(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 0) {
		LOG_ERR("Failed to get boot order because of invalid length: 0x%x", msg->data_len);
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t ret = 0;
	uint8_t length = BOOT_ORDER_LENGTH;

	uint8_t *bootorder = (uint8_t *)malloc(sizeof(uint8_t) * length);
	if (bootorder == NULL) {
		LOG_ERR("Failed to allocate boot order buffer");
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}

	ret = plat_pldm_get_boot_order(BOOT_ORDER_LENGTH, bootorder);
	if (ret != PLDM_SUCCESS) {
		LOG_ERR("Failed to get boot order, ret: 0x%x", ret);
		SAFE_FREE(bootorder);
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}

	msg->data_len = length;
	memcpy(&msg->data[0], bootorder, length);
	SAFE_FREE(bootorder);
	msg->completion_code = CC_SUCCESS;
	return;
}

void OEM_SET_BOOT_ORDER(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 6) {
		LOG_ERR("Failed to set boot order because of invalid length: 0x%x", msg->data_len);
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t ret = 0;
	ret = plat_pldm_set_boot_order(msg->data);
	if (ret != PLDM_SUCCESS) {
		LOG_ERR("Failed to set boot order, ret: 0x%x", ret);
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}

	msg->completion_code = CC_SUCCESS;
	msg->data_len = 0;
	return;
}
