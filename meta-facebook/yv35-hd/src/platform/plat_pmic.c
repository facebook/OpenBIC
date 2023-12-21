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

#include "plat_pmic.h"

#include <stdlib.h>
#include <logging/log.h>
#include "libipmi.h"
#include "power_status.h"
#include "oem_1s_handler.h"
#include "libutil.h"
#include "plat_sensor_table.h"
#include "plat_i2c.h"

LOG_MODULE_REGISTER(plat_pmic);

K_THREAD_STACK_DEFINE(monitor_pmic_error_stack, MONITOR_PMIC_ERROR_STACK_SIZE);
struct k_thread monitor_pmic_error_thread;
k_tid_t monitor_pmic_error_tid;

static const uint8_t PMIC_addr[MAX_COUNT_DIMM / 2] = { 0x48, 0x4A, 0x49, 0x4B };

static const uint8_t pmic_err_data_index[MAX_LEN_GET_PMIC_ERROR_INFO] = { 0, 1, 3, 4, 5, 6, 46 };
static const uint8_t pmic_err_pattern[MAX_COUNT_PMIC_ERROR_TYPE][MAX_LEN_GET_PMIC_ERROR_INFO] = {
	// R05,  R06,  R08,  R09,  R0A,  R0B,  R33
	{ 0x02, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00 }, // SWAOUT_OV
	{ 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00 }, // SWBOUT_OV
	{ 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00 }, // SWCOUT_OV
	{ 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 }, // SWDOUT_OV
	{ 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // VIN_BULK_OV
	{ 0x00, 0x00, 0x02, 0x00, 0x02, 0x00, 0x00 }, // VIN_MGMT_OV
	{ 0x02, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00 }, // SWAOUT_UV
	{ 0x02, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00 }, // SWBOUT_UV
	{ 0x02, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00 }, // SWCOUT_UV
	{ 0x02, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00 }, // SWDOUT_UV
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08 }, // VIN_BULK_UV
	{ 0x00, 0x00, 0x00, 0x10, 0x02, 0x00, 0x00 }, // VIN_MGMT_TO_VIN_BUCK_SWITCHOVER
	{ 0x00, 0x00, 0x00, 0x80, 0x02, 0x00, 0x00 }, // HIGH_TEMP_WARNING
	{ 0x00, 0x00, 0x00, 0x20, 0x02, 0x00, 0x00 }, // VOUT_1V8_PG
	{ 0x00, 0x00, 0x00, 0x0F, 0x02, 0x00, 0x00 }, // HIGH_CURRENT_WARNING
	{ 0x00, 0x00, 0x00, 0x00, 0x02, 0xF0, 0x00 }, // CURRENT_LIMIT_WARNING
	{ 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00 }, // CURRENT_TEMP_SHUTDOWN
};
static const uint8_t pmic_critical_error_index[MAX_LEN_PMIC_CRITICAL_ERROR_INDEX] = { 0, 1,  2, 3,
										      4, 6,  7, 8,
										      9, 10, 16 };

static bool is_pmic_error_flag[MAX_COUNT_DIMM][MAX_COUNT_PMIC_ERROR_TYPE];

static void add_pmic_error_sel(uint8_t dimm_id, uint8_t error_type)
{
	common_addsel_msg_t sel_msg;

	memset(&sel_msg, 0, sizeof(common_addsel_msg_t));
	sel_msg.InF_target = BMC_IPMB;
	sel_msg.sensor_type = IPMI_SENSOR_TYPE_PROCESSOR;
	sel_msg.sensor_number = SENSOR_NUM_PMIC_ERROR;
	sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
	sel_msg.event_data1 = dimm_id;
	sel_msg.event_data2 = error_type;
	sel_msg.event_data3 = 0x00;
	if (!common_add_sel_evt_record(&sel_msg)) {
		LOG_ERR("Fail to add PMIC error event log: dimm %d error 0x%x", dimm_id,
			error_type);
	}
}

static bool is_critical_error(uint8_t dimm_id)
{
	for (int i = 0; i < MAX_LEN_PMIC_CRITICAL_ERROR_INDEX; i++) {
		if (is_pmic_error_flag[dimm_id][pmic_critical_error_index[i]]) {
			return true;
		}
	}
	return false;
}

static void clear_pmic_error(uint8_t dimm_id)
{
	uint8_t retry = 5;
	I2C_MSG *msg = (I2C_MSG *)malloc(sizeof(I2C_MSG));
	if (msg == NULL) {
		LOG_ERR("Failed to allocate memory.");
		return;
	}

	/* Set R35 to clear error injection */
	memset(msg, 0, sizeof(*msg));
	msg->bus = (dimm_id < (MAX_COUNT_DIMM / 2)) ? I2C_BUS11 : I2C_BUS12;
	msg->target_addr = PMIC_addr[dimm_id % 4];
	msg->tx_len = 2;
	msg->data[0] = 0x35;
	msg->data[1] = 0x00;

	if (i2c_master_write(msg, retry)) {
		LOG_ERR("Failed to set PMIC R35 to 0x00.");
	}

	if (is_critical_error(dimm_id)) {
		uint8_t i = 0;
		for (; i < CLEAR_MTP_RETRY_MAX; i++) {
			/* Set R39 to clear MTP and R04-R07 */
			memset(msg, 0, sizeof(*msg));
			msg->bus = (dimm_id < (MAX_COUNT_DIMM / 2)) ? I2C_BUS11 : I2C_BUS12;
			msg->target_addr = PMIC_addr[dimm_id % 4];
			msg->tx_len = 2;
			msg->data[0] = 0x39;
			msg->data[1] = 0x74;

			if (i2c_master_write(msg, retry)) {
				LOG_ERR("Failed to set PMIC R39 to 0x74.");
				continue;
			}

			k_msleep(100);

			memset(msg, 0, sizeof(*msg));
			msg->bus = (dimm_id < (MAX_COUNT_DIMM / 2)) ? I2C_BUS11 : I2C_BUS12;
			msg->target_addr = PMIC_addr[dimm_id % 4];
			msg->tx_len = 1;
			msg->rx_len = 4;
			msg->data[0] = 0x04;

			if (i2c_master_read(msg, retry)) {
				LOG_ERR("Failed to read PMIC R39.");
				continue;
			}

			if (!(msg->data[0] || msg->data[1] || msg->data[2] || msg->data[3])) {
				break;
			}
		}

		if (i == CLEAR_MTP_RETRY_MAX) {
			LOG_ERR("Failed to clear MTP, retry reach max.");
		}
	}

	/* Set R14 to clear R08-R0B, R33 */
	memset(msg, 0, sizeof(*msg));
	msg->bus = (dimm_id < (MAX_COUNT_DIMM / 2)) ? I2C_BUS11 : I2C_BUS12;
	msg->target_addr = PMIC_addr[dimm_id % 4];
	msg->tx_len = 2;
	msg->data[0] = 0x14;
	msg->data[1] = 0x01;

	if (i2c_master_write(msg, retry)) {
		LOG_ERR("Failed to set PMIC R14 to 0x01.");
	}
	SAFE_FREE(msg);
	return;
}

static void compare_pmic_error(uint8_t dimm_id, uint8_t *pmic_err_data)
{
	CHECK_NULL_ARG(pmic_err_data);

	bool is_pmic_error_match = false;
	uint8_t err_index, reg_index, data_index, pattern;

	for (err_index = 0; err_index < MAX_COUNT_PMIC_ERROR_TYPE; err_index++) {
		for (reg_index = 0; reg_index < MAX_LEN_GET_PMIC_ERROR_INFO; reg_index++) {
			data_index = pmic_err_data_index[reg_index];
			pattern = pmic_err_pattern[err_index][reg_index];

			// Not match
			if ((pmic_err_data[data_index] & pattern) != pattern) {
				break;
			}

			// PMIC_ERROR_LOG(R05 bit[2:0]) must be the same
			uint8_t pattern_pmic_err_log = pattern & BIT_MASK(3);
			if ((reg_index == 0) && (pattern_pmic_err_log != 0) &&
			    (pattern_pmic_err_log != (pmic_err_data[data_index] & BIT_MASK(3)))) {
				break;
			}
		}

		// All bytes of error pattern are match
		if (reg_index == MAX_LEN_GET_PMIC_ERROR_INFO) {
			is_pmic_error_match = true;
			if (is_pmic_error_flag[dimm_id][err_index] == false) {
				add_pmic_error_sel(dimm_id, err_index);
				is_pmic_error_flag[dimm_id][err_index] = true;
			}
		} else { // One byte of error pattern is not match
			is_pmic_error_flag[dimm_id][err_index] = false;
		}
	}

	if (is_pmic_error_match) {
		clear_pmic_error(dimm_id);
	}
}

void pmic_error_check()
{
	uint8_t dimm_id, retry = 5;
	I2C_MSG *msg = (I2C_MSG *)malloc(sizeof(I2C_MSG));
	if (msg == NULL) {
		LOG_ERR("Failed to allocate memory");
		return;
	}

	for (dimm_id = 0; dimm_id < MAX_COUNT_DIMM; dimm_id++) {
		// Read PMIC error
		msg->bus = (dimm_id < (MAX_COUNT_DIMM / 2)) ? I2C_BUS11 : I2C_BUS12;
		msg->target_addr = PMIC_addr[dimm_id % 4];
		msg->tx_len = 1;
		msg->data[0] = 0x05;
		msg->rx_len = 47;

		if (i2c_master_read(msg, retry)) {
			LOG_ERR("Failed to get PMIC error, dimm ID %d bus %d addr 0x%x", dimm_id,
				msg->bus, msg->target_addr);
			continue;
		}
		// Compare error pattern, if status change add SEL to BMC and update record
		compare_pmic_error(dimm_id, msg->data);
	}
	SAFE_FREE(msg);
}

static void monitor_pmic_error_handler()
{
	// initialize PMIC error flag array
	memset(is_pmic_error_flag, false, sizeof(is_pmic_error_flag));

	while (1) {
		// Check PMIC error after post complete.
		if (get_DC_status() && get_post_status()) {
			pmic_error_check();
		}
		k_msleep(MONITOR_PMIC_ERROR_TIME_MS);
	}
}

int pal_set_pmic_error_flag(uint8_t dimm_id, uint8_t error_type)
{
	if (dimm_id >= MAX_COUNT_DIMM) {
		LOG_ERR("Invalid dimm id %d", dimm_id);
		return INVALID_DIMM_ID;
	}

	if (error_type >= MAX_COUNT_PMIC_ERROR_TYPE) {
		LOG_ERR("Invalid pmic error type 0x%x", error_type);
		return INVALID_ERROR_TYPE;
	}

	is_pmic_error_flag[dimm_id][error_type] = true;

	return SUCCESS;
}

void start_monitor_pmic_error_thread()
{
	LOG_INF("Start thread to monitor PMIC error");

	monitor_pmic_error_tid =
		k_thread_create(&monitor_pmic_error_thread, monitor_pmic_error_stack,
				K_THREAD_STACK_SIZEOF(monitor_pmic_error_stack),
				monitor_pmic_error_handler, NULL, NULL, NULL,
				CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&monitor_pmic_error_thread, "monitor_pmic_error_thread");
}

void read_pmic_error_when_dc_off()
{
	ipmi_msg msg = { 0 };
	/* Get slot ID */
	msg.data_len = 0;
	msg.InF_source = SELF;
	msg.InF_target = BMC_IPMB;
	msg.netfn = NETFN_OEM_REQ;
	msg.cmd = CMD_OEM_GET_BOARD_ID;

	ipmb_error ret = ipmb_read(&msg, IPMB_inf_index_map[msg.InF_target]);
	if (ret != IPMB_ERROR_SUCCESS) {
		LOG_ERR("Failed to get slot ID , ret %d", ret);
		return;
	}
	/* slot1 SB CPLD BUS is 4, slot3 SB CPLD BUS is 6 */
	uint8_t cpld_bus = msg.data[2] + 3;

	/* Read SB CPLD (BMC channel) to know if PMIC happen critical error */
	memset(&msg, 0, sizeof(msg));
	msg.InF_source = SELF;
	msg.InF_target = BMC_IPMB;
	msg.netfn = NETFN_APP_REQ;
	msg.cmd = CMD_APP_MASTER_WRITE_READ;
	msg.data_len = 4;
	msg.data[0] = (cpld_bus << 1) + 1;
	msg.data[1] = SYS_CPLD_BMC_CHANNEL_ADDR;
	msg.data[2] = 1;
	msg.data[3] = PMIC_FAULT_STATUS_OFFSET;
	ret = ipmb_read(&msg, IPMB_inf_index_map[msg.InF_target]);
	if (ret != IPMB_ERROR_SUCCESS) {
		LOG_ERR("Failed to read PMIC status, ret %d", ret);
		return;
	}

	uint8_t pmic_fault_status = msg.data[0];
	if ((pmic_fault_status & BIT(5))) {
		pmic_error_check();
	}
}
