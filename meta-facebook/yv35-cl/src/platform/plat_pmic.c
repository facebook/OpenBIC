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

#include <stdio.h>
#include <stdlib.h>
#include <zephyr.h>
#include <logging/log.h>
#include "ipmb.h"
#include "ipmi.h"
#include "intel_dimm.h"
#include "pmic.h"
#include "sensor.h"
#include "libipmi.h"
#include "power_status.h"
#include "oem_1s_handler.h"
#include "libutil.h"
#include "plat_i3c.h"
#include "plat_ipmi.h"
#include "plat_sensor_table.h"
#include "plat_class.h"
#include "plat_i2c.h"

LOG_MODULE_REGISTER(plat_pmic);

K_THREAD_STACK_DEFINE(monitor_pmic_error_stack, MONITOR_PMIC_ERROR_STACK_SIZE);
struct k_thread monitor_pmic_error_thread;
k_tid_t monitor_pmic_error_tid;

static const char dimm_lable[MAX_COUNT_DIMM][4] = { "A0", "A2", "A3", "A4", "A6", "A7" };
static const uint8_t pmic_err_data_index[MAX_LEN_GET_PMIC_ERROR_INFO] = { 3, 4, 6, 7, 8, 9 };
static const uint8_t pmic_err_pattern[MAX_COUNT_PMIC_ERROR_TYPE][MAX_LEN_GET_PMIC_ERROR_INFO] = {
	// R05,  R06,  R08,  R09,  R0A,  R0B
	{ 0x02, 0x08, 0x00, 0x00, 0x00, 0x00 }, // SWAOUT_OV
	{ 0x02, 0x04, 0x00, 0x00, 0x00, 0x00 }, // SWBOUT_OV
	{ 0x02, 0x02, 0x00, 0x00, 0x00, 0x00 }, // SWCOUT_OV
	{ 0x02, 0x01, 0x00, 0x00, 0x00, 0x00 }, // SWDOUT_OV
	{ 0x04, 0x00, 0x00, 0x00, 0x00, 0x00 }, // VIN_BULK_OV
	{ 0x00, 0x00, 0x02, 0x00, 0x02, 0x00 }, // VIN_MGMT_OV
	{ 0x02, 0x80, 0x00, 0x00, 0x00, 0x00 }, // SWAOUT_UV
	{ 0x02, 0x40, 0x00, 0x00, 0x00, 0x00 }, // SWBOUT_UV
	{ 0x02, 0x20, 0x00, 0x00, 0x00, 0x00 }, // SWCOUT_UV
	{ 0x02, 0x10, 0x00, 0x00, 0x00, 0x00 }, // SWDOUT_UV
	{ 0x00, 0x00, 0x80, 0x00, 0x00, 0x00 }, // VIN_BULK_UV
	{ 0x00, 0x00, 0x00, 0x10, 0x02, 0x00 }, // VIN_MGMT_TO_VIN_BUCK_SWITCHOVER
	{ 0x00, 0x00, 0x00, 0x80, 0x02, 0x00 }, // HIGH_TEMP_WARNING
	{ 0x00, 0x00, 0x00, 0x20, 0x02, 0x00 }, // VOUT_1V8_PG
	{ 0x00, 0x00, 0x00, 0x0F, 0x02, 0x00 }, // HIGH_CURRENT_WARNING
	{ 0x00, 0x00, 0x00, 0x00, 0x02, 0xF0 }, // CURRENT_LIMIT_WARNING
	{ 0x00, 0x00, 0x40, 0x00, 0x00, 0x00 }, // CURRENT_TEMP_SHUTDOWN
};

static bool is_pmic_error_flag[MAX_COUNT_DIMM][MAX_COUNT_PMIC_ERROR_TYPE];

static uint8_t pmic_i3c_err_data_index[MAX_LEN_GET_PMIC_ERROR_INFO] = { 0, 1, 3, 4, 5, 6 };
static uint8_t pmic_i3c_addr_list[MAX_COUNT_DIMM / 2] = { PMIC_A0_A4_ADDR, PMIC_A2_A6_ADDR,
							  PMIC_A3_A7_ADDR };

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

void monitor_pmic_error_handler()
{
	ipmb_error status = 0;
	int ret = 0, dimm_id = 0;
	uint8_t seq_source = 0xFF;
	memory_write_read_req pmic_req;
	ipmi_msg pmic_msg;

	// initialize PMIC error flag array
	memset(is_pmic_error_flag, false, sizeof(is_pmic_error_flag));

	while (1) {
		// Check sensor poll enable
		if (get_sensor_poll_enable_flag() == false) {
			k_msleep(MONITOR_PMIC_ERROR_TIME_MS);
			continue;
		}

		// If post isn't complete, BIC doesn't monitor PMIC error
		if (get_post_status() == false) {
			k_msleep(MONITOR_PMIC_ERROR_TIME_MS);
			continue;
		}

		for (dimm_id = 0; dimm_id < MAX_COUNT_DIMM; dimm_id++) {
			// Read PMIC error
			memset(&pmic_req, 0, sizeof(memory_write_read_req));
			ret = get_dimm_info(dimm_id, &pmic_req.smbus_identifier,
					    &pmic_req.smbus_address);
			if (ret < 0) {
				continue;
			}
			pmic_req.intel_id = INTEL_ID;
			pmic_req.addr_size = PMIC_ADDR_SIZE;
			pmic_req.addr_value = PMIC_POR_ERROR_LOG_ADDR_VAL;
			pmic_req.data_len = MAX_LEN_GET_PMIC_ERROR_INFO;
			pmic_msg = construct_ipmi_message(seq_source, NETFN_NM_REQ,
							  CMD_SMBUS_READ_MEMORY, SELF, ME_IPMB,
							  PMIC_READ_DATA_LEN, (uint8_t *)&pmic_req);

			// Double check post status before send IPMB to get PMIC error
			if (get_post_status() == false) {
				continue;
			}
			status = ipmb_read(&pmic_msg, IPMB_inf_index_map[pmic_msg.InF_target]);
			if (status == IPMB_ERROR_SUCCESS) {
				// Check completion code before compare error
				if (pmic_msg.completion_code != CC_SUCCESS) {
					continue;
				}

				// Compare error pattern, if status change add SEL to BMC and update record
				ret = compare_pmic_error(dimm_id, pmic_msg.data, pmic_msg.data_len,
							 READ_PMIC_ERROR_VIA_ME);
				if (ret < 0) {
					continue;
				}

			} else {
				LOG_ERR("Failed to get PMIC error, dimm %s bus %d addr 0x%x status 0x%x",
					dimm_lable[dimm_id], pmic_req.smbus_identifier,
					pmic_req.smbus_address, status);
				continue;
			}
		}

		k_msleep(MONITOR_PMIC_ERROR_TIME_MS);
	}
}

int get_dimm_info(uint8_t dimm_id, uint8_t *bus, uint8_t *addr)
{
	int ret = 0;

	if ((bus == NULL) || (addr == NULL)) {
		return -1;
	}

	if (dimm_id >= MAX_COUNT_DIMM) {
		LOG_ERR("Wrong dimm index 0x%x", dimm_id);
		return -1;
	}

	if (dimm_id < (MAX_COUNT_DIMM / 2)) {
		*bus = BUS_ID_DIMM_CHANNEL_0_TO_3;
	} else {
		*bus = BUS_ID_DIMM_CHANNEL_4_TO_7;
	}

	switch (dimm_id % (MAX_COUNT_DIMM / 2)) {
	case 0:
		*addr = ADDR_DIMM_CHANNEL_0_4;
		break;
	case 1:
		*addr = ADDR_DIMM_CHANNEL_2_6;
		break;
	case 2:
		*addr = ADDR_DIMM_CHANNEL_3_7;
		break;
	default:
		ret = -1;
		LOG_ERR("Wrong dimm index 0x%x", dimm_id);
		break;
	}

	return ret;
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

int compare_pmic_error(uint8_t dimm_id, uint8_t *pmic_err_data, uint8_t pmic_err_data_len,
		       uint8_t read_path)
{
	uint8_t err_index = 0, reg_index = 0, data_index = 0;
	uint8_t pattern = 0;

	if (pmic_err_data == NULL) {
		return -1;
	}

	for (err_index = 0; err_index < MAX_COUNT_PMIC_ERROR_TYPE; err_index++) {
		for (reg_index = 0; reg_index < MAX_LEN_GET_PMIC_ERROR_INFO; reg_index++) {
			switch (read_path) {
			case READ_PMIC_ERROR_VIA_ME:
				data_index = pmic_err_data_index[reg_index];
				break;
			case READ_PMIC_ERROR_VIA_I3C:
				data_index = pmic_i3c_err_data_index[reg_index];
				break;
			default:
				LOG_ERR("Wrong path 0x%x to read PMIC error", read_path);
				return -1;
			}

			// Not enough data
			if (data_index >= pmic_err_data_len) {
				break;
			}

			pattern = pmic_err_pattern[err_index][reg_index];

			// Not match
			if ((pmic_err_data[data_index] & pattern) != pattern) {
				break;
			}
		}

		// All bytes of error pattern are match
		if (reg_index == MAX_LEN_GET_PMIC_ERROR_INFO) {
			if (is_pmic_error_flag[dimm_id][err_index] == false) {
				add_pmic_error_sel(dimm_id, err_index);
				is_pmic_error_flag[dimm_id][err_index] = true;
			}

			// One byte of error pattern is not match
		} else {
			is_pmic_error_flag[dimm_id][err_index] = false;
		}
	}

	return 0;
}

void add_pmic_error_sel(uint8_t dimm_id, uint8_t error_type)
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
		LOG_ERR("Fail to add PMIC error event log: dimm%d error 0x%x", dimm_id, error_type);
	}
}

int get_pmic_fault_status()
{
	uint8_t cpld_bus = 0;
	ipmb_error status = IPMB_ERROR_UNKNOWN;
	ipmi_msg ipmi_msg = { 0 };

	// Get slot ID
	ipmi_msg.InF_source = SELF;
	ipmi_msg.InF_target = BMC_IPMB;
	ipmi_msg.netfn = NETFN_OEM_REQ;
	ipmi_msg.cmd = CMD_OEM_GET_BOARD_ID;
	ipmi_msg.data_len = 0;
	status = ipmb_read(&ipmi_msg, IPMB_inf_index_map[ipmi_msg.InF_target]);
	if (status != IPMB_ERROR_SUCCESS) {
		LOG_ERR("Failed to get slot ID, status 0x%x", status);
		return -1;
	}

	// slot1 SB CPLD BUS is 4
	// slot2 SB CPLD BUS is 5
	// slot3 SB CPLD BUS is 6
	// slot4 SB CPLD BUS is 7
	cpld_bus = ipmi_msg.data[2] + 3;

	// Read SB CPLD (BMC channel) to know which PMIC happen critical error
	ipmi_msg.InF_source = SELF;
	ipmi_msg.InF_target = BMC_IPMB;
	ipmi_msg.netfn = NETFN_APP_REQ;
	ipmi_msg.cmd = CMD_APP_MASTER_WRITE_READ;
	ipmi_msg.data_len = 4;
	ipmi_msg.data[0] = (cpld_bus << 1) + 1; // bus
	ipmi_msg.data[1] = CL_CPLD_BMC_CHANNEL_ADDR; // addr
	ipmi_msg.data[2] = 1; // read len
	ipmi_msg.data[3] = PMIC_FAULT_STATUS_OFFSET; // offset
	status = ipmb_read(&ipmi_msg, IPMB_inf_index_map[ipmi_msg.InF_target]);
	if (status != IPMB_ERROR_SUCCESS) {
		LOG_ERR("Failed to read PMIC critical error from SB CPLD (BMC channel), status 0x%x",
			status);
		return -1;
	}

	return ipmi_msg.data[0];
}

int switch_i3c_dimm_mux(uint8_t i3c_mux_position, uint8_t dimm_mux_position)
{
	I2C_MSG i2c_msg = { 0 };
	int ret = 0, retry = 3;

	i2c_msg.bus = I2C_BUS1;
	i2c_msg.target_addr = CPLD_ADDR;
	i2c_msg.tx_len = 2;
	i2c_msg.rx_len = 0;
	i2c_msg.data[0] = DIMM_I3C_MUX_CONTROL_OFFSET; // CPLD_I3C_DIMM_MUX
	i2c_msg.data[1] = (dimm_mux_position << 1) | i3c_mux_position;

	ret = i2c_master_write(&i2c_msg, retry);
	if (ret != 0) {
		LOG_ERR("Failed to switch I3C MUX: 0x%x, ret=%d", i3c_mux_position, ret);
	}
	return ret;
}

void read_pmic_error_via_i3c()
{
	int ret = 0, i = 0;
	uint8_t dimm_id = 0;
	int pmic_fault_status = 0;
	I3C_MSG i3c_msg = { 0 };
	bool is_pmic_fault[MAX_COUNT_DIMM] = { false, false, false, false, false, false };

	// Attach PMIC I3C address
	i3c_msg.bus = I3C_BUS3;
	for (i = 0; i < (MAX_COUNT_DIMM / 2); i++) {
		i3c_msg.target_addr = pmic_i3c_addr_list[i];
		ret = i3c_attach(&i3c_msg);
		if (ret < 0) {
			LOG_ERR("Failed to attach addr 0x%x", i3c_msg.target_addr);
		}
	}

	// Check PMIC fault status from SB CPLD (BMC channel)
	pmic_fault_status = get_pmic_fault_status();
	if (pmic_fault_status < 0) {
		LOG_ERR("Failed to read PMIC fault status.");
		return;
	} else if (pmic_fault_status == 0) {
		// No PMIC critical error, doesn't need to read register
		return;
	}

	// Check which PMIC is error
	// DIMM PMIC Fault (1: Fault 0: Normal)
	// bit 0: DIMM A6 A7 Fault
	// bit 1: DIMM A4 A5 Fault
	// bit 2: DIMM A2 A3 Fault
	// bit 3: DIMM A0 A1 Fault
	if (GETBIT(pmic_fault_status, 0)) {
		is_pmic_fault[DIMM_ID_A6] = true;
		is_pmic_fault[DIMM_ID_A7] = true;
	}
	if (GETBIT(pmic_fault_status, 1)) {
		is_pmic_fault[DIMM_ID_A4] = true;
	}
	if (GETBIT(pmic_fault_status, 2)) {
		is_pmic_fault[DIMM_ID_A2] = true;
		is_pmic_fault[DIMM_ID_A3] = true;
	}
	if (GETBIT(pmic_fault_status, 3)) {
		is_pmic_fault[DIMM_ID_A0] = true;
	}

	for (dimm_id = 0; dimm_id < MAX_COUNT_DIMM; dimm_id++) {
		// If no PMIC critical error doesn't need to read
		if (!is_pmic_fault[dimm_id]) {
			continue;
		}

		// Switch I3C mux to BIC and switch DIMM mux
		switch (dimm_id / (MAX_COUNT_DIMM / 2)) {
		case 0:
			switch_i3c_dimm_mux(I3C_MUX_TO_BIC, DIMM_MUX_TO_DIMM_A0A1A3);
			break;
		case 1:
			switch_i3c_dimm_mux(I3C_MUX_TO_BIC, DIMM_MUX_TO_DIMM_A4A6A7);
			break;
		default:
			LOG_ERR("Invalid dimm id %d", dimm_id);
			return;
		}

		// Brocase CCC after switch DIMM mux
		ret = i3c_brocast_ccc(&i3c_msg, I3C_CCC_RSTDAA, I3C_BROADCAST_ADDR);
		if (ret < 0) {
			continue;
		}

		ret = i3c_brocast_ccc(&i3c_msg, I3C_CCC_SETAASA, I3C_BROADCAST_ADDR);
		if (ret < 0) {
			continue;
		}

		// Read PMIC error via I3C
		i3c_msg.target_addr = pmic_i3c_addr_list[dimm_id % (MAX_COUNT_DIMM / 2)];
		i3c_msg.tx_len = 1;
		i3c_msg.rx_len = MAX_LEN_I3C_GET_PMIC_ERROR_INFO;
		memset(&i3c_msg.data, 0, MAX_LEN_I3C_GET_PMIC_ERROR_INFO);
		i3c_msg.data[0] = PMIC_POR_ERROR_LOG_ADDR_VAL;

		ret = i3c_transfer(&i3c_msg);
		if (ret != 0) {
			LOG_ERR("Failed to read PMIC error via I3C");
			continue;
		}

		// Compare error pattern, add SEL to BMC and update record
		ret = compare_pmic_error(dimm_id, i3c_msg.data, i3c_msg.rx_len,
					 READ_PMIC_ERROR_VIA_I3C);
		if (ret < 0) {
			continue;
		}
	}

	// Switch I3C MUX to CPU after read finish
	switch_i3c_dimm_mux(I3C_MUX_TO_CPU, DIMM_MUX_TO_DIMM_A0A1A3);
	return;
}
