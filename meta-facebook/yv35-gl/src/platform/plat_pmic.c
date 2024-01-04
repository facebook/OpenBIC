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
#include "pmic.h"
#include "libipmi.h"
#include "power_status.h"
#include "libutil.h"
#include "plat_i3c.h"
#include "plat_sensor_table.h"
#include "plat_dimm.h"
#include "plat_class.h"
#include "rg3mxxb12.h"
#include "p3h284x.h"

LOG_MODULE_REGISTER(plat_pmic);

K_THREAD_STACK_DEFINE(monitor_pmic_error_stack, MONITOR_PMIC_ERROR_STACK_SIZE);
struct k_thread monitor_pmic_error_thread;
k_tid_t monitor_pmic_error_tid;

static const uint8_t pmic_i3c_err_data_index[MAX_COUNT_PMIC_ERROR_OFFSET] = { 0, 1, 3, 4, 5, 6, 46 };
static const uint8_t pmic_err_pattern[MAX_COUNT_PMIC_ERROR_TYPE][MAX_COUNT_PMIC_ERROR_OFFSET] = {
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

static bool is_pmic_error_flag[MAX_COUNT_DIMM][MAX_COUNT_PMIC_ERROR_TYPE];

void start_monitor_pmic_error_thread()
{
	LOG_INF("Start thread to monitor PMIC error");

	monitor_pmic_error_tid =
		k_thread_create(&monitor_pmic_error_thread, monitor_pmic_error_stack,
				K_THREAD_STACK_SIZEOF(monitor_pmic_error_stack),
				monitor_pmic_error_via_i3c_handler, NULL, NULL, NULL,
				CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&monitor_pmic_error_thread, "monitor_pmic_error_thread");
}

void monitor_pmic_error_via_i3c_handler()
{
	int ret = 0, dimm_id;
	uint8_t error_data[MAX_LEN_I3C_GET_PMIC_ERR] = { 0 };

	// initialize PMIC error flag array
	memset(is_pmic_error_flag, false, sizeof(is_pmic_error_flag));

	while (1) {
		for (dimm_id = 0; dimm_id < MAX_COUNT_DIMM; dimm_id++) {
			if (!get_post_status()) {
				break;
			}

			if (!get_dimm_presence_status(dimm_id)) {
				continue;
			}

			memset(&error_data, 0, sizeof(error_data));
			ret = get_pmic_error_raw_data(dimm_id, error_data);
			if (ret < 0) {
				continue;
			}

			// Compare error pattern, add SEL to BMC and update record
			ret = compare_pmic_error(dimm_id, error_data, sizeof(error_data));
			if (ret != 0) {
				continue;
			}
		}

		k_msleep(MONITOR_PMIC_ERROR_TIME_MS);
	}
}

int compare_pmic_error(uint8_t dimm_id, uint8_t *pmic_err_data, uint8_t pmic_err_data_len)
{
	uint8_t err_index = 0, reg_index = 0, data_index = 0;
	uint8_t pattern = 0;

	if (pmic_err_data == NULL) {
		return -1;
	}

	for (err_index = 0; err_index < MAX_COUNT_PMIC_ERROR_TYPE; err_index++) {
		for (reg_index = 0; reg_index < MAX_COUNT_PMIC_ERROR_OFFSET; reg_index++) {
			data_index = pmic_i3c_err_data_index[reg_index];

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
		if (reg_index == MAX_COUNT_PMIC_ERROR_OFFSET) {
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
		LOG_ERR("Fail to add PMIC error event log: dimm: 0x%x error: 0x%x", dimm_id,
			error_type);
	}
}

int get_pmic_fault_status()
{
	uint8_t cpld_bus = 0;
	int ret = -1;
	ipmb_error status;
	ipmi_msg ipmi_msg = { 0 };

	// Get slot ID
	ipmi_msg.InF_source = SELF;
	ipmi_msg.InF_target = BMC_IPMB;
	ipmi_msg.netfn = NETFN_OEM_REQ;
	ipmi_msg.cmd = CMD_OEM_GET_BOARD_ID;
	ipmi_msg.data_len = 0;

	status = ipmb_read(&ipmi_msg, IPMB_inf_index_map[ipmi_msg.InF_target]);
	if (status != IPMB_ERROR_SUCCESS) {
		LOG_ERR("Failed to get slot ID status(%d)", status);
		return ret;
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
	ipmi_msg.data[1] = GL_CPLD_BMC_CHANNEL_ADDR; // addr
	ipmi_msg.data[2] = 1; // read len
	ipmi_msg.data[3] = PMIC_FAULT_STATUS_OFFSET; // offset

	status = ipmb_read(&ipmi_msg, IPMB_inf_index_map[ipmi_msg.InF_target]);
	if (status != IPMB_ERROR_SUCCESS) {
		LOG_ERR("Failed to read PMIC fault status(%d) via IPMB", status);
		return ret;
	}

	return ipmi_msg.data[0];
}

void read_pmic_error_when_dc_off()
{
	int ret = 0;
	uint8_t dimm_id;
	int pmic_fault_status = 0;
	I3C_MSG i3c_msg = { 0 };
	bool is_pmic_fault[MAX_COUNT_DIMM] = { false, false, false, false,
					       false, false, false, false };

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
	// bit 0: DIMM GH Fault
	// bit 1: DIMM EF Fault
	// bit 2: DIMM CD Fault
	// bit 3: DIMM AB Fault
	if (GETBIT(pmic_fault_status, 0)) {
		is_pmic_fault[DIMM_ID_A6] = true;
		is_pmic_fault[DIMM_ID_A7] = true;
	}
	if (GETBIT(pmic_fault_status, 1)) {
		is_pmic_fault[DIMM_ID_A4] = true;
		is_pmic_fault[DIMM_ID_A5] = true;
	}
	if (GETBIT(pmic_fault_status, 2)) {
		is_pmic_fault[DIMM_ID_A2] = true;
		is_pmic_fault[DIMM_ID_A3] = true;
	}
	if (GETBIT(pmic_fault_status, 3)) {
		is_pmic_fault[DIMM_ID_A0] = true;
		is_pmic_fault[DIMM_ID_A1] = true;
	}

	if (k_mutex_lock(&i3c_dimm_mutex, K_MSEC(I3C_DIMM_MUTEX_TIMEOUT_MS))) {
		LOG_ERR("Failed to lock I3C dimm MUX");
		return;
	}

	for (dimm_id = 0; dimm_id < MAX_COUNT_DIMM; dimm_id++) {
		// If no PMIC critical error doesn't need to read
		if (!is_pmic_fault[dimm_id]) {
			continue;
		}

		// Switch I3C mux to BIC and switch DIMM mux
		ret = switch_i3c_dimm_mux(I3C_MUX_TO_BIC);
		if (ret != 0) {
			continue;
		}

		uint8_t slave_port_setting = (dimm_id / (MAX_COUNT_DIMM / 2)) ?
						     I3C_HUB_TO_DIMMEFGH :
						     I3C_HUB_TO_DIMMABCD;

		if (!rg3mxxb12_set_slave_port(I3C_BUS4, RG3MXXB12_DEFAULT_STATIC_ADDRESS,
					      slave_port_setting)) {
			LOG_ERR("Failed to set slave port to slave port: 0x%x", slave_port_setting);
			continue;
		}

		// Broadcast CCC after switch DIMM mux
		// I3C_CCC_RSTDAA: Reset dynamic address assignment
		// I3C_CCC_SETAASA: Set all addresses to static address
		memset(&i3c_msg, 0, sizeof(i3c_msg));
		i3c_msg.bus = I3C_BUS4;
		ret = all_brocast_ccc(&i3c_msg);
		if (ret != 0) {
			LOG_ERR("Failed to brocast CCC, ret%d bus%d", ret, i3c_msg.bus);
			continue;
		}

		// Read PMIC error via I3C
		i3c_msg.target_addr = pmic_i3c_addr_list[dimm_id % (MAX_COUNT_DIMM / 2)];
		i3c_msg.tx_len = 1;
		i3c_msg.rx_len = MAX_LEN_I3C_GET_PMIC_ERR;
		memset(&i3c_msg.data, 0, MAX_LEN_I3C_GET_PMIC_ERR);
		i3c_msg.data[0] = PMIC_POR_ERROR_LOG_ADDR_VAL;

		ret = i3c_transfer(&i3c_msg);
		if (ret != 0) {
			LOG_ERR("Failed to read PMIC error via I3C");
			continue;
		}

		LOG_HEXDUMP_DBG(i3c_msg.data, i3c_msg.rx_len, "PMIC error data");
		// Compare error pattern, add SEL to BMC and update record
		ret = compare_pmic_error(dimm_id, i3c_msg.data, i3c_msg.rx_len);
		if (ret < 0) {
			continue;
		}
	}

	if (k_mutex_unlock(&i3c_dimm_mutex)) {
		LOG_ERR("Failed to unlock I3C dimm MUX");
	}

	// Switch I3C MUX to CPU after read finish
	switch_i3c_dimm_mux(I3C_MUX_TO_CPU);
	return;
}

void clear_pmic_error()
{
	int dimm_id, ret = 0;
	uint8_t dimm_status = SENSOR_INIT_STATUS;
	I3C_MSG i3c_msg = { 0 };
	uint16_t i3c_hub_type = I3C_HUB_TYPE_UNKNOWN;
	i3c_hub_type = get_i3c_hub_type();

	if (k_mutex_lock(&i3c_dimm_mutex, K_MSEC(I3C_DIMM_MUTEX_TIMEOUT_MS))) {
		LOG_ERR("Failed to lock I3C dimm MUX");
		return;
	}

	for (dimm_id = 0; dimm_id < MAX_COUNT_DIMM; dimm_id++) {
		dimm_status = get_dimm_status(dimm_id);
		if ((dimm_status == SENSOR_INIT_STATUS) || (dimm_status == SENSOR_NOT_PRESENT) ||
		    (dimm_status == SENSOR_POLLING_DISABLE)) {
			continue;
		}

		ret = switch_i3c_dimm_mux(I3C_MUX_TO_BIC);
		if (ret != 0) {
			continue;
		}

		uint8_t slave_port_setting = (dimm_id / (MAX_COUNT_DIMM / 2)) ?
						     I3C_HUB_TO_DIMMEFGH :
						     I3C_HUB_TO_DIMMABCD;

		if (i3c_hub_type == RG3M87B12_DEVICE_INFO) {
			if (!rg3mxxb12_set_slave_port(I3C_BUS4, RG3MXXB12_DEFAULT_STATIC_ADDRESS,
						      slave_port_setting)) {
				LOG_ERR("Failed to set slave port to slave port: 0x%x",
					slave_port_setting);
				continue;
			}
		} else {
			if (!p3h284x_set_slave_port(I3C_BUS4, P3H284X_DEFAULT_STATIC_ADDRESS,
						    slave_port_setting)) {
				LOG_ERR("Failed to set slave port to slave port: 0x%x",
					slave_port_setting);
				continue;
			}
		}

		// Broadcast CCC after switch DIMM mux
		// I3C_CCC_RSTDAA: Reset dynamic address assignment
		// I3C_CCC_SETAASA: Set all addresses to static address
		memset(&i3c_msg, 0, sizeof(i3c_msg));
		i3c_msg.bus = I3C_BUS4;
		ret = all_brocast_ccc(&i3c_msg);
		if (ret != 0) {
			LOG_ERR("Failed to brocast CCC, ret%d bus%d", ret, i3c_msg.bus);
			continue;
		}

		i3c_msg.target_addr = pmic_i3c_addr_list[dimm_id % (MAX_COUNT_DIMM / 2)];
		i3c_msg.tx_len = 2;
		i3c_msg.rx_len = 1;
		memset(&i3c_msg.data, 0, 2);

		// Set R39 to clear registers R04 ~ R07
		// Host Region Codes: 0x74
		// Clear Registers R04 to R07, Erase MTP memory for R04 Register
		i3c_msg.data[0] = PMIC_VENDOR_PASSWORD_CONTROL_OFFSET;
		i3c_msg.data[1] = 0x74;
		ret = i3c_controller_write(&i3c_msg);
		if (ret != 0) {
			continue;
		}

		// Set R14 to clear registers R08 ~ R0B, R33
		// R14[0]: GLOBAL_CLEAR_STATUS, Clear all status bits
		i3c_msg.data[0] = PMIC_CLEAR_STATUS_BITS4_OFFSET;
		i3c_msg.data[1] = 0x01;
		ret = i3c_controller_write(&i3c_msg);
		if (ret != 0) {
			continue;
		}
	}

	if (k_mutex_unlock(&i3c_dimm_mutex)) {
		LOG_ERR("Failed to unlock I3C dimm MUX");
	}

	// Switch I3C MUX to CPU after clear finish
	switch_i3c_dimm_mux(I3C_MUX_TO_CPU);

	return;
}
