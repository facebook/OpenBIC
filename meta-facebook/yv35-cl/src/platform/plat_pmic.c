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
#include "sensor.h"
#include "libipmi.h"
#include "intel_dimm.h"
#include "power_status.h"
#include "oem_1s_handler.h"
#include "libutil.h"
#include "plat_i3c.h"
#include "plat_ipmi.h"
#include "plat_sensor_table.h"
#include "plat_class.h"
#include "plat_i2c.h"
#include "plat_dimm.h"
#include "plat_mctp.h"

LOG_MODULE_REGISTER(plat_pmic);

K_THREAD_STACK_DEFINE(monitor_pmic_error_stack, MONITOR_PMIC_ERROR_STACK_SIZE);
struct k_thread monitor_pmic_error_thread;
k_tid_t monitor_pmic_error_tid;

static const uint8_t pmic_err_data_index[MAX_COUNT_PMIC_ERROR_OFFSET] = { 3, 4, 6, 7, 8, 9 };
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

static uint8_t pmic_i3c_err_data_index[MAX_COUNT_PMIC_ERROR_OFFSET] = { 0, 1, 3, 4, 5, 6, 46 };

void start_monitor_pmic_error_thread()
{
	LOG_INF("Start thread to monitor PMIC error");

	monitor_pmic_error_tid =
		k_thread_create(&monitor_pmic_error_thread, monitor_pmic_error_stack,
				K_THREAD_STACK_SIZEOF(monitor_pmic_error_stack),
				monitor_pmic_error_via_me_handler, NULL, NULL, NULL,
				CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&monitor_pmic_error_thread, "monitor_pmic_error_thread");
}

void monitor_pmic_error_via_i3c_handler()
{
	int ret = 0, dimm_id = 0;
	uint8_t error_data[MAX_LEN_I3C_GET_PMIC_ERR] = { 0 };

	// initialize PMIC error flag array
	memset(is_pmic_error_flag, false, sizeof(is_pmic_error_flag));

	while (1) {
		for (dimm_id = 0; dimm_id < MAX_COUNT_DIMM; dimm_id++) {
			if (!get_post_status()) {
				break;
			}

			if (!is_dimm_present(dimm_id)) {
				continue;
			}

			memset(&error_data, 0, sizeof(error_data));
			ret = get_pmic_error_raw_data(dimm_id, error_data);
			if (ret < 0) {
				continue;
			}

			// Compare error pattern, add SEL to BMC and update record
			ret = compare_pmic_error(dimm_id, error_data, sizeof(error_data),
						 READ_PMIC_ERROR_VIA_I3C);
			if (ret < 0) {
				continue;
			}
		}

		k_msleep(MONITOR_PMIC_ERROR_TIME_MS);
	}
}

int write_read_pmic_via_me(uint8_t dimm_id, uint8_t offset, uint8_t read_len, uint8_t write_len,
			   uint8_t *data, int *data_len)
{
	CHECK_NULL_ARG_WITH_RETURN(data, -1);
	CHECK_NULL_ARG_WITH_RETURN(data_len, -1);

	int ret = 0;
	uint8_t seq_source = 0xFF;
	memory_write_read_req pmic_req;
	ipmi_msg pmic_msg;
	ipmb_error status = IPMB_ERROR_SUCCESS;

	pmic_req.intel_id = INTEL_ID;
	ret = get_dimm_info(dimm_id, &pmic_req.smbus_identifier, &pmic_req.smbus_address);
	if (ret < 0) {
		return -1;
	}
	pmic_req.addr_size = PMIC_ADDR_SIZE;
	pmic_req.addr_value = offset;
	pmic_req.data_len = read_len;
	if (write_len == 0) {
		pmic_msg = construct_ipmi_message(seq_source, NETFN_NM_REQ, CMD_SMBUS_READ_MEMORY,
						  SELF, ME_IPMB, PMIC_READ_DATA_LEN,
						  (uint8_t *)&pmic_req);
	} else {
		memcpy(pmic_req.write_data, data, write_len);
		pmic_msg = construct_ipmi_message(seq_source, NETFN_NM_REQ, CMD_SMBUS_WRITE_MEMORY,
						  SELF, ME_IPMB, PMIC_WRITE_DATA_LEN,
						  (uint8_t *)&pmic_req);
	}

	status = ipmb_read(&pmic_msg, IPMB_inf_index_map[pmic_msg.InF_target]);
	if ((status == IPMB_ERROR_SUCCESS) && (pmic_msg.completion_code == CC_SUCCESS)) {
		*data_len = pmic_msg.data_len;
		memcpy(data, pmic_msg.data, pmic_msg.data_len);
		return 0;
	}

	LOG_ERR("Failed to write/read PMIC register, dimm id%d bus %d addr 0x%x offset 0x%x status 0x%x CC 0x%x",
		dimm_id, pmic_req.smbus_identifier, pmic_req.smbus_address, pmic_req.addr_value,
		status, pmic_msg.completion_code);
	return -1;
}

void monitor_pmic_error_via_me_handler()
{
	int ret = 0, dimm_id = 0;
	uint8_t dimm_status = SENSOR_INIT_STATUS;
	bool is_mps_unlock_region[MAX_COUNT_DIMM] = { false, false, false, false, false, false };
	uint8_t cache_data[MAX_COUNT_PMIC_ERROR_OFFSET] = { 0 };
	int cache_data_len = 0;
	uint8_t mps_pmic_vender[2] = { 0x0B, 0x2A };

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
			// If dimm isn't present, skip monitor
			dimm_status = get_dimm_status(dimm_id);
			if ((dimm_status == SENSOR_INIT_STATUS) ||
			    (dimm_status == SENSOR_NOT_PRESENT)) {
				continue;
			}

			// Read Vender
			if (!is_mps_unlock_region[dimm_id]) {
				// Check Vender ID and unlock MPS region
				memset(cache_data, 0, sizeof(cache_data));
				ret = write_read_pmic_via_me(dimm_id, PMIC_VENDER_ID_OFFSET,
							     MAX_COUNT_PMIC_VENDER_ID, 0,
							     cache_data, &cache_data_len);
				if (ret < 0) {
					LOG_ERR("Failed to check dimm vender, ret %d", ret);
					continue;
				}

				if (memcmp(&cache_data[3], mps_pmic_vender, 2) == 0) {
					cache_data[0] = 0x73;
					ret = write_read_pmic_via_me(
						dimm_id,
						PMIC_VENDOR_MEMORY_REGION_PASSWORD_UPPER_BYTE_OFFSET,
						PMIC_DATA_LEN, 1, cache_data, &cache_data_len);
					if (ret < 0) {
						LOG_ERR("Failed to check dimm vender, ret %d", ret);
						continue;
					}

					cache_data[0] = 0x94;
					ret = write_read_pmic_via_me(
						dimm_id,
						PMIC_VENDOR_MEMORY_REGION_PASSWORD_LOWER_BYTE_OFFSET,
						PMIC_DATA_LEN, 1, cache_data, &cache_data_len);
					if (ret < 0) {
						LOG_ERR("Failed to check dimm vender, ret %d", ret);
						continue;
					}

					// DIMM Vendor Region
					// 0x40:　Unlock DIMM Vendor Region
					cache_data[0] = 0x40;
					ret = write_read_pmic_via_me(
						dimm_id, PMIC_VENDOR_PASSWORD_CONTROL_OFFSET,
						PMIC_DATA_LEN, 1, cache_data, &cache_data_len);
					if (ret < 0) {
						LOG_ERR("Failed to check dimm vender, ret %d", ret);
						continue;
					}
				}

				is_mps_unlock_region[dimm_id] = true;
			}

			// Read PMIC error
			memset(cache_data, 0, sizeof(cache_data));
			ret = write_read_pmic_via_me(dimm_id, PMIC_POR_ERROR_LOG_ADDR_VAL,
						     MAX_LEN_ME_GET_PMIC_ERR, 0, cache_data,
						     &cache_data_len);
			if (ret < 0) {
				continue;
			}

			LOG_HEXDUMP_DBG(cache_data, cache_data_len, "pmic error");
			// Compare error pattern, if status change add SEL to BMC and update record
			ret = compare_pmic_error(dimm_id, cache_data, cache_data_len,
						 READ_PMIC_ERROR_VIA_ME);
			if (ret < 0) {
				continue;
			}
		}

		k_msleep(MONITOR_PMIC_ERROR_TIME_MS);
	}
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
		for (reg_index = 0; reg_index < MAX_COUNT_PMIC_ERROR_OFFSET; reg_index++) {
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

int get_dimm_info(uint8_t dimm_id, uint8_t *bus, uint8_t *addr)
{
	int ret = 0;

	CHECK_NULL_ARG_WITH_RETURN(bus, -1);
	CHECK_NULL_ARG_WITH_RETURN(addr, -1);

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

void add_pmic_error_sel(uint8_t dimm_id, uint8_t error_type)
{
	common_addsel_msg_t sel_msg;

	memset(&sel_msg, 0, sizeof(common_addsel_msg_t));
	sel_msg.InF_target = PLDM;
	sel_msg.sensor_type = IPMI_SENSOR_TYPE_PROCESSOR;
	sel_msg.sensor_number = SENSOR_NUM_PMIC_ERROR;
	sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
	sel_msg.event_data1 = dimm_id;
	sel_msg.event_data2 = error_type;
	sel_msg.event_data3 = 0x00;
	if (!mctp_add_sel_to_ipmi(&sel_msg)) {
		LOG_ERR("Fail to add PMIC error event log: dimm%d error 0x%x", dimm_id, error_type);
	}
}

int get_pmic_fault_status()
{
	uint8_t cpld_bus = 0;
	int ret = -1;
	ipmi_msg ipmi_msg = { 0 };

	// Get slot ID
	ipmi_msg.InF_source = SELF;
	ipmi_msg.InF_target = PLDM;
	ipmi_msg.netfn = NETFN_OEM_REQ;
	ipmi_msg.cmd = CMD_OEM_GET_BOARD_ID;
	ipmi_msg.data_len = 0;
	ret = pldm_send_ipmi_request(&ipmi_msg);
	if (ret < 0) {
		LOG_ERR("Failed to get slot ID, ret %d", ret);
		return ret;
	}

	// slot1 SB CPLD BUS is 4
	// slot2 SB CPLD BUS is 5
	// slot3 SB CPLD BUS is 6
	// slot4 SB CPLD BUS is 7
	cpld_bus = ipmi_msg.data[2] + 3;

	// Read SB CPLD (BMC channel) to know which PMIC happen critical error
	ipmi_msg.InF_source = SELF;
	ipmi_msg.InF_target = PLDM;
	ipmi_msg.netfn = NETFN_APP_REQ;
	ipmi_msg.cmd = CMD_APP_MASTER_WRITE_READ;
	ipmi_msg.data_len = 4;
	ipmi_msg.data[0] = (cpld_bus << 1) + 1; // bus
	ipmi_msg.data[1] = CL_CPLD_BMC_CHANNEL_ADDR; // addr
	ipmi_msg.data[2] = 1; // read len
	ipmi_msg.data[3] = PMIC_FAULT_STATUS_OFFSET; // offset
	ret = pldm_send_ipmi_request(&ipmi_msg);
	if (ret < 0) {
		LOG_ERR("Failed to get slot ID, ret %d", ret);
		return ret;
	}

	return ipmi_msg.data[0];
}

void read_pmic_error_when_dc_off()
{
	int ret = 0;
	uint8_t dimm_id = 0;
	int pmic_fault_status = 0;
	I3C_MSG i3c_msg = { 0 };
	bool is_pmic_fault[MAX_COUNT_DIMM] = { false, false, false, false, false, false };

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

	if (k_mutex_lock(&i3c_dimm_mux_mutex, K_MSEC(I3C_DIMM_MUX_MUTEX_TIMEOUT_MS))) {
		LOG_ERR("Failed to lock I3C dimm MUX");
		return;
	}

	for (dimm_id = 0; dimm_id < MAX_COUNT_DIMM; dimm_id++) {
		// If no PMIC critical error doesn't need to read
		if (!is_pmic_fault[dimm_id]) {
			continue;
		}

		// Switch I3C mux to BIC and switch DIMM mux
		ret = switch_i3c_dimm_mux(I3C_MUX_TO_BIC, dimm_id / (MAX_COUNT_DIMM / 2));
		if (ret < 0) {
			continue;
		}

		// Broadcast CCC after switch DIMM mux
		// I3C_CCC_RSTDAA: Reset dynamic address assignment
		// I3C_CCC_SETAASA: Set all addresses to static address
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
		ret = compare_pmic_error(dimm_id, i3c_msg.data, i3c_msg.rx_len,
					 READ_PMIC_ERROR_VIA_I3C);
		if (ret < 0) {
			continue;
		}
	}

	if (k_mutex_unlock(&i3c_dimm_mux_mutex)) {
		LOG_ERR("Failed to unlock I3C dimm MUX");
	}

	// Switch I3C MUX to CPU after read finish
	switch_i3c_dimm_mux(I3C_MUX_TO_CPU, DIMM_MUX_TO_DIMM_A0A1A3);
	return;
}

void clear_pmic_error()
{
	int dimm_id = 0, ret = 0;
	uint8_t dimm_status = SENSOR_INIT_STATUS;
	I3C_MSG i3c_msg = { 0 };

	if (k_mutex_lock(&i3c_dimm_mux_mutex, K_MSEC(I3C_DIMM_MUX_MUTEX_TIMEOUT_MS))) {
		LOG_ERR("Failed to lock I3C dimm MUX");
		return;
	}

	for (dimm_id = 0; dimm_id < MAX_COUNT_DIMM; dimm_id++) {
		if ((dimm_id == DIMM_ID_A0) || (dimm_id == DIMM_ID_A4)) {
			continue;
		}

		dimm_status = get_dimm_status(dimm_id);
		if ((dimm_status == SENSOR_INIT_STATUS) || (dimm_status == SENSOR_NOT_PRESENT) ||
		    (dimm_status == SENSOR_POLLING_DISABLE)) {
			continue;
		}

		ret = switch_i3c_dimm_mux(I3C_MUX_TO_BIC, dimm_id / (MAX_COUNT_DIMM / 2));
		if (ret < 0) {
			continue;
		}

		i3c_msg.bus = I3C_BUS4;
		all_brocast_ccc(&i3c_msg);

		i3c_msg.target_addr = pmic_i3c_addr_list[dimm_id % (MAX_COUNT_DIMM / 2)];
		i3c_msg.tx_len = 2;
		i3c_msg.rx_len = 1;
		memset(&i3c_msg.data, 0, 2);

		// Set R39 to clear registers R04 ~ R07
		// Host Region Codes: 0x74
		// Clear Registers R04 to R07, Erase MTP memory for R04 Register
		i3c_msg.data[0] = PMIC_VENDOR_PASSWORD_CONTROL_OFFSET;
		i3c_msg.data[1] = 0x74;
		ret = i3c_transfer(&i3c_msg);
		if (ret != 0) {
			continue;
		}

		// Set R14 to clear registers R08 ~ R0B, R33
		// R14[0]: GLOBAL_CLEAR_STATUS, Clear all status bits
		i3c_msg.data[0] = PMIC_CLEAR_STATUS_BITS4_OFFSET;
		i3c_msg.data[1] = 0x01;
		ret = i3c_transfer(&i3c_msg);
		if (ret != 0) {
			continue;
		}
	}

	if (k_mutex_unlock(&i3c_dimm_mux_mutex)) {
		LOG_ERR("Failed to unlock I3C dimm MUX");
	}

	// Switch I3C MUX to CPU after clear finish
	switch_i3c_dimm_mux(I3C_MUX_TO_CPU, DIMM_MUX_TO_DIMM_A0A1A3);

	return;
}
