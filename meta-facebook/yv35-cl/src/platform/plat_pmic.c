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

static const uint8_t pmic_err_data_index[MAX_LEN_I3C_GET_PMIC_ERR] = { 3, 4, 6, 7, 8, 9 };
static const uint8_t pmic_err_pattern[MAX_COUNT_PMIC_ERROR_TYPE][MAX_LEN_I3C_GET_PMIC_ERR] = {
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

static uint8_t pmic_i3c_err_data_index[MAX_LEN_I3C_GET_PMIC_ERR] = { 0, 1, 3, 4, 5, 6 };

void start_monitor_pmic_error_thread()
{
	LOG_ERR("Start thread to monitor PMIC error");

	monitor_pmic_error_tid =
		k_thread_create(&monitor_pmic_error_thread, monitor_pmic_error_stack,
				K_THREAD_STACK_SIZEOF(monitor_pmic_error_stack),
				monitor_pmic_error_handler, NULL, NULL, NULL,
				CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&monitor_pmic_error_thread, "monitor_pmic_error_thread");
}

void monitor_pmic_error_handler()
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

int compare_pmic_error(uint8_t dimm_id, uint8_t *pmic_err_data, uint8_t pmic_err_data_len,
		       uint8_t read_path)
{
	uint8_t err_index = 0, reg_index = 0, data_index = 0;
	uint8_t pattern = 0;

	if (pmic_err_data == NULL) {
		return -1;
	}

	for (err_index = 0; err_index < MAX_COUNT_PMIC_ERROR_TYPE; err_index++) {
		for (reg_index = 0; reg_index < MAX_LEN_I3C_GET_PMIC_ERR; reg_index++) {
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
		if (reg_index == MAX_LEN_I3C_GET_PMIC_ERR) {
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
	sel_msg.InF_target = MCTP;
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

void read_pmic_error_when_dc_off()
{
	int ret = 0, dimm_id = 0;
	I3C_MSG i3c_msg = { 0 };

	if (k_mutex_lock(&i3c_dimm_mux_mutex, K_MSEC(I3C_DIMM_MUX_MUTEX_TIMEOUT_MS))) {
		LOG_ERR("Failed to lock I3C dimm MUX");
		return;
	}

	for (dimm_id = 0; dimm_id < MAX_COUNT_DIMM; dimm_id++) {
		if (!is_dimm_present(dimm_id)) {
			continue;
		}

		ret = switch_i3c_dimm_mux(I3C_MUX_TO_BIC, dimm_id / (MAX_COUNT_DIMM / 2));
		if (ret < 0) {
			continue;
		}

		i3c_msg.bus = I3C_BUS4;
		// I3C_CCC_RSTDAA: Reset dynamic address assignment
		// I3C_CCC_SETAASA: Set all addresses to static address
		ret = all_brocast_ccc(&i3c_msg);
		if (ret != 0) {
			LOG_ERR("Failed to brocast CCC, ret%d bus%d", ret, i3c_msg.bus);
			continue;
		}

		i3c_msg.target_addr = pmic_i3c_addr_list[dimm_id % (MAX_COUNT_DIMM / 2)];
		i3c_msg.tx_len = 1;
		i3c_msg.rx_len = MAX_LEN_I3C_GET_PMIC_ERR;
		memset(&i3c_msg.data, 0, i3c_msg.rx_len);
		i3c_msg.data[0] = PMIC_POR_ERROR_LOG_ADDR_VAL;

		ret = i3c_transfer(&i3c_msg);
		if (ret != 0) {
			LOG_ERR("Failed to read DIMM %d PMIC error via I3C, ret%d", dimm_id, ret);
			continue;
		}

		LOG_HEXDUMP_DBG(i3c_msg.data, i3c_msg.rx_len, "PMIC error data");
		// Compare error pattern, add SEL to BMC and update record
		ret = compare_pmic_error(dimm_id, &i3c_msg.data[0], i3c_msg.rx_len,
					 READ_PMIC_ERROR_VIA_I3C);
		if (ret < 0) {
			continue;
		}
	}

	if (k_mutex_unlock(&i3c_dimm_mux_mutex)) {
		LOG_ERR("Failed to unlock I3C dimm MUX");
	}
}
