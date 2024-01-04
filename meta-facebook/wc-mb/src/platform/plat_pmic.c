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
#include "libipmi.h"
#include "power_status.h"
#include "oem_1s_handler.h"
#include "libutil.h"
#include "plat_ipmi.h"
#include "plat_sensor_table.h"

LOG_MODULE_REGISTER(plat_pmic);

K_THREAD_STACK_DEFINE(monitor_pmic_error_stack, MONITOR_PMIC_ERROR_STACK_SIZE);
struct k_thread monitor_pmic_error_thread;
k_tid_t monitor_pmic_error_tid;

static const char dimm_lable[MAX_COUNT_DIMM][4] = { "A", "C", "E", "G" };

/* me-peci-read(0x2e/0x47)command response:
	cc
	d[2:0]	iana
	d[3]	R05 value
	d[4]	R06 value
	d[5]	R07 value - reserve
	d[6]	R08 value
	d[7]	R09 value
	d[8]	R0A value
	d[9]	R0B value
*/
static const uint8_t pmic_err_data_index[MAX_LEN_GET_PMIC_ERROR_INFO] = { 3, 4, 6, 7, 8, 9 };
static const uint8_t pmic_err_pattern[MAX_COUNT_PMIC_ERROR_TYPE][MAX_LEN_GET_PMIC_ERROR_INFO] = {
	// R05,  R06,  R08,  R09,  R0A,  R0B
	{ 0x42, 0x08, 0x00, 0x00, 0x00, 0x00 }, // SWAOUT_OV
	{ 0x22, 0x04, 0x00, 0x00, 0x00, 0x00 }, // SWBOUT_OV
	{ 0x12, 0x02, 0x00, 0x00, 0x00, 0x00 }, // SWCOUT_OV
	{ 0x0A, 0x01, 0x00, 0x00, 0x00, 0x00 }, // SWDOUT_OV
	{ 0x04, 0x00, 0x00, 0x00, 0x00, 0x00 }, // VIN_BULK_OV
	{ 0x00, 0x00, 0x02, 0x00, 0x02, 0x00 }, // VIN_MGMT_OV
	{ 0x42, 0x80, 0x00, 0x00, 0x00, 0x00 }, // SWAOUT_UV
	{ 0x22, 0x40, 0x00, 0x00, 0x00, 0x00 }, // SWBOUT_UV
	{ 0x12, 0x20, 0x00, 0x00, 0x00, 0x00 }, // SWCOUT_UV
	{ 0x0A, 0x10, 0x00, 0x00, 0x00, 0x00 }, // SWDOUT_UV
	{ 0x00, 0x00, 0x80, 0x00, 0x00, 0x00 }, // VIN_BULK_UV
	{ 0x00, 0x00, 0x00, 0x10, 0x02, 0x00 }, // VIN_MGMT_TO_VIN_BUCK_SWITCHOVER
	{ 0x00, 0x00, 0x00, 0x80, 0x02, 0x00 }, // HIGH_TEMP_WARNING
	{ 0x00, 0x00, 0x00, 0x20, 0x02, 0x00 }, // VOUT_1V8_PG
	{ 0x00, 0x00, 0x00, 0x0F, 0x02, 0x00 }, // HIGH_CURRENT_WARNING
	{ 0x00, 0x00, 0x00, 0x00, 0x02, 0xF0 }, // CURRENT_LIMIT_WARNING
	{ 0x00, 0x00, 0x40, 0x00, 0x00, 0x00 }, // CURRENT_TEMP_SHUTDOWN
};

static bool is_pmic_error_flag[MAX_COUNT_DIMM][MAX_COUNT_PMIC_ERROR_TYPE];

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
	int dimm_id = 0;
	uint8_t seq_source = 0xFF;
	memory_write_read_req pmic_req;
	ipmi_msg pmic_msg;

	// initialize PMIC error flag array
	memset(is_pmic_error_flag, false, sizeof(is_pmic_error_flag));

	while (1) {
		// If post isn't complete, BIC doesn't monitor PMIC error
		if (get_post_status() == false) {
			k_msleep(MONITOR_PMIC_ERROR_TIME_MS);
			continue;
		}

		for (dimm_id = 0; dimm_id < MAX_COUNT_DIMM; dimm_id++) {
			memset(&pmic_req, 0, sizeof(memory_write_read_req));

			if (get_dimm_info(dimm_id, &pmic_req.smbus_identifier,
					  &pmic_req.smbus_address))
				continue;

			pmic_req.intel_id = INTEL_ID;
			pmic_req.addr_size = PMIC_ADDR_SIZE;
			pmic_req.addr_value = PMIC_POR_ERROR_LOG_ADDR_VAL;
			pmic_req.data_len =
				MAX_LEN_GET_PMIC_ERROR_INFO; //get R05~R0B 7-1 based on spec
			pmic_msg = construct_ipmi_message(seq_source, NETFN_NM_REQ,
							  CMD_SMBUS_READ_MEMORY, SELF, ME_IPMB,
							  PMIC_READ_DATA_LEN, (uint8_t *)&pmic_req);

			// Double check post status before send IPMB to get PMIC error
			if (get_post_status() == false)
				continue;

			status = ipmb_read(&pmic_msg, IPMB_inf_index_map[pmic_msg.InF_target]);
			if (status == IPMB_ERROR_SUCCESS) {
				// Check completion code before compare error
				if (pmic_msg.completion_code != CC_SUCCESS)
					continue;

				// Compare error pattern, if status change add SEL to BMC and update record
				if (compare_pmic_error(dimm_id, pmic_msg.data))
					continue;
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
	CHECK_NULL_ARG_WITH_RETURN(bus, 1);
	CHECK_NULL_ARG_WITH_RETURN(addr, 1);

	if (dimm_id >= MAX_COUNT_DIMM) {
		LOG_ERR("Wrong dimm index 0x%x", dimm_id);
		return 1;
	}

	if (dimm_id < (MAX_COUNT_DIMM / 2))
		*bus = BUS_ID_DIMM_CHANNEL_0_TO_3;
	else
		*bus = BUS_ID_DIMM_CHANNEL_4_TO_7;

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
		LOG_ERR("Wrong dimm index 0x%x", dimm_id);
		return 1;
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

int compare_pmic_error(uint8_t dimm_id, uint8_t *pmic_err_data)
{
	CHECK_NULL_ARG_WITH_RETURN(pmic_err_data, 1);

	if (dimm_id >= MAX_COUNT_DIMM) {
		LOG_ERR("Invalid dimm id %d", dimm_id);
		return 1;
	}

	uint8_t err_index = 0, reg_index = 0, data_index = 0;
	uint8_t pattern = 0;

	for (err_index = 0; err_index < MAX_COUNT_PMIC_ERROR_TYPE; err_index++) {
		for (reg_index = 0; reg_index < MAX_LEN_GET_PMIC_ERROR_INFO; reg_index++) {
			data_index = pmic_err_data_index[reg_index];
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
	if (dimm_id >= MAX_COUNT_DIMM) {
		LOG_ERR("Invalid dimm id %d", dimm_id);
		return;
	}

	if (error_type >= MAX_COUNT_PMIC_ERROR_TYPE) {
		LOG_ERR("Invalid error type %d", error_type);
		return;
	}

	common_addsel_msg_t sel_msg;
	memset(&sel_msg, 0, sizeof(common_addsel_msg_t));
	sel_msg.InF_target = BMC_IPMB;
	sel_msg.sensor_type = IPMI_SENSOR_TYPE_PROCESSOR;
	sel_msg.sensor_number = SENSOR_NUM_PMIC_ERROR;
	sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
	sel_msg.event_data1 = dimm_id;
	sel_msg.event_data2 = error_type;
	sel_msg.event_data3 = 0xFF;
	if (!common_add_sel_evt_record(&sel_msg)) {
		LOG_ERR("Failed to add PMIC error event log: dimm%d error 0x%x", dimm_id,
			error_type);
	}
}
