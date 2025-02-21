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
#include <logging/log.h>
#include <zephyr.h>
#include "libutil.h"
#include "pmic.h"
#include "sensor.h"
#include "power_status.h"
#include "plat_i3c.h"
#include "plat_i2c.h"
#include "plat_dimm.h"
#include "plat_mctp.h"
#include "pldm_oem.h"
#include "pldm_sensor.h"
#include "pldm_monitor.h"
#include "oem_1s_handler.h"
#include "util_worker.h"

LOG_MODULE_REGISTER(plat_pmic);

K_THREAD_STACK_DEFINE(monitor_pmic_error_stack, MONITOR_PMIC_ERROR_STACK_SIZE);

struct k_thread monitor_pmic_error_thread;
k_tid_t monitor_pmic_error_tid;
static const uint8_t pmic_err_pattern[MAX_COUNT_PMIC_ERROR_TYPE][MAX_COUNT_PMIC_ERROR_OFFSET] = {
	// R05,  R06,  R08,  R09,  R0A,  R0B,  R33
	{ 0x02, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00 }, // SWAOUT_OV
	{ 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00 }, // SWBOUT_OV
	{ 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00 }, // SWCOUT_OV
	{ 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 }, // SWDOUT_OV
	{ 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00 }, // VIN_BULK_OV
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

static bool is_pmic_error_flag[DIMM_ID_MAX][MAX_COUNT_PMIC_ERROR_TYPE];
static uint8_t pmic_i3c_err_data_index[MAX_COUNT_PMIC_ERROR_OFFSET] = { 0, 1, 3, 4, 5, 6, 46 };
static uint8_t pmic_i3c_err_offset[MAX_COUNT_PMIC_ERROR_OFFSET] = { 0x5, 0x6, 0x8, 0x9,
								    0xa, 0xb, 0x33 };
static const uint8_t pmic_critical_error_index[MAX_LEN_PMIC_CRITICAL_ERROR_INDEX] = { 0, 1,  2, 3,
										      4, 6,  7, 8,
										      9, 10, 16 };
typedef struct _pmic_event_info {
	bool is_init;
	uint8_t dimm_id;
	uint8_t error_type;
	struct k_work_delayable add_sel_work;
} pmic_event_info;

pmic_event_info pmic_event_items[] = {
	{
		.is_init = false,
		.dimm_id = DIMM_ID_A,
	},
	{
		.is_init = false,
		.dimm_id = DIMM_ID_B,
	},
	{
		.is_init = false,
		.dimm_id = DIMM_ID_C,
	},
	{
		.is_init = false,
		.dimm_id = DIMM_ID_D,
	},
	{
		.is_init = false,
		.dimm_id = DIMM_ID_E,
	},
	{
		.is_init = false,
		.dimm_id = DIMM_ID_F,
	},
	{
		.is_init = false,
		.dimm_id = DIMM_ID_G,
	},
	{
		.is_init = false,
		.dimm_id = DIMM_ID_H,
	},
	{
		.is_init = false,
		.dimm_id = DIMM_ID_I,
	},
	{
		.is_init = false,
		.dimm_id = DIMM_ID_J,
	},
	{
		.is_init = false,
		.dimm_id = DIMM_ID_K,
	},
	{
		.is_init = false,
		.dimm_id = DIMM_ID_L,
	},
};

void pmic_addsel_work_handler(struct k_work *work_item)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work_item);
	pmic_event_info *work_info = CONTAINER_OF(dwork, pmic_event_info, add_sel_work);

	struct pldm_addsel_data msg = { 0 };
	msg.event_type = DIMM_PMIC_ERROR;
	msg.assert_type = EVENT_ASSERTED;
	msg.event_data_1 = work_info->dimm_id;
	msg.event_data_2 = work_info->error_type;
	if (send_event_log_to_bmc(msg) != PLDM_SUCCESS) {
		LOG_ERR("Failed to assert PMIC error log, dimm: 0x%x, error type: 0x%x",
			work_info->dimm_id, work_info->error_type);
	};
}

void init_pmic_event_work()
{
	for (int index = 0; index < ARRAY_SIZE(pmic_event_items); ++index) {
		if (pmic_event_items[index].is_init != true) {
			k_work_init_delayable(&pmic_event_items[index].add_sel_work,
					      pmic_addsel_work_handler);
			pmic_event_items[index].is_init = true;
		}
	}
}

static pmic_event_info *find_pmic_event_work_items(uint8_t dimm_id)
{
	for (int index = 0; index < ARRAY_SIZE(pmic_event_items); ++index) {
		if (pmic_event_items[index].dimm_id == dimm_id) {
			return &pmic_event_items[index];
		}
	}

	return NULL;
}

void start_monitor_pmic_error_thread()
{
	LOG_INF("Start thread to monitor PMIC error");
	monitor_pmic_error_tid =
		k_thread_create(&monitor_pmic_error_thread, monitor_pmic_error_stack,
				K_THREAD_STACK_SIZEOF(monitor_pmic_error_stack),
				monitor_pmic_error_via_i3c_handler, NULL, NULL, NULL,
				K_PRIO_PREEMPT(1), 0, K_NO_WAIT);

	k_thread_name_set(&monitor_pmic_error_thread, "monitor_pmic_error_thread");
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

void monitor_pmic_error_via_i3c_handler()
{
	int ret = 0, dimm_id = 0;
	uint8_t error_data[MAX_LEN_I3C_GET_PMIC_ERR] = { 0 };

	// initialize PMIC error flag array
	memset(is_pmic_error_flag, false, sizeof(is_pmic_error_flag));
	while (1) {
		k_msleep(MONITOR_PMIC_ERROR_TIME_MS);

		// Check sensor poll enable
		if (get_sensor_poll_enable_flag() == false) {
			continue;
		}

		// Check which PMIC is error
		if (k_mutex_lock(&i3c_dimm_mutex, K_MSEC(I3C_DIMM_MUTEX_TIMEOUT_MS))) {
			LOG_ERR("Failed to lock I3C dimm MUX");
			continue;
		}

		for (dimm_id = 0; dimm_id < DIMM_ID_MAX; dimm_id++) {
			if (!get_post_status()) {
				break;
			}

			if (get_dimm_present(dimm_id) == DIMM_NOT_PRSNT) {
				continue;
			}

			memset(&error_data, 0, sizeof(error_data));
			ret = get_pmic_error_data_one(dimm_id, error_data, true);
			if (ret < 0) {
				LOG_ERR("Fail to get PMIC error data on monitor thread, dimm: 0x%x",
					dimm_id);
				continue;
			}

			// Compare error pattern, add SEL to BMC and update record
			ret = compare_pmic_error(dimm_id, error_data, sizeof(error_data));
			if (ret < 0) {
				continue;
			}
		}

		if (k_mutex_unlock(&i3c_dimm_mutex)) {
			LOG_ERR("Failed to unlock I3C dimm MUX");
		}
	}
}

int compare_pmic_error(uint8_t dimm_id, uint8_t *pmic_err_data, uint8_t pmic_err_data_len)
{
	CHECK_NULL_ARG_WITH_RETURN(pmic_err_data, -1);

	bool is_pmic_error_match = false;
	uint8_t err_index = 0, reg_index = 0, data_index = 0;
	uint8_t pattern = 0;

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
			is_pmic_error_match = true;
			if (is_pmic_error_flag[dimm_id][err_index] == false) {
				add_pmic_error_sel(dimm_id, err_index);
				is_pmic_error_flag[dimm_id][err_index] = true;
			}
		} else {
			// One byte of error pattern is not match
			is_pmic_error_flag[dimm_id][err_index] = false;
		}
	}

	if (is_pmic_error_match) {
		clear_pmic_error(dimm_id);
	}

	return 0;
}

void add_pmic_error_sel(uint8_t dimm_id, uint8_t error_type)
{
	pmic_event_info *event_item = find_pmic_event_work_items(dimm_id);
	if (event_item == NULL) {
		LOG_ERR("Failed to find PMIC event items, dimm id: 0x%x, error type: 0x%x", dimm_id,
			error_type);
		return;
	}

	event_item->error_type = error_type;
	k_work_schedule_for_queue(&plat_work_q, &event_item->add_sel_work, K_NO_WAIT);
}

int pal_set_pmic_error_flag(uint8_t dimm_id, uint8_t error_type)
{
	if (dimm_id >= DIMM_ID_MAX) {
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

int get_pmic_error_data_one(uint8_t dimm_id, uint8_t *buffer, uint8_t is_need_check_post_status)
{
	CHECK_NULL_ARG_WITH_RETURN(buffer, -1);
	I3C_MSG i3c_msg = { 0 };
	int ret = 0;

	if (get_dimm_present(dimm_id) == DIMM_NOT_PRSNT) {
		return -1;
	}

	uint8_t i3c_ctrl_mux_data = ((dimm_id / (DIMM_ID_MAX / 2)) ? I3C_MUX_BIC_TO_DIMMG_TO_L :
								     I3C_MUX_BIC_TO_DIMMA_TO_F);
	ret = switch_i3c_dimm_mux(i3c_ctrl_mux_data);
	if (ret < 0) {
		return ret;
	}

	if ((!get_post_status()) && is_need_check_post_status) {
		switch_i3c_dimm_mux(I3C_MUX_CPU_TO_DIMM);
		return -1;
	}

	i3c_msg.bus = I3C_BUS3;
	i3c_msg.target_addr = pmic_i3c_addr_list[dimm_id % (DIMM_ID_MAX / 2)];

	i3c_attach(&i3c_msg);
	ret = all_brocast_ccc(&i3c_msg);
	if (ret != 0) {
		LOG_ERR("Failed to brocast CCC, ret%d bus%d", ret, i3c_msg.bus);
		i3c_detach(&i3c_msg);
		return ret;
	}

	// Read one PMIC register at once
	for (int i = 0; i < MAX_COUNT_PMIC_ERROR_OFFSET; i++) {
		// Read PMIC error via I3C
		i3c_msg.tx_len = 1;
		i3c_msg.rx_len = 1;
		i3c_msg.data[0] = pmic_i3c_err_offset[i];
		ret = i3c_transfer(&i3c_msg);
		if (ret != 0) {
			LOG_ERR("Failed to read PMIC error via I3C");
			break;
		}

		buffer[pmic_i3c_err_data_index[i]] = i3c_msg.data[0];
	}

	i3c_detach(&i3c_msg);

	return ret;
}

int get_pmic_error_data(uint8_t dimm_id, uint8_t *buffer, uint8_t is_need_check_post_status)
{
	CHECK_NULL_ARG_WITH_RETURN(buffer, -1);
	int ret = 0;
	I3C_MSG i3c_msg = { 0 };

	if (get_dimm_present(dimm_id) == DIMM_NOT_PRSNT) {
		return -1;
	}

	uint8_t i3c_ctrl_mux_data = ((dimm_id / (DIMM_ID_MAX / 2)) ? I3C_MUX_BIC_TO_DIMMG_TO_L :
								     I3C_MUX_BIC_TO_DIMMA_TO_F);
	ret = switch_i3c_dimm_mux(i3c_ctrl_mux_data);
	if (ret < 0) {
		return ret;
	}

	if ((!get_post_status()) && is_need_check_post_status) {
		switch_i3c_dimm_mux(I3C_MUX_CPU_TO_DIMM);
		return -1;
	}

	i3c_msg.bus = I3C_BUS3;
	i3c_msg.target_addr = pmic_i3c_addr_list[dimm_id % (DIMM_ID_MAX / 2)];

	i3c_attach(&i3c_msg);
	ret = all_brocast_ccc(&i3c_msg);
	if (ret != 0) {
		LOG_ERR("Failed to brocast CCC, ret%d bus%d", ret, i3c_msg.bus);
		i3c_detach(&i3c_msg);
		return ret;
	}

	// Read PMIC error via I3C
	i3c_msg.tx_len = 1;
	i3c_msg.rx_len = MAX_LEN_I3C_GET_PMIC_ERR;
	i3c_msg.data[0] = PMIC_POR_ERROR_LOG_ADDR_VAL;
	ret = i3c_transfer(&i3c_msg);
	i3c_detach(&i3c_msg);

	if (ret != 0) {
		LOG_ERR("Failed to read PMIC error via I3C");
	}

	memcpy(buffer, i3c_msg.data, i3c_msg.rx_len);
	return ret;
}

void read_pmic_error_when_dc_off()
{
	int ret = 0;
	uint8_t dimm_id = 0;
	uint8_t error_data[MAX_LEN_I3C_GET_PMIC_ERR] = { 0 };

	// Check which PMIC is error
	if (k_mutex_lock(&i3c_dimm_mutex, K_MSEC(I3C_DIMM_MUTEX_TIMEOUT_MS))) {
		LOG_ERR("Failed to lock I3C dimm MUX");
		return;
	}

	for (dimm_id = 0; dimm_id < DIMM_ID_MAX; dimm_id++) {
		if (get_dimm_present(dimm_id) == DIMM_NOT_PRSNT) {
			continue;
		}

		memset(&error_data, 0, sizeof(error_data));
		ret = get_pmic_error_data_one(dimm_id, error_data, false);
		if (ret != 0) {
			LOG_ERR("Read PMIC error data fail, dimm id: 0x%x", dimm_id);
			continue;
		}

		LOG_HEXDUMP_DBG(error_data, MAX_LEN_I3C_GET_PMIC_ERR, "PMIC error data");
		// Compare error pattern, add SEL to BMC and update record
		ret = compare_pmic_error(dimm_id, error_data, MAX_LEN_I3C_GET_PMIC_ERR);
		if (ret < 0) {
			continue;
		}
	}

	if (k_mutex_unlock(&i3c_dimm_mutex)) {
		LOG_ERR("Failed to unlock I3C dimm MUX");
	}

	return;
}

void clear_pmic_error(uint8_t dimm_id)
{
	int ret = 0;
	I3C_MSG i3c_msg = { 0 };

	if (k_mutex_lock(&i3c_dimm_mutex, K_MSEC(I3C_DIMM_MUTEX_TIMEOUT_MS))) {
		LOG_ERR("Failed to lock I3C dimm MUX");
		return;
	}

	uint8_t i3c_ctrl_mux_data = (dimm_id / (DIMM_ID_MAX / 2)) ? I3C_MUX_BIC_TO_DIMMG_TO_L :
								    I3C_MUX_BIC_TO_DIMMA_TO_F;
	ret = switch_i3c_dimm_mux(i3c_ctrl_mux_data);
	if (ret < 0) {
		k_mutex_unlock(&i3c_dimm_mutex);
		LOG_ERR("Failed to switch I3C dimm MUX, dimm id: 0x%x", dimm_id);
		return;
	}

	i3c_msg.bus = I3C_BUS3;
	i3c_msg.target_addr = pmic_i3c_addr_list[dimm_id % (DIMM_ID_MAX / 2)];

	i3c_attach(&i3c_msg);
	all_brocast_ccc(&i3c_msg);
	i3c_msg.tx_len = 2;
	i3c_msg.rx_len = 1;

	memset(&i3c_msg.data, 0, 2);
	/* Set R35 to clear error injection */
	i3c_msg.data[0] = PMIC_CLEAR_ERROR_INJECTION_CFG_OFFSET;
	i3c_msg.data[1] = 0x00;
	ret = i3c_transfer(&i3c_msg);
	if (ret != 0) {
		LOG_ERR("Failed to transfer I3C clear pmic error command: 0x%x",
			PMIC_CLEAR_ERROR_INJECTION_CFG_OFFSET);
	}

	if (is_critical_error(dimm_id)) {
		uint8_t index = 0;
		for (index = 0; index < CLEAR_MTP_RETRY_MAX; index++) {
			// Set R39 to clear registers R04 ~ R07
			// Host Region Codes: 0x74
			// Clear Registers R04 to R07, Erase MTP memory for R04 Register
			i3c_msg.tx_len = 2;
			i3c_msg.rx_len = 1;
			i3c_msg.data[0] = PMIC_VENDOR_PASSWORD_CONTROL_OFFSET;
			i3c_msg.data[1] = 0x74;

			ret = i3c_transfer(&i3c_msg);
			if (ret != 0) {
				LOG_ERR("Failed to transfer I3C clear pmic error command: 0x%x",
					PMIC_VENDOR_PASSWORD_CONTROL_OFFSET);
				continue;
			}

			k_msleep(CLEAR_MTP_DELAY_MS);
			// Check R04 ~ R07 status
			i3c_msg.tx_len = 1;
			i3c_msg.rx_len = 4;
			i3c_msg.data[0] = PMIC_ERROR_STATUS_START_OFFSET;

			ret = i3c_transfer(&i3c_msg);
			if (ret != 0) {
				LOG_ERR("Failed to transfer I3C clear pmic error command: 0x%x",
					PMIC_VENDOR_PASSWORD_CONTROL_OFFSET);
				continue;
			}

			if (!(i3c_msg.data[0] || i3c_msg.data[1] || i3c_msg.data[2] ||
			      i3c_msg.data[3])) {
				break;
			}
		}

		if (index == CLEAR_MTP_RETRY_MAX) {
			LOG_ERR("Failed to clear MTP, retry reach max.");
		}
	}
	// Set R14 to clear registers R08 ~ R0B, R33
	// R14[0]: GLOBAL_CLEAR_STATUS, Clear all status bits
	i3c_msg.tx_len = 2;
	i3c_msg.rx_len = 1;
	i3c_msg.data[0] = PMIC_CLEAR_STATUS_BITS4_OFFSET;
	i3c_msg.data[1] = 0x01;

	ret = i3c_transfer(&i3c_msg);
	if (ret != 0) {
		LOG_ERR("Failed to transfer I3C clear pmic error command: 0x%x",
			PMIC_CLEAR_STATUS_BITS4_OFFSET);
	}

	i3c_detach(&i3c_msg);
	if (k_mutex_unlock(&i3c_dimm_mutex)) {
		LOG_ERR("Failed to unlock I3C dimm MUX");
	}
	return;
}
