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
#include <drivers/sensor.h>
#include <drivers/pwm.h>
#include <logging/log.h>
#include "sensor.h"
#include "pmbus.h"
#include "libutil.h"
#include "hal_i2c.h"
#include "plat_dev.h"
#include "plat_hook.h"
#include "plat_class.h"
#include "common_i2c_mux.h"
#include "plat_sensor_table.h"
#include "nvme.h"
#include "plat_ipmi.h"

LOG_MODULE_REGISTER(plat_dev);

#define SW_HEARTBEAT_STACK_SIZE 512
#define SW_HEARTBEAT_DELAY_MS 2000
K_THREAD_STACK_DEFINE(sw_heartbeat_thread, SW_HEARTBEAT_STACK_SIZE);
struct k_thread sw_heartbeat_thread_handler;
k_tid_t sw_heartbeat_tid;

static bool is_sw0_ready = false;
static bool is_sw1_ready = false;

freya_info accl_freya_info[] = {
	[0] = { .is_cache_freya1_info = false, .is_cache_freya2_info = false },
	[1] = { .is_cache_freya1_info = false, .is_cache_freya2_info = false },
	[2] = { .is_cache_freya1_info = false, .is_cache_freya2_info = false },
	[3] = { .is_cache_freya1_info = false, .is_cache_freya2_info = false },
	[4] = { .is_cache_freya1_info = false, .is_cache_freya2_info = false },
	[5] = { .is_cache_freya1_info = false, .is_cache_freya2_info = false },
	[6] = { .is_cache_freya1_info = false, .is_cache_freya2_info = false },
	[7] = { .is_cache_freya1_info = false, .is_cache_freya2_info = false },
	[8] = { .is_cache_freya1_info = false, .is_cache_freya2_info = false },
	[9] = { .is_cache_freya1_info = false, .is_cache_freya2_info = false },
	[10] = { .is_cache_freya1_info = false, .is_cache_freya2_info = false },
	[11] = { .is_cache_freya1_info = false, .is_cache_freya2_info = false },
};

vr_fw_info cb_vr_fw_info = { .is_init = false };

void clear_freya_cache_flag(uint8_t card_id)
{
	if (card_id >= ARRAY_SIZE(accl_freya_info)) {
		LOG_ERR("Invalid card id to clear freya cache flag, card id: 0x%x", card_id);
		return;
	}

	accl_freya_info[card_id].is_cache_freya1_info = false;
	accl_freya_info[card_id].is_cache_freya2_info = false;
	memset(&accl_freya_info[card_id].freya1_fw_info, 0, FREYA_FW_VERSION_LENGTH);
	memset(&accl_freya_info[card_id].freya2_fw_info, 0, FREYA_FW_VERSION_LENGTH);

	accl_freya_info[card_id].freya1_fw_info.is_freya_ready = FREYA_NOT_READY;
	accl_freya_info[card_id].freya2_fw_info.is_freya_ready = FREYA_NOT_READY;
}

int get_freya_fw_info(uint8_t bus, uint8_t addr, freya_fw_info *fw_info)
{
	CHECK_NULL_ARG_WITH_RETURN(fw_info, -1);

	int ret = 0;
	uint8_t read_buf[FREYA_MODULE_IDENTIFIER_BLOCK_LENGTH] = { 0 };

	ret = read_nvme_info(bus, addr, FREYA_STATUS_BLOCK_OFFSET, FREYA_STATUS_BLOCK_LENGTH,
			     read_buf);
	if (ret != 0) {
		LOG_ERR("Read freya ready status fail, bus: 0x%x, addr: 0x%x", bus, addr);

		// Set freya not ready value to avoid returning freya ready value
		fw_info->is_freya_ready = FREYA_NOT_READY;
		return -1;
	}

	if ((read_buf[FREYA_READY_STATUS_OFFSET] & FREYA_READY_STATUS_BIT) != 0) {
		fw_info->is_freya_ready = FREYA_NOT_READY;
		return FREYA_NOT_READY_RET_CODE;
	}

	fw_info->is_freya_ready = FREYA_READY;

	memset(read_buf, 0, sizeof(read_buf));
	ret = read_nvme_info(bus, addr, FREYA_MODULE_IDENTIFIER_BLOCK_OFFSET,
			     FREYA_MODULE_IDENTIFIER_BLOCK_LENGTH, read_buf);
	if (ret != 0) {
		LOG_ERR("Read freya module identifier fail, bus: 0x%x, addr: 0x%x", bus, addr);
		return -1;
	}

	fw_info->is_module_identifier_support =
		((read_buf[FREYA_MODULE_IDENTIFIER_OFFSET] != IS_FREYA_MODULE_IDENTIFIER_SUPPORT) ?
			 FREYA_NOT_SUPPORT_MODULE_IDENTIFIER :
			       FREYA_SUPPORT_MODULE_IDENTIFIER);
	if (fw_info->is_module_identifier_support == FREYA_NOT_SUPPORT_MODULE_IDENTIFIER) {
		return FREYA_NOT_SUPPORT_MODULE_IDENTIFIER_RET_CODE;
	}

	fw_info->form_factor_info = read_buf[FREYA_FFI_OFFSET];

	memset(read_buf, 0, sizeof(read_buf));
	ret = read_nvme_info(bus, addr, FREYA_FIRMWARE_VERSION_BLOCK_OFFSET,
			     FREYA_FIRMWARE_VERSION_BLOCK_LENGTH, read_buf);
	if (ret != 0) {
		LOG_ERR("Read freya firmware version fail, bus: 0x%x, addr: 0x%x", bus, addr);
		return -1;
	}

	memcpy(&fw_info->major_version, &read_buf[FREYA_FIRMWARE_VERSION_OFFSET],
	       FREYA_FIRMWARE_VERSION_LENGTH);
	return 0;
}

bool get_pex_heartbeat(char *label)
{
	CHECK_NULL_ARG_WITH_RETURN(label, false);

	const struct device *heartbeat = NULL;
	struct sensor_value sensor_value;
	int ret = 0;

	heartbeat = device_get_binding(label);
	if (heartbeat == NULL) {
		LOG_ERR("%s device not found", label);
		return false;
	}

	ret = sensor_sample_fetch(heartbeat);
	if (ret < 0) {
		LOG_ERR("Failed to read %s due to sensor_sample_fetch failed, ret: %d", label, ret);
		return false;
	}

	ret = sensor_channel_get(heartbeat, SENSOR_CHAN_RPM, &sensor_value);
	if (ret < 0) {
		LOG_ERR("Failed to read %s due to sensor_channel_get failed, ret: %d", label, ret);
		return false;
	}

	if (sensor_value.val1 <= 0) {
		return false;
	}

	return true;
}

void sw_heartbeat_read()
{
	bool ret = false;

	while (1) {
		ret = get_pex_heartbeat("HB0");
		if (ret != true) {
			LOG_ERR("Fail to get switch 0 heartbeat");
		}

		is_sw0_ready = ret;

		ret = get_pex_heartbeat("HB1");
		if (ret != true) {
			LOG_ERR("Fail to get switch 1 heartbeat");
		}

		is_sw1_ready = ret;

		k_msleep(SW_HEARTBEAT_DELAY_MS);
	}
}

void init_sw_heartbeat_thread()
{
	if (sw_heartbeat_tid != NULL &&
	    (strcmp(k_thread_state_str(sw_heartbeat_tid), "dead") != 0) &&
	    (strcmp(k_thread_state_str(sw_heartbeat_tid), "unknown") != 0)) {
		return;
	}
	sw_heartbeat_tid =
		k_thread_create(&sw_heartbeat_thread_handler, sw_heartbeat_thread,
				K_THREAD_STACK_SIZEOF(sw_heartbeat_thread), sw_heartbeat_read, NULL,
				NULL, NULL, CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&sw_heartbeat_thread_handler, "sw_heartbeat_thread");
}

void abort_sw_heartbeat_thread()
{
	if (sw_heartbeat_tid != NULL &&
	    (strcmp(k_thread_state_str(sw_heartbeat_tid), "dead") != 0) &&
	    (strcmp(k_thread_state_str(sw_heartbeat_tid), "unknown") != 0)) {
		k_thread_abort(sw_heartbeat_tid);
	}
}

bool is_sw_ready(uint8_t sensor_num)
{
	switch (sensor_num) {
	case SENSOR_NUM_TEMP_PEX_0:
		return is_sw0_ready;
	case SENSOR_NUM_TEMP_PEX_1:
		return is_sw1_ready;
	default:
		LOG_ERR("Invalid sensor number to get switch heartbeat, sensor num: 0x%x",
			sensor_num);
		return false;
	}
}

void init_sw_heartbeat_work()
{
	uint8_t board_revision = get_board_revision();
	if (board_revision > EVT2_STAGE) {
		if (get_acb_power_good_flag()) {
			init_sw_heartbeat_thread();
		} else {
			abort_sw_heartbeat_thread();
			is_sw0_ready = false;
			is_sw1_ready = false;
		}
	}
}
