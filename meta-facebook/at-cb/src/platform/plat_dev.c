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
#include "util_worker.h"
#include "pex89000.h"
#include "pldm_monitor.h"
#include "plat_pldm_monitor.h"
#include "xdpe15284.h"
#include "util_pmbus.h"
#include "plat_isr.h"
#include "pldm_state_set.h"

LOG_MODULE_REGISTER(plat_dev);

#define SW_HEARTBEAT_STACK_SIZE 1024
#define SW_HEARTBEAT_DELAY_MS 2000

#define PSOC_QSPI_FW_INDEX (FREYA_FIRMWARE_VERSION_OFFSET + 2)
#define SOC_QSPI_FW_LENGTH 3
#define INVALID_FW_DATA 0xFF
#define UNEXPECTED_FW_DATA 0x00

K_THREAD_STACK_DEFINE(sw_heartbeat_thread, SW_HEARTBEAT_STACK_SIZE);
struct k_thread sw_heartbeat_thread_handler;
k_tid_t sw_heartbeat_tid;

static bool is_sw0_ready = false;
static bool is_sw1_ready = false;
static bool accl_cable_power_fault[ASIC_CARD_COUNT] = {
	false, false, false, false, false, false, false, false, false, false, false, false,
};

typedef struct _sw_error_event_cfg {
	uint8_t event_type;
	uint32_t bit_map;
} sw_error_event_cfg;

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

switch_error_check_info sw_error_check_info[] = {
	{ .is_addsel = false,
	  .is_init_work = false,
	  .device_type = PLDM_ADDSEL_DEVICE_TYPE_PEX_SWITCH_0 },
	{ .is_addsel = false,
	  .is_init_work = false,
	  .device_type = PLDM_ADDSEL_DEVICE_TYPE_PEX_SWITCH_1 },
};

void clear_freya_cache_flag(uint8_t card_id)
{
	if (card_id >= ARRAY_SIZE(accl_freya_info)) {
		LOG_ERR("Invalid card id to clear freya cache flag, card id: 0x%x", card_id);
		return;
	}

	if (accl_freya_info[card_id].is_cache_freya1_info != false) {
		plat_asic_nvme_status_event(card_id, PCIE_DEVICE_ID1,
					    PLDM_STATE_SET_OEM_DEVICE_NVME_NOT_READY);
	}

	if (accl_freya_info[card_id].is_cache_freya2_info != false) {
		plat_asic_nvme_status_event(card_id, PCIE_DEVICE_ID2,
					    PLDM_STATE_SET_OEM_DEVICE_NVME_NOT_READY);
	}

	accl_freya_info[card_id].is_cache_freya1_info = false;
	accl_freya_info[card_id].is_cache_freya2_info = false;
	memset(&accl_freya_info[card_id].freya1_fw_info, 0, FREYA_FW_VERSION_LENGTH);
	memset(&accl_freya_info[card_id].freya2_fw_info, 0, FREYA_FW_VERSION_LENGTH);

	accl_freya_info[card_id].freya1_fw_info.is_freya_ready = FREYA_NOT_READY;
	accl_freya_info[card_id].freya2_fw_info.is_freya_ready = FREYA_NOT_READY;
}

void clear_accl_cable_power_fault_flag()
{
	uint8_t index = 0;
	for (index = 0; index < ARRAY_SIZE(accl_cable_power_fault); ++index) {
		accl_cable_power_fault[index] = false;
	}
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
		fw_info->is_freya_ready = FREYA_NOT_READY;
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
		fw_info->is_freya_ready = FREYA_NOT_READY;
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
	uint8_t index = 0, pwr_fault_index = 0;
	uint8_t val = 0;
	int result = 0;

	accl_power_fault_info power_fault_info[] = {
		{ .check_bit = CPLD_ACCL_3V3_POWER_FAULT_BIT,
		  .power_fault_state = PLDM_STATE_SET_OEM_DEVICE_3V3_POWER_FAULT },
		{ .check_bit = CPLD_ACCL_12V_POWER_FAULT_BIT,
		  .power_fault_state = PLDM_STATE_SET_OEM_DEVICE_12V_POWER_FAULT },
		{ .check_bit = CPLD_ACCL_3V3_AUX_FAULT_BIT,
		  .power_fault_state = PLDM_STATE_SET_OEM_DEVICE_3V3_AUX_FAULT },
	};

	is_sw0_ready = get_acb_power_good_flag();
	is_sw1_ready = get_acb_power_good_flag();

	while (1) {
		if (get_acb_power_good_flag() == false) {
			is_sw0_ready = false;
			is_sw1_ready = false;
			k_msleep(SW_HEARTBEAT_DELAY_MS);
			continue;
		}

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

		// Check ACCL power fault
		for (index = 0; index < ASIC_CARD_COUNT; ++index) {
			if (accl_cable_power_fault[index] != true) {
				ret = is_accl_cable_power_good_fault(index);
				if (ret != false) {
					plat_accl_cable_power_good_fail_event(
						index, PLDM_STATE_SET_OEM_DEVICE_POWER_GOOD_FAULT);
					accl_cable_power_fault[index] = true;
					continue;
				}

				result = get_cpld_register(asic_card_info[index].power_fault_reg,
							   &val);
				if (ret != 0) {
					LOG_ERR("Failed to get power fault register, card id: 0x%x, reg: 0x%x",
						index, asic_card_info[index].power_fault_reg);
					continue;
				}

				for (pwr_fault_index = 0;
				     pwr_fault_index < ARRAY_SIZE(power_fault_info);
				     ++pwr_fault_index) {
					if (val & power_fault_info[pwr_fault_index].check_bit) {
						plat_accl_power_good_fail_event(
							index, power_fault_info[pwr_fault_index]
								       .power_fault_state);
						accl_cable_power_fault[index] = true;
					}
				}
			}
		}

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
		init_sw_heartbeat_thread();
	}
}

void clear_sw_error_check_flag()
{
	for (uint8_t index = 0; index < ARRAY_SIZE(sw_error_check_info); ++index) {
		sw_error_check_info[index].is_addsel = false;
	}
}

void sw_error_event_handler(struct k_work *work_item)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work_item);
	switch_error_check_info *check_info =
		CONTAINER_OF(dwork, switch_error_check_info, add_sel_work);

	uint8_t index = 0;
	uint8_t event_type = 0;
	uint8_t board_info = gpio_get(BOARD_ID0);
	sw_error_event_cfg cfg[] = {
		{ .event_type = PLDM_ADDSEL_SYSTEM_ERROR, .bit_map = PEX_SYSTEM_ERROR },
		{ .event_type = PLDM_ADDSEL_PEX_FATAL_ERROR, .bit_map = PEX_FATAL_ERROR },
		{ .event_type = PLDM_ADDSEL_POR_BISR_TIMEOUT, .bit_map = PEX_POR_BISR_TIMEOUT },
		{ .event_type = PLDM_ADDSEL_FLASH_SIGNATURE_FAIL,
		  .bit_map = PEX_ARM_FLASH_SIGNATURE_FAIL },
		{ .event_type = PLDM_ADDSEL_WATCHDOG_0_TIMEOUT_CPU_CORE_RESET,
		  .bit_map = PEX_WDT0_CPU_RESET },
		{ .event_type = PLDM_ADDSEL_WATCHDOG_0_TIMEOUT_SYSTEM_RESET,
		  .bit_map = PEX_WDT0_SYSTEM_RESET },
		{ .event_type = PLDM_ADDSEL_WATCHDOG_1_TIMEOUT_CPU_CORE_RESET,
		  .bit_map = PEX_WDT1_CPU_RESET },
		{ .event_type = PLDM_ADDSEL_WATCHDOG_1_TIMEOUT_SYSTEM_RESET,
		  .bit_map = PEX_WDT1_SYSTEM_RESET },
		{ .event_type = PLDM_ADDSEL_LOCAL_CPU_PARITY_ERROR,
		  .bit_map = PEX_LOCAL_CPU_PARITY_ERROR },
		{ .event_type = PLDM_ADDSEL_SECURE_BOOT_FAIL, .bit_map = PEX_SECURE_BOOT_FAIL },
		{ .event_type = PLDM_ADDSEL_SBR_LOAD_FAIL, .bit_map = PEX_SBR_LOAD_FAIL },
		{ .event_type = PLDM_ADDSEL_STATION_0_FATAL_ERROR,
		  .bit_map = PEX_STATION_0_FATAL_ERROR },
		{ .event_type = PLDM_ADDSEL_STATION_1_FATAL_ERROR,
		  .bit_map = PEX_STATION_1_FATAL_ERROR },
		{ .event_type = PLDM_ADDSEL_STATION_2_FATAL_ERROR,
		  .bit_map = PEX_STATION_2_FATAL_ERROR },
		{ .event_type = PLDM_ADDSEL_STATION_3_FATAL_ERROR,
		  .bit_map = PEX_STATION_3_FATAL_ERROR },
		{ .event_type = PLDM_ADDSEL_STATION_4_FATAL_ERROR,
		  .bit_map = PEX_STATION_4_FATAL_ERROR },
		{ .event_type = PLDM_ADDSEL_STATION_5_FATAL_ERROR,
		  .bit_map = PEX_STATION_5_FATAL_ERROR },
		{ .event_type = PLDM_ADDSEL_STATION_6_FATAL_ERROR,
		  .bit_map = PEX_STATION_6_FATAL_ERROR },
		{ .event_type = PLDM_ADDSEL_STATION_7_FATAL_ERROR,
		  .bit_map = PEX_STATION_7_FATAL_ERROR },
		{ .event_type = PLDM_ADDSEL_STATION_8_FATAL_ERROR,
		  .bit_map = PEX_STATION_8_FATAL_ERROR },
		{ .event_type = PLDM_ADDSEL_PSB_STATION_FATAL_ERROR,
		  .bit_map = PEX_PSB_STATION_FATAL_ERROR },
	};

	for (index = 0; index < ARRAY_SIZE(cfg); ++index) {
		if (check_info->error_status & cfg[index].bit_map) {
			event_type = PLDM_ADDSEL_ASSERT_MASK | cfg[index].event_type;
			if (plat_set_effecter_states_req(check_info->device_type, board_info,
							 event_type) != PLDM_SUCCESS) {
				LOG_ERR("Fail to addsel switch error event, device type: 0x%x, event type: 0x%x",
					check_info->device_type, cfg[index].event_type);
			}
		}
	}
}

void get_switch_error_status(uint8_t sensor_num, uint8_t bus, uint8_t addr, uint8_t index)
{
	uint8_t sw_index = sensor_num - SENSOR_NUM_TEMP_PEX_0;

	if (sw_index >= ARRAY_SIZE(sw_error_check_info)) {
		LOG_ERR("Invalid switch index: 0x%x, sensor num: 0x%x", sw_index, sensor_num);
		return;
	}

	if (sw_error_check_info[sw_index].is_addsel != true) {
		if (pex_access_engine(bus, addr, index, pex_access_ccr_system_error,
				      &sw_error_check_info[sw_index].error_status) !=
		    pex_api_success) {
			LOG_ERR("Get CCR system error status fail, sensor num: 0x%x", sensor_num);
			return;
		}

		if (sw_error_check_info[sw_index].is_init_work != true) {
			k_work_init_delayable(&(sw_error_check_info[sw_index].add_sel_work),
					      sw_error_event_handler);
			sw_error_check_info[sw_index].is_init_work = true;
		}

		k_work_schedule_for_queue(&plat_work_q,
					  &(sw_error_check_info[sw_index].add_sel_work), K_NO_WAIT);
		sw_error_check_info[sw_index].is_addsel = true;
	}
}

bool init_vr_write_protect(uint8_t bus, uint8_t addr, uint8_t default_val)
{
	int ret = 0;
	uint8_t page = 0;
	uint8_t reg_val = 0;

	xdpe15284_set_write_protect_default_val(default_val);
	ret = pmbus_read_command(bus, addr, PMBUS_PAGE, &reg_val, 1);
	if (ret != 0) {
		LOG_ERR("Get bus: 0x%x, addr: 0x%x, page fail", bus, addr);
		return false;
	}

	page = reg_val;
	if (xdpe15284_set_write_protect(bus, addr, XDPE15284_ENABLE_WRITE_PROTECT) != true) {
		LOG_ERR("Initialize page: 0x%x write protect to 0x%x fail", page, default_val);
		return false;
	}

	page = (page == PMBUS_PAGE_0 ? PMBUS_PAGE_1 : PMBUS_PAGE_0);
	ret = pmbus_set_page(bus, addr, page);
	if (ret != 0) {
		LOG_ERR("Set bus: 0x%x, addr: 0x%x to page: 0x%x fail", bus, addr, page);
		return false;
	}

	ret = pmbus_read_command(bus, addr, PMBUS_PAGE, &reg_val, 1);
	if (ret != 0) {
		LOG_ERR("Get bus: 0x%x, addr: 0x%x, page fail", bus, addr);
		return false;
	}

	if (reg_val != page) {
		LOG_ERR("Set page to 0x%x fail", page);
		return false;
	}

	if (xdpe15284_set_write_protect(bus, addr, XDPE15284_ENABLE_WRITE_PROTECT) != true) {
		LOG_ERR("Initialize page: 0x%x write protect to 0x%x fail", page, default_val);
		return false;
	}

	return true;
}
