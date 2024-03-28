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

/* Artemis module firmware update related define */
#define HBIN_HEADER_UID 0x36148967
#define HBIN_HEADER_VER 0x03
#define HBIN_HEADER_SIZE 0x30
#define PAYLOAD_HEADER_UID1 0x1D436F81
#define PAYLOAD_HEADER_UID2 0xE2BC907E
#define PAYLOAD_HEADER_SIZE 0x10
#define PAYLOAD_HEADER_VER 0x02
#define PAYLOAD_HEADER_TYPE_BOOT1 0x01
#define PAYLOAD_HEADER_TYPE_QSPI 0x16
#define PAYLOAD_HEADER_TYPE_PSOC 0x1C

#define TARGET_SRC_DEFAULT_VAL 0x00
#define TARGET_DEST_DEFAULT_VAL 0x01
#define FW_UPDATE_COMMAND_DEFAULT_VAL 0x01
#define ERROR_CODE_REG_LEN 4
#define TRANSFER_MEM_INFO_LEN 8
#define BLOCK_DATA_COUNT 32
#define WAIT_FIRMWARE_READY_DELAY_TIMEOUT_S 5
#define WAIT_BOOT1_FIRMWARE_UPDATE_COMPLETE_TIMEOUT_S 20
#define WAIT_QSPI_FIRMWARE_UPDATE_COMPLETE_TIMEOUT_S 15
#define WAIT_PSOC_FIRMWARE_UPDATE_COMPLETE_TIMEOUT_S 60
#define FW_ABORT_UPDATE_RETURN_VAL -2
#define FW_READY_WAIT_TIMEOUT_RETURN_VAL -2
#define FW_EXEC_STATUS_RUNNING_RETURN_VAL 1

#define CHECK_IS_PROGRESS_DELAY_MS 1000
#define BOOT1_COMPLETE_BIT BIT(7)

enum ATM_FW_UPDATE_REG_OFFSET {
	SB_CMD_FW_SMBUS_ERROR_CODE = 0x82,
	SB_CMD_FW_UPDATE = 0x84,
	SB_CMD_FW_UPDATE_CONTROL = 0x85,
	SB_CMD_FW_UPDATE_EXT = 0x87,
	SB_CMD_FW_UPDATE_ERROR_CODE = 0x94,
	TRANSFER_MEM_START = 0xA4,
	LCS_STATE = 0xBF,
	TRANSFER_HEADER_PAGE = 0xC1,
	TRANSFER_DATA_PACKET = 0xC7,
	SB_PAYLOAD_EXEC_T = 0xE9,
};

enum ATM_SB_CMD_FW_UPDATE_BIT {
	SB_CMD_FW_UPDATE_IN_PROGRESS_BIT = BIT(1),
	SB_CMD_FW_UPDATE_UPDATE_COMPLETE_BIT = BIT(2),
	SB_CMD_FW_UPDATE_TRANSFER_DONE_BIT = BIT(3),
	SB_CMD_FW_UPDATE_TRANSFER_READY_BIT = BIT(6),
	SB_CMD_FW_UPDATE_INITIATE_DOWNLOAD_BIT = BIT(7),
};

typedef struct _hbin_header {
	uint32_t uid;
	uint16_t header_ver;
	uint16_t header_size;
	uint8_t fw_ver;
	uint8_t over;
	uint8_t addr_lanes;
	uint8_t data_lanes;
	uint8_t rdcmd;
	uint8_t rd_dummy;
	uint8_t div;
	uint8_t resv;
	uint32_t cert1_offset;
	uint32_t cert1_size;
	uint32_t cert2_offset;
	uint32_t cert2_size;
	uint32_t cert3_offset;
	uint32_t cert3_size;
	uint32_t payload_offset;
	uint32_t payload_size;
} hbin_header;

typedef struct _payload_header {
	uint32_t uid1;
	uint32_t uid2;
	uint16_t header_size;
	uint8_t type;
	uint8_t header_ver;
	uint8_t flags;
	uint8_t device;
	uint8_t unused2;
	uint8_t unused3;
} payload_header;

typedef struct _sb_cmd_fw_update {
	uint8_t length;
	uint8_t control_option;
	uint8_t pec;
} __attribute__((__packed__)) sb_cmd_fw_update;

typedef struct _sb_cmd_fw_update_ext {
	uint8_t length;
	uint32_t payload_size;
	uint32_t payload_addr;
	uint8_t target_src;
	uint8_t target_dest;
	uint8_t command;
	uint8_t fw_flash;
	uint8_t error_code;
	uint8_t cert1_error_code;
	uint8_t cert2_error_code;
	uint8_t cert3_error_code;
	uint8_t phantom_flash;
	uint8_t exec_fw_reset;
	uint16_t reserved;
	uint32_t state_info;
	uint8_t pec;
} __attribute__((__packed__)) sb_cmd_fw_update_ext;

typedef struct _sb_transfer_header {
	uint8_t length;
	uint16_t page;
	uint16_t reserved;
	uint8_t pec;
} __attribute__((__packed__)) sb_transfer_header;

typedef struct _sb_transferdata_packet {
	uint8_t length;
	uint8_t data[32];
	uint8_t pec;
} __attribute__((__packed__)) sb_transferdata_packet;

typedef struct _sb_payload_exec_t {
	uint8_t length;
	uint8_t exec_id;
	uint8_t exec_status;
	uint8_t exec_state;
	uint8_t exec_result;
	uint8_t forced_state_fail;
	uint8_t abort;
	uint8_t pec;
} __attribute__((__packed__)) sb_payload_exec_t;

wait_fw_update_status_info atm_wait_fw_info = { .is_init = false,
						.type = UNKNOWN_TYPE,
						.is_work_done = false,
						.status = EXEC_STATUS_DEFAULT,
						.result = EXEC_RESULT_DEFAULT };

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

int check_error_status(uint8_t bus, uint8_t addr, uint32_t *error_code, uint8_t *smbus_error_code)
{
	CHECK_NULL_ARG_WITH_RETURN(error_code, -1);
	CHECK_NULL_ARG_WITH_RETURN(smbus_error_code, -1);

	int ret = 0;
	int retry = 5;
	uint8_t offset = SB_CMD_FW_UPDATE_ERROR_CODE;
	I2C_MSG msg = { 0 };

	msg = construct_i2c_message(bus, addr, 1, &offset, sizeof(uint32_t));

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Get error code value fail, bus: 0x%x, addr: 0x%x", bus, addr);
		return ret;
	}

	memcpy(error_code, &msg.data[0], sizeof(uint32_t));

	offset = SB_CMD_FW_SMBUS_ERROR_CODE;
	msg = construct_i2c_message(bus, addr, 1, &offset, sizeof(uint8_t));

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Get smbus error code value fail, bus: 0x%x, addr: 0x%x", bus, addr);
		return ret;
	}

	*smbus_error_code = msg.data[0];
	return ret;
}

int get_boot1_complete_bit(uint8_t bus, uint8_t addr, uint8_t *bit)
{
	CHECK_NULL_ARG_WITH_RETURN(bit, -1);

	int ret = 0;
	int retry = 5;
	uint8_t offset = LCS_STATE;
	I2C_MSG msg = { 0 };

	msg = construct_i2c_message(bus, addr, 1, &offset, 1);
	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Get boot1 complete bit fail, bus: 0x%x, addr: 0x%x", bus, addr);
		return ret;
	}

	*bit = (msg.data[0] & BOOT1_COMPLETE_BIT);
	return ret;
}

int sb_cmd_fw_update_bit_operation(uint8_t bus, uint8_t addr, uint8_t optional, uint8_t bit_value)
{
	int ret = 0;
	int retry = 5;
	uint8_t set_val = 0;
	const uint8_t control_offset = 1;
	I2C_MSG msg = { 0 };

	msg.bus = bus;
	msg.target_addr = addr;
	msg.tx_len = 1;
	msg.rx_len = sizeof(sb_cmd_fw_update);
	msg.data[0] = SB_CMD_FW_UPDATE;

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Get sb_cmd_fw_update data fail, bus: 0x%x, addr: 0x%x, operation: 0x%x, bit_val: 0x%x",
			bus, addr, optional, bit_value);
		return ret;
	}

	switch (optional) {
	case BIT_GET:
		// Get BIT value
		return (msg.data[control_offset] & bit_value);

	case BIT_SET:
		// Set BIT value
		set_val = msg.data[control_offset] | bit_value;
		break;

	case BIT_CLEAR:
		// Clear BIT value
		set_val = msg.data[control_offset] & (~bit_value);
		break;

	default:
		LOG_ERR("Invalid optional of bit operation: 0x%x", optional);
		return -1;
	}

	sb_cmd_fw_update arg = { 0 };
	arg.control_option = set_val;

	msg.tx_len = sizeof(sb_cmd_fw_update) + 1;
	msg.data[0] = SB_CMD_FW_UPDATE;
	memcpy(&msg.data[1], &arg, sizeof(sb_cmd_fw_update));
	ret = i2c_master_write(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Set sb_cmd_fw_update data fail, bus: 0x%x, addr: 0x%x, operation: 0x%x, bit_val: 0x%x",
			bus, addr, optional, bit_value);
		return ret;
	}

	return 0;
}

int check_fw_update_bit_operation(uint8_t bus, uint8_t addr, uint8_t optional, uint8_t bit_value)
{
	int ret = 0;
	uint8_t expected_val = 0;

	switch (optional) {
	case BIT_SET:
	case BIT_CLEAR:
		ret = sb_cmd_fw_update_bit_operation(bus, addr, optional, bit_value);
		if (ret != 0) {
			return ret;
		}

		if (bit_value == SB_CMD_FW_UPDATE_INITIATE_DOWNLOAD_BIT) {
			//wait bit operation finish
			k_msleep(300);
		}

		ret = sb_cmd_fw_update_bit_operation(bus, addr, BIT_GET, bit_value);
		if (ret < 0) {
			LOG_ERR("Checking: Fail to get bit val: 0x%x", bit_value);
			return ret;
		}

		expected_val = ((optional == BIT_SET) ? bit_value : 0);
		if (ret != expected_val) {
			uint32_t error_code = 0;
			uint8_t smbus_error_code = 0;
			// read back error status to unlock SMbus write.
			ret = check_error_status(bus, addr, &error_code, &smbus_error_code);
			if (ret != 0) {
				LOG_ERR("Checking bit operation:Fail read error status, bus: 0x%x, addr: 0x%x",
					bus, addr);
				return ret;
			}

			// Retry
			ret = sb_cmd_fw_update_bit_operation(bus, addr, optional, bit_value);
			if (ret != 0) {
				LOG_ERR("Checking: Fail to %s bit val: 0x%x",
					((optional == BIT_SET) ? "set" : "clear"), bit_value);
				return ret;
			}

			ret = sb_cmd_fw_update_bit_operation(bus, addr, BIT_GET, bit_value);
			if (ret < 0) {
				LOG_ERR("Checking: Retry to get bit val: 0x%x fail", bit_value);
				return ret;
			}

			if (ret != expected_val) {
				LOG_ERR("Checking: Retry to %s bit val: 0x%x fail",
					((optional == BIT_SET) ? "set" : "clear"), bit_value);
				return -1;
			}
		}

		return 0;
	default:
		LOG_ERR("Invalid optional of bit operation: 0x%x", optional);
		return -1;
	}
}

int get_atm_transfer_info(uint8_t bus, uint8_t addr, uint32_t *transfer_mem_start,
			  uint32_t *transfer_mem_size)
{
	CHECK_NULL_ARG_WITH_RETURN(transfer_mem_start, -1);
	CHECK_NULL_ARG_WITH_RETURN(transfer_mem_size, -1);

	int ret = 0;
	int retry = 5;
	uint8_t offset = TRANSFER_MEM_START;
	I2C_MSG msg = { 0 };

	msg = construct_i2c_message(bus, addr, 1, &offset, TRANSFER_MEM_INFO_LEN);

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Get transfer_mem info fail, bus: 0x%x, addr: 0x%x", bus, addr);
		return ret;
	}

	memcpy(transfer_mem_start, &msg.data[0], sizeof(uint32_t));
	memcpy(transfer_mem_size, &msg.data[4], sizeof(uint32_t));
	return ret;
}

int pre_atm_fw_update_check(uint8_t *msg_buf, uint16_t buf_len, hbin_header *h_header,
			    payload_header *p_header)
{
	CHECK_NULL_ARG_WITH_RETURN(msg_buf, -1);
	CHECK_NULL_ARG_WITH_RETURN(h_header, -1);
	CHECK_NULL_ARG_WITH_RETURN(p_header, -1);

	memset(h_header, 0, sizeof(hbin_header));
	memset(p_header, 0, sizeof(payload_header));

	if (buf_len < sizeof(hbin_header)) {
		LOG_ERR("buf length less than hbin header length, buf_len: 0x%x", buf_len);
		return -1;
	}
	memcpy(h_header, &msg_buf[0], sizeof(hbin_header));
	if (buf_len < h_header->payload_offset + sizeof(payload_header)) {
		LOG_ERR("buf length less than hbin header length + payload header lenght, buf_len: 0x%x, payload_offset: 0x%x",
			buf_len, h_header->payload_offset);
		return -1;
	}
	memcpy(p_header, &msg_buf[h_header->payload_offset], sizeof(payload_header));

	if ((h_header->uid != HBIN_HEADER_UID) || (h_header->header_ver != HBIN_HEADER_VER) ||
	    (h_header->header_size != HBIN_HEADER_SIZE)) {
		LOG_ERR("Invalid hbin header data, UID: 0x%x, version: 0x%x, size: 0x%x",
			h_header->uid, h_header->header_ver, h_header->header_size);
		return -1;
	}

	if ((p_header->uid1 != PAYLOAD_HEADER_UID1) || (p_header->uid2 != PAYLOAD_HEADER_UID2) ||
	    (p_header->header_size != PAYLOAD_HEADER_SIZE) ||
	    (p_header->header_ver != PAYLOAD_HEADER_VER)) {
		LOG_ERR("Invalid payload header data, UID1: 0x%x, UID2: 0x%x, size: 0x%x, version: 0x%x",
			p_header->uid1, p_header->uid2, p_header->header_size,
			p_header->header_ver);
		return -1;
	}

	if ((p_header->type != PAYLOAD_HEADER_TYPE_QSPI) &&
	    (p_header->type != PAYLOAD_HEADER_TYPE_PSOC) &&
	    (p_header->type != PAYLOAD_HEADER_TYPE_BOOT1)) {
		LOG_ERR("Invalid payload header type: 0x%x", p_header->type);
		return -1;
	}
	return 0;
}

int check_is_fw_abort_update(uint8_t bus, uint8_t addr, uint32_t *error_code,
			     uint8_t *smbus_error_code)
{
	CHECK_NULL_ARG_WITH_RETURN(error_code, -1);
	CHECK_NULL_ARG_WITH_RETURN(smbus_error_code, -1);

	int ret = -1;

	/* Check initiate_download bit */
	ret = sb_cmd_fw_update_bit_operation(bus, addr, BIT_GET,
					     SB_CMD_FW_UPDATE_INITIATE_DOWNLOAD_BIT);
	if (ret < 0) {
		LOG_ERR("Check initiate_download bit value fail, bus: 0x%x, addr: 0x%x", bus, addr);
		return ret;
	}

	if (ret == 0) {
		/* Firmware abort the update */
		ret = check_error_status(bus, addr, error_code, smbus_error_code);
		if (ret != 0) {
			LOG_ERR("Firmware abort update but check firmware update status fail, bus: 0x%x, addr: 0x%x",
				bus, addr);
			return ret;
		}
		return FW_ABORT_UPDATE_RETURN_VAL;
	}

	return 0;
}

int wait_for_fw_ready(uint8_t bus, uint8_t addr, uint8_t bit_value, uint8_t timeout_s)
{
	int ret = -1;
	uint8_t index = 0;

	for (index = 0; index < timeout_s; ++index) {
		k_sleep(K_SECONDS(WAIT_FIRMWARE_READY_DELAY_S));

		ret = sb_cmd_fw_update_bit_operation(bus, addr, BIT_GET, bit_value);
		if (ret < 0) {
			LOG_ERR("Check firmware status fail for checking firmware ready, bus: 0x%x, addr: 0x%x, bit_val: 0x%x, current wait time: 0x%x",
				bus, addr, bit_value, index + 1);
			return -1;
		}

		if (ret != 0) {
			/* Firmware ready */
			return 0;
		}
	}

	LOG_ERR("Wait firmware ready timeout, bus: 0x%x, addr: 0x%x, bit_val: 0x%x, timeout: 0x%x",
		bus, addr, bit_value, timeout_s);
	return FW_READY_WAIT_TIMEOUT_RETURN_VAL;
}

int init_fw_update_setting(uint8_t bus, uint8_t addr, uint32_t image_size, uint32_t transfer_addr)
{
	int ret = 0;
	int retry = 5;
	uint8_t tbuf[sizeof(sb_cmd_fw_update_ext) + 1] = { 0 };
	uint8_t smbus_error_code = 0;
	uint32_t error_code = 0;
	I2C_MSG msg = { 0 };

	tbuf[0] = SB_CMD_FW_UPDATE_EXT;
	msg = construct_i2c_message(bus, addr, 1, tbuf, sizeof(sb_cmd_fw_update_ext));
	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Read sb_cmd_fw_update_ext data fail, bus: 0x%x, addr: 0x%x, payload_size: 0x%x, payload_addr: 0x%x",
			bus, addr, image_size, transfer_addr);
		return ret;
	}

	tbuf[0] = SB_CMD_FW_UPDATE_EXT;
	memcpy(&tbuf[1], &msg.data[0], sizeof(sb_cmd_fw_update_ext));

	sb_cmd_fw_update_ext *arg = (sb_cmd_fw_update_ext *)&tbuf[1];
	arg->payload_size = image_size;
	arg->payload_addr = transfer_addr;
	arg->target_src = TARGET_SRC_DEFAULT_VAL;
	arg->target_dest = TARGET_DEST_DEFAULT_VAL;
	arg->command = FW_UPDATE_COMMAND_DEFAULT_VAL;
	arg->error_code = 0;
	arg->cert1_error_code = 0;
	arg->cert2_error_code = 0;
	arg->cert3_error_code = 0;

	msg = construct_i2c_message(bus, addr, sizeof(sb_cmd_fw_update_ext) + 1, tbuf, 0);
	ret = i2c_master_write(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Initial firmware update setting fail, bus: 0x%x, addr: 0x%x, payload_size: 0x%x, payload_addr: 0x%x",
			bus, addr, image_size, transfer_addr);
		return ret;
	}

	/* Initiate download */
	ret = sb_cmd_fw_update_bit_operation(bus, addr, BIT_SET,
					     SB_CMD_FW_UPDATE_INITIATE_DOWNLOAD_BIT);
	if (ret != 0) {
		LOG_ERR("Set initiate download fail, bus: 0x%x, addr: 0x%x", bus, addr);
		return ret;
	}

	ret = check_is_fw_abort_update(bus, addr, &error_code, &smbus_error_code);
	if (ret != 0) {
		if (ret == FW_ABORT_UPDATE_RETURN_VAL) {
			LOG_ERR("Firmware abort update on init firmware update setting because error code: 0x%x, smmbus error code: 0x%x, bus: 0x%x, addr: 0x%x",
				error_code, smbus_error_code, bus, addr);
		}
		return ret;
	}

	/* Wait for firmware transfer ready */
	ret = wait_for_fw_ready(bus, addr, SB_CMD_FW_UPDATE_TRANSFER_READY_BIT,
				WAIT_FIRMWARE_READY_DELAY_TIMEOUT_S);
	if (ret == FW_READY_WAIT_TIMEOUT_RETURN_VAL) {
		LOG_ERR("Wait firmware ready timeout on init firmware update setting, bus: 0x%x, addr: 0x%x",
			bus, addr);
	}
	return ret;
}

int check_exec_status(uint8_t bus, uint8_t addr, uint8_t *status, uint8_t *result)
{
	CHECK_NULL_ARG_WITH_RETURN(status, -1);
	CHECK_NULL_ARG_WITH_RETURN(result, -1);

	int ret = 0;
	int retry = 5;
	uint8_t offset = SB_PAYLOAD_EXEC_T;
	I2C_MSG msg = { 0 };

	msg = construct_i2c_message(bus, addr, 1, &offset, sizeof(sb_payload_exec_t));
	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Read sb_payload_exec_t data fail, bus: 0x%x, addr: 0x%x", bus, addr);
		return ret;
	}

	sb_payload_exec_t *arg = (sb_payload_exec_t *)&msg.data[0];

	*status = arg->exec_status;
	*result = arg->exec_result;
	return ret;
}

void wait_firmware_work_handler(struct k_work *work_item)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work_item);
	wait_fw_update_status_info *work_info =
		CONTAINER_OF(dwork, wait_fw_update_status_info, wait_firmware_work);

	int ret = -1;
	int index = 0;
	uint8_t bit = 0;

	for (index = 0; index < work_info->timeout_s; ++index) {
		k_sleep(K_SECONDS(WAIT_FIRMWARE_READY_DELAY_S));

		if (work_info->type == PAYLOAD_HEADER_TYPE_BOOT1) {
			ret = get_boot1_complete_bit(work_info->bus, work_info->addr, &bit);
			if (ret < 0) {
				LOG_WRN("Get boot1 complete bit fail on waiting firmware work, current time: 0x%x",
					index + 1);
				continue;
			}

			if (bit == 0) {
				continue;
			}

			LOG_INF("Boot1 complete");
			work_info->status = EXEC_STATUS_COMPLETE;
			work_info->result = EXEC_RESULT_PASS;
			work_info->is_work_done = true;
			return;
		} else {
			ret = check_exec_status(work_info->bus, work_info->addr, &work_info->status,
						&work_info->result);
			if (ret != 0) {
				LOG_WRN("Check exec status fail on waiting firmware work, current time: 0x%x",
					index + 1);
				continue;
			}

			if (work_info->status == EXEC_STATUS_COMPLETE) {
				if (work_info->result != EXEC_RESULT_PASS) {
					LOG_ERR("Error: exec result is %s",
						(work_info->result == EXEC_RESULT_ABORTED ?
							 "aborted" :
							 "fail"));
				}
				work_info->is_work_done = true;
				return;
			}
		}
	}

	LOG_ERR("Wait firmware exec status timeout, total wait: %d seconds, type: 0x%x",
		work_info->timeout_s, work_info->type);
	work_info->status = EXEC_STATUS_TIMEOUT;
	work_info->is_work_done = true;
	return;
}

int transfer_data(uint8_t bus, uint8_t addr, uint32_t offset, uint8_t *msg_buf, uint16_t buf_len)
{
	CHECK_NULL_ARG_WITH_RETURN(msg_buf, -1);

	int ret = 0;
	int retry = 5;
	uint8_t index = 0;
	uint8_t data_len = 0;
	uint8_t package_count =
		((buf_len % BLOCK_DATA_COUNT == 0) ? (buf_len / BLOCK_DATA_COUNT) :
						     (buf_len / BLOCK_DATA_COUNT + 1));
	uint8_t tbuf[sizeof(sb_transferdata_packet) + 1] = { 0 };
	uint32_t page = 0;
	I2C_MSG msg = { 0 };
	sb_transfer_header header = { 0 };
	sb_transferdata_packet packet = { 0 };

	for (index = 0; index < package_count; ++index) {
		data_len = ((((index + 1) * BLOCK_DATA_COUNT) <= buf_len) ?
				    BLOCK_DATA_COUNT :
				    (buf_len - (index * BLOCK_DATA_COUNT)));

		/* Set page */
		header.page = ((offset + (index * BLOCK_DATA_COUNT)) / BLOCK_DATA_COUNT);
		tbuf[0] = TRANSFER_HEADER_PAGE;
		memcpy(&tbuf[1], &header, sizeof(sb_transfer_header));
		msg = construct_i2c_message(bus, addr, sizeof(sb_transfer_header) + 1, tbuf, 0);

		ret = i2c_master_write(&msg, retry);
		if (ret != 0) {
			LOG_ERR("Set page: 0x%x fail, bus: 0x%x, addr: 0x%x, offset: 0x%x", page,
				bus, addr, offset);
			return ret;
		}

		memcpy(&packet.data[0], &msg_buf[index * BLOCK_DATA_COUNT], data_len);
		tbuf[0] = TRANSFER_DATA_PACKET;
		memcpy(&tbuf[1], &packet, sizeof(sb_transferdata_packet));
		msg = construct_i2c_message(bus, addr, sizeof(sb_transferdata_packet) + 1, tbuf, 0);

		ret = i2c_master_write(&msg, retry);
		if (ret != 0) {
			LOG_ERR("Transfer data fail, bus: 0x%x, addr: 0x%x, page: 0x%x, offset: 0x%x",
				bus, addr, page, offset);
			return ret;
		}
	}
	return 0;
}

int atm_fw_update(uint8_t bus, uint8_t addr, uint32_t offset, uint8_t *msg_buf, uint16_t buf_len,
		  uint32_t image_size, bool is_end_package)
{
	CHECK_NULL_ARG_WITH_RETURN(msg_buf, -1);

	static hbin_header h_header = { 0 };
	static payload_header p_header = { 0 };
	static uint32_t transfer_mem_start = 0;
	static uint32_t transfer_mem_size = 0;

	int ret = -1;
	uint8_t exec_status = 0;
	uint8_t exec_result = 0;
	uint8_t smbus_error_code = 0;
	uint32_t error_code = 0;

	if (offset == 0) {
		ret = get_atm_transfer_info(bus, addr, &transfer_mem_start, &transfer_mem_size);
		if (ret != 0) {
			return ret;
		}
		if (image_size >= transfer_mem_size) {
			LOG_ERR("Artemis module image size more than transfer_mem_size, image_size: 0x%x, transfer_mem_size: 0x%x",
				image_size, transfer_mem_size);
			return -1;
		}
		ret = pre_atm_fw_update_check(msg_buf, buf_len, &h_header, &p_header);
		if (ret != 0) {
			LOG_ERR("Artemis module pre-check fail");
			return ret;
		}

		/* Step 1 ~ 3 */
		ret = init_fw_update_setting(bus, addr, image_size, transfer_mem_start);
		if (ret != 0) {
			LOG_ERR("Artemis module initialize setting fail");
			goto exit;
		}
	}

	/* Step 4 */
	ret = transfer_data(bus, addr, offset, msg_buf, buf_len);
	if (ret != 0) {
		LOG_ERR("Artemis module transfer data fail");
		goto exit;
	}

	if (is_end_package) {
		/* Step 5: Set transfer_done */
		ret = check_fw_update_bit_operation(bus, addr, BIT_SET,
						    SB_CMD_FW_UPDATE_TRANSFER_DONE_BIT);
		if (ret != 0) {
			LOG_ERR("Set transfer_done fail, bus: 0x%x, addr: 0x%x", bus, addr);
			goto exit;
		}

		/* Step 6: Check firmware update status and Check firmware is progress */
		ret = check_is_fw_abort_update(bus, addr, &error_code, &smbus_error_code);
		if (ret != 0) {
			if (ret == FW_ABORT_UPDATE_RETURN_VAL) {
				LOG_ERR("Firmware abort update on Step 6 because error code: 0x%x, smbus error code: 0x%x, bus: 0x%x, addr: 0x%x",
					error_code, smbus_error_code, bus, addr);
			}
			goto exit;
		}

		ret = wait_for_fw_ready(bus, addr, SB_CMD_FW_UPDATE_IN_PROGRESS_BIT,
					WAIT_FIRMWARE_READY_DELAY_TIMEOUT_S);
		if (ret != 0) {
			if (ret == FW_READY_WAIT_TIMEOUT_RETURN_VAL) {
				LOG_ERR("Wait firmware update complete timeout on Step 6, bus: 0x%x, addr: 0x%x",
					bus, addr);
			}
			goto exit;
		}

		/* Step 7: Clear transfer_done */
		ret = check_fw_update_bit_operation(bus, addr, BIT_CLEAR,
						    SB_CMD_FW_UPDATE_TRANSFER_DONE_BIT);
		if (ret != 0) {
			LOG_ERR("Clear transfer_done fail, bus: 0x%x, addr: 0x%x", bus, addr);
			goto exit;
		}

		/* Step 8: Check firmware update status, Wait firmware update complete and Clear update_complete */
		ret = check_is_fw_abort_update(bus, addr, &error_code, &smbus_error_code);
		if (ret != 0) {
			if (ret == FW_ABORT_UPDATE_RETURN_VAL) {
				LOG_ERR("Firmware abort update on Step 8 because error code: 0x%x, smbus error code: 0x%x,  bus: 0x%x, addr: 0x%x",
					error_code, smbus_error_code, bus, addr);
			}
			goto exit;
		}

		ret = wait_for_fw_ready(bus, addr, SB_CMD_FW_UPDATE_UPDATE_COMPLETE_BIT,
					WAIT_FIRMWARE_READY_DELAY_TIMEOUT_S);
		if (ret != 0) {
			if (ret == FW_READY_WAIT_TIMEOUT_RETURN_VAL) {
				LOG_ERR("Wait firmware update complete timeout on Step 8, bus: 0x%x, addr: 0x%x",
					bus, addr);
			}
			goto exit;
		}

		ret = check_fw_update_bit_operation(bus, addr, BIT_CLEAR,
						    SB_CMD_FW_UPDATE_UPDATE_COMPLETE_BIT);
		if (ret != 0) {
			LOG_ERR("Clear update_complete fail on Step 8, bus: 0x%x, addr: 0x%x", bus,
				addr);
			goto exit;
		}

		/* Step 9: Check error_code, exec_status, exec_result */
		if (p_header.type == PAYLOAD_HEADER_TYPE_BOOT1) {
			uint8_t bit = 0;
			ret = get_boot1_complete_bit(bus, addr, &bit);
			if (ret < 0) {
				LOG_ERR("Get boot1 complete bit fail on Step 9");
			}

			if (bit != 0) {
				LOG_WRN("Boot1 complete bit should be 0");
			}
		}

		ret = check_error_status(bus, addr, &error_code, &smbus_error_code);
		if (ret != 0) {
			LOG_ERR("Get error code fail on Step 9, bus: 0x%x, addr: 0x%x", bus, addr);
			goto exit;
		}

		if (p_header.type != PAYLOAD_HEADER_TYPE_BOOT1) {
			ret = check_exec_status(bus, addr, &exec_status, &exec_result);
			if (ret != 0) {
				LOG_ERR("Get exec status fail on Step 9, bus: 0x%x, addr: 0x%x",
					bus, addr);
				goto exit;
			}
		}

		LOG_INF("Error code: 0x%x, smbus error code: 0x%x, exec status: 0x%x, exec_result: 0x%x after firmware update complete",
			error_code, smbus_error_code, exec_status, exec_result);

		if (atm_wait_fw_info.is_init != true) {
			k_work_init_delayable(&(atm_wait_fw_info.wait_firmware_work),
					      wait_firmware_work_handler);
			atm_wait_fw_info.is_init = true;
		}

		atm_wait_fw_info.bus = bus;
		atm_wait_fw_info.addr = addr;
		atm_wait_fw_info.type = p_header.type;
		if (p_header.type == PAYLOAD_HEADER_TYPE_QSPI) {
			atm_wait_fw_info.timeout_s = WAIT_QSPI_FIRMWARE_UPDATE_COMPLETE_TIMEOUT_S;
		} else if (p_header.type == PAYLOAD_HEADER_TYPE_PSOC) {
			atm_wait_fw_info.timeout_s = WAIT_PSOC_FIRMWARE_UPDATE_COMPLETE_TIMEOUT_S;
		} else {
			atm_wait_fw_info.timeout_s = WAIT_BOOT1_FIRMWARE_UPDATE_COMPLETE_TIMEOUT_S;
		}

		k_work_schedule_for_queue(&plat_work_q, &atm_wait_fw_info.wait_firmware_work,
					  K_NO_WAIT);
	} else {
		return 0;
	}
exit:
	/* Clear initiate_download */
	if (check_fw_update_bit_operation(bus, addr, BIT_CLEAR,
					  SB_CMD_FW_UPDATE_INITIATE_DOWNLOAD_BIT) != 0) {
		LOG_ERR("Abort firmware update fail, bus: 0x%x, addr: 0x%x", bus, addr);
	}
	return ret;
}

#define CLK_GEN_SSC_HW_EXPECT_VALUE 0x02
void init_clk_gen_spread_spectrum_control_register()
{
	int ret = -1;
	int retry = 5;
	uint8_t ssc = 0;
	I2C_MSG msg = { 0 };

	msg.bus = I2C_BUS3;
	msg.target_addr = CLOCK_GEN_ADDR;
	msg.rx_len = 2;
	msg.tx_len = 1;
	msg.data[0] = 0x01;

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Get spread spectrum control register fail data");
		return;
	}

	ssc = msg.data[1];
	if ((ssc >> 6) != CLK_GEN_SSC_HW_EXPECT_VALUE) {
		LOG_INF("Get unexpected SSC HW control data %x", ssc);
		//enable software control ssc
		msg.rx_len = 0;
		msg.tx_len = 3;
		msg.data[0] = 0x01;
		msg.data[1] = 0x01;
		msg.data[2] = ((ssc | BIT(5)) | BIT(4)) & (~BIT(3));

		ret = i2c_master_write(&msg, retry);
		if (ret != 0) {
			LOG_ERR("Set spread spectrum control register fail data %x", msg.data[3]);
		}
	}
}
