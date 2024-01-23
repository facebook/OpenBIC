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

#ifndef PLAT_DEV
#define PLAT_DEV

#include <stdint.h>
#include "sensor.h"

#define FREYA_STATUS_BLOCK_OFFSET 0x00
#define FREYA_STATUS_BLOCK_LENGTH 0x08
#define FREYA_READY_STATUS_BIT BIT(6)
#define FREYA_READY_STATUS_OFFSET 0x01
#define FREYA_READY_STATUS_LENGTH 1
#define NVME_TEMPERATURE_INDEX 3
#define FREYA_MODULE_IDENTIFIER_BLOCK_OFFSET 0x20
#define FREYA_MODULE_IDENTIFIER_BLOCK_LENGTH 0x37
#define IS_FREYA_MODULE_IDENTIFIER_SUPPORT 0x01
#define FREYA_MODULE_IDENTIFIER_OFFSET 0x01
#define FREYA_MODULE_IDENTIFIER_LENGTH 1
#define FREYA_FFI_OFFSET 0x2B
#define FREYA_FFI_LENGTH 1
#define FREYA_FIRMWARE_VERSION_BLOCK_OFFSET 0x68
#define FREYA_FIRMWARE_VERSION_BLOCK_LENGTH 0x08
#define FREYA_FIRMWARE_VERSION_OFFSET 0x02
#define FREYA_FIRMWARE_VERSION_LENGTH 5
#define FREYA_NOT_READY_RET_CODE -2
#define FREYA_NOT_SUPPORT_MODULE_IDENTIFIER_RET_CODE -3

typedef struct _freya_fw_info {
	uint8_t is_freya_ready;
	uint8_t is_module_identifier_support;
	uint8_t form_factor_info; // FFI
	uint8_t major_version;
	uint8_t minor_version;
	uint8_t additional_version;
	uint8_t secondary_major_version;
	uint8_t secondary_minor_version;
} freya_fw_info;

typedef struct _freya_info {
	bool is_cache_freya1_info;
	bool is_cache_freya2_info;
	freya_fw_info freya1_fw_info;
	freya_fw_info freya2_fw_info;
} freya_info;

typedef struct _vr_fw_info {
	uint8_t checksum[4];
	uint8_t remaining_write;
	uint8_t vendor;
	bool is_init;
} vr_fw_info;

typedef struct _switch_error_check_info {
	bool is_addsel;
	bool is_init_work;
	uint8_t device_type;
	uint32_t error_status;
	struct k_work_delayable add_sel_work;
} switch_error_check_info;

enum FREYA_ID {
	FREYA_ID1,
	FREYA_ID2,
};

extern freya_info accl_freya_info[];
extern vr_fw_info cb_vr_fw_info;
extern switch_error_check_info sw_error_check_info[];

void clear_freya_cache_flag(uint8_t card_id);
void clear_accl_cable_power_fault_flag();
int get_freya_fw_info(uint8_t bus, uint8_t addr, freya_fw_info *fw_info);
bool is_sw_ready(uint8_t sensor_num);
void init_sw_heartbeat_work();
void clear_sw_error_check_flag();
void get_switch_error_status(uint8_t sensor_num, uint8_t bus, uint8_t addr, uint8_t index);
bool init_vr_write_protect(uint8_t bus, uint8_t addr, uint8_t default_val);

#endif
