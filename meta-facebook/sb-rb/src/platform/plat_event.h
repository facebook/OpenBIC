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
#ifndef PLAT_EVENT_H
#define PLAT_EVENT_H
#include <stdbool.h>
#include <stdint.h>
#include "plat_cpld.h"

#define HAMSA_SMB_ERR_EVENT_HEADER 0x60

typedef struct _vr_fault_info {
	uint8_t mtia_event_source;
	uint8_t cpld_reg_offset;
	uint8_t cpld_reg_bit;
	bool is_pmbus_vr;
	uint8_t rail_id;
} vr_fault_info;

enum event_type {
	VR_POWER_FAULT = 0x01,
	ASIC_MODULE_ERROR = 0x29,
};

typedef struct __attribute__((packed)) _plat_asic_error_event {
	uint8_t event_id_0;
	uint8_t event_id_1;
	uint8_t chip_id;
	uint8_t module_id;
} plat_asic_error_event;

void process_mtia_vr_power_fault_sel(cpld_info *cpld_info, uint8_t *current_cpld_value);
void plat_set_ac_on_log();
void plat_set_dc_on_log(bool is_assert);
void plat_set_iris_temp_error_log(bool is_assert, uint8_t sensor_id);
void asic_thermtrip_error_log(bool is_assert);
void plat_asic_error_error_log(bool is_assert, plat_asic_error_event event);
plat_asic_error_event *plat_get_asic_error_event();
#endif
