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

#ifndef VR_FAULT_H
#define VR_FAULT_H

#include <stdlib.h>
#include <zephyr.h>

#define VR_CPLD_NO_PWR_FAULT 0x00
#define VR_TYPE_IS_UNKNOWN 0x00
#define VR_MONITOR_STOP_TIME 10 // ms

typedef struct {
	uint8_t bus;
	uint8_t addr;
	uint8_t offset;
} cpld_vr_reg_t;

typedef struct {
	uint8_t bit;
	uint8_t event;
	uint8_t bus;
	uint8_t addr;
	uint8_t page;
} vr_pwr_fault_t;

typedef struct _add_vr_sel_info {
	cpld_vr_reg_t cpld_vr_reg;
	struct k_work_delayable add_vr_work;
} add_vr_sel_info;

extern const vr_pwr_fault_t vr_pwr_fault_table[];
extern const size_t vr_pwr_fault_table_size;
extern const cpld_vr_reg_t cpld_vr_reg_table;

uint8_t pal_get_vr_vender_type();
void pal_record_vr_power_fault(uint8_t event_type, uint8_t error_type, uint8_t vr_data1,
			       uint8_t vr_data2);
bool pal_skip_pmbus_cmd_code(uint8_t vendor_type, uint8_t cmd, uint8_t page);
void vr_pwr_fault_handler(struct k_work *work_item);

#endif
