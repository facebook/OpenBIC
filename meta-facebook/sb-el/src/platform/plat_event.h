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

#include "plat_cpld.h"

typedef struct _vr_fault_info {
	uint8_t mtia_event_source;
	uint8_t cpld_reg_offset;
	uint8_t cpld_reg_bit;
	bool is_pmbus_vr;
	uint8_t rail_id;
} vr_fault_info;

 void process_mtia_vr_power_fault_sel(cpld_info *cpld_info, uint8_t *current_cpld_value);