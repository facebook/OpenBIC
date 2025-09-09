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

typedef struct _rainbow_cpld_info_ rainbow_cpld_info;

typedef struct _rainbow_cpld_info_ {
	uint8_t cpld_offset;
	uint8_t dc_off_defaut;
	uint8_t dc_on_defaut;
	bool is_fault_log; // if true, check the value is defaut or not
	uint8_t is_fault_bit_map; //flag for fault

	//flag for 1st polling
	bool is_first_polling;

	//flag for 1st polling after changing DC status
	bool is_first_polling_after_dc_change;

	//temp data for last polling
	uint8_t last_polling_value;

	bool (*status_changed_cb)(rainbow_cpld_info *, uint8_t *);

	//bit check mask
	uint8_t bit_check_mask; //bit check mask

	uint8_t event_type;

} rainbow_cpld_info;

typedef struct _vr_fault_info {
	uint8_t mtia_event_source;
	uint8_t cpld_reg_offset;
	uint8_t cpld_reg_bit;
	bool is_pmbus_vr;
	uint8_t sensor_id;
} vr_fault_info;

#endif
