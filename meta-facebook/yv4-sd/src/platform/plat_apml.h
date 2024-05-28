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

#ifndef PLAT_APML_H
#define PLAT_APML_H

#include "apml.h"
#include "plat_i2c.h"

#define TSI_HIGH_TEMP_THRESHOLD 0x5F
#define TSI_TEMP_ALERT_UPDATE_RATE 0x0A
#define PLAT_SBRMI_REVISION 0x20

typedef struct _addc_trigger_info {
	uint8_t event_version;
	uint8_t RAS_status;
	uint8_t total_socket;
	uint8_t apml_index;
	uint8_t cpuid[16];
} __attribute__((__packed__)) addc_trigger_info;

bool get_tsi_status();
void reset_tsi_status();
void set_tsi_threshold();
void read_cpuid();

#endif
