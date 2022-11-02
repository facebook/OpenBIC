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

#include <logging/log.h>
#include "plat_apml.h"

LOG_MODULE_REGISTER(plat_apml);

static bool is_threshold_set = false;

bool get_tsi_status()
{
	return is_threshold_set;
}

void reset_tsi_status()
{
	is_threshold_set = false;
}

void set_tsi_threshold()
{
	uint8_t read_data;
	/* Set high temperature threshold */
	if (apml_read_byte(APML_BUS, SB_TSI_ADDR, SBTSI_HIGH_TEMP_INTEGER_THRESHOLD, &read_data)) {
		LOG_ERR("Failed to read high temperature threshold.");
		return;
	}
	if (read_data != TSI_HIGH_TEMP_THRESHOLD) {
		if (apml_write_byte(APML_BUS, SB_TSI_ADDR, SBTSI_HIGH_TEMP_INTEGER_THRESHOLD,
				    TSI_HIGH_TEMP_THRESHOLD)) {
			LOG_ERR("Failed to set TSI temperature threshold.");
			return;
		}
	}

	/* Set the frequency of the high/low temperature comparison to 64 Hz */
	if (apml_read_byte(APML_BUS, SB_TSI_ADDR, SBTSI_UPDATE_RATE, &read_data)) {
		LOG_ERR("Failed to read alert config.");
		return;
	}
	if (read_data != TSI_TEMP_ALERT_UPDATE_RATE) {
		if (apml_write_byte(APML_BUS, SB_TSI_ADDR, SBTSI_UPDATE_RATE,
				    TSI_TEMP_ALERT_UPDATE_RATE)) {
			LOG_ERR("Failed to set alert config.");
			return;
		}
	}
	is_threshold_set = true;
}
