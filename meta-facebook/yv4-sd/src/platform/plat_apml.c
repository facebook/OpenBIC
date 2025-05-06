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

#include <stdlib.h>
#include <logging/log.h>
#include "power_status.h"
#include "ipmi.h"
#include "libutil.h"
#include "apml.h"
#include "plat_apml.h"
#include "plat_class.h"

LOG_MODULE_REGISTER(plat_apml);

static bool is_threshold_set = false;
static uint8_t cpuid[16];

int pal_check_sbrmi_command_code_length()
{
	if (!get_post_status()) {
		return 0;
	}

	// YV4 platform will only be equipped with AMD Turin CPU with SBRMI Rev 2.1.
	// For Rev 2.1, it requires SBRMI 2-byte command code.

	// BIC has a chance to get wrong revision and therefore
	// set the wrong command code length (1-byte).
	// It will cause BIC to be unable to read CPU power.
	// Follow AMD's suggestion, hardcode SBRMI command code length to 2 bytes.

	set_sbrmi_command_code_len(SBRMI_CMD_CODE_LEN_TWO_BYTE);
	return 0;
}

uint8_t pal_get_apml_bus()
{
	uint8_t board_rev = APML_BUS_UNKNOWN;

	if (get_board_rev(&board_rev) == false) {
		LOG_ERR("Failed to get board revision");
		return APML_BUS_UNKNOWN;
	}

	if (board_rev <= BOARD_REV_EVT) {
		return I2C_BUS14;
	} else {
		// For DVT and later, the hardware design was changed to I2C_BUS10
		return I2C_BUS10;
	}
}

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
	uint8_t apml_bus = apml_get_bus();
	// Set high temperature threshold
	if (apml_read_byte(apml_bus, SB_TSI_ADDR, SBTSI_HIGH_TEMP_INTEGER_THRESHOLD, &read_data)) {
		LOG_ERR("Failed to read high temperature threshold.");
		return;
	}
	if (read_data != TSI_HIGH_TEMP_THRESHOLD) {
		if (apml_write_byte(apml_bus, SB_TSI_ADDR, SBTSI_HIGH_TEMP_INTEGER_THRESHOLD,
				    TSI_HIGH_TEMP_THRESHOLD)) {
			LOG_ERR("Failed to set TSI temperature threshold.");
			return;
		}
	}

	// Set the frequency of the high/low temperature comparison to 64 Hz
	if (apml_read_byte(apml_bus, SB_TSI_ADDR, SBTSI_UPDATE_RATE, &read_data)) {
		LOG_ERR("Failed to read alert config.");
		return;
	}
	if (read_data != TSI_TEMP_ALERT_UPDATE_RATE) {
		if (apml_write_byte(apml_bus, SB_TSI_ADDR, SBTSI_UPDATE_RATE,
				    TSI_TEMP_ALERT_UPDATE_RATE)) {
			LOG_ERR("Failed to set alert config.");
			return;
		}
	}
	is_threshold_set = true;
}

static void read_cpuid_callback(const apml_msg *msg)
{
	CHECK_NULL_ARG(msg);
	const cpuid_RdData *rd_data = (const cpuid_RdData *)msg->RdData;
	uint8_t exc_value = (uint8_t)msg->ui32_arg;
	memcpy(&cpuid[exc_value * 8], rd_data->data_out, 8);
}

static void read_cpuid_error_callback(const apml_msg *msg)
{
	CHECK_NULL_ARG(msg);
	uint8_t exc_value = (uint8_t)msg->ui32_arg;
	LOG_ERR("Failed to read CPU ID, exc_value %d", exc_value);
}

void read_cpuid()
{
	uint8_t apml_bus = apml_get_bus();

	memset(cpuid, 0, sizeof(cpuid));

	// read eax&ebx
	apml_msg apml_data = { 0 };
	apml_data.msg_type = APML_MSG_TYPE_CPUID;
	apml_data.bus = apml_bus;
	apml_data.target_addr = SB_RMI_ADDR;
	apml_data.cb_fn = read_cpuid_callback;
	apml_data.error_cb_fn = read_cpuid_error_callback;
	apml_data.ui32_arg = 0x00;
	if (get_sbrmi_command_code_len() == SBRMI_CMD_CODE_LEN_TWO_BYTE) {
		cpuid_WrData_TwoPOne *wrdata = (cpuid_WrData_TwoPOne *)&apml_data.WrData;
		wrdata->ecx_value = 0x00;
	} else {
		cpuid_WrData *wrdata = (cpuid_WrData *)&apml_data.WrData;
		wrdata->ecx_value = 0x00;
	}
	apml_read(&apml_data);

	// read ecx&edx
	memset(&apml_data, 0x00, sizeof(apml_msg));
	apml_data.msg_type = APML_MSG_TYPE_CPUID;
	apml_data.bus = apml_bus;
	apml_data.target_addr = SB_RMI_ADDR;
	apml_data.cb_fn = read_cpuid_callback;
	apml_data.error_cb_fn = read_cpuid_error_callback;
	apml_data.ui32_arg = 0x01;
	if (get_sbrmi_command_code_len() == SBRMI_CMD_CODE_LEN_TWO_BYTE) {
		cpuid_WrData_TwoPOne *wrdata = (cpuid_WrData_TwoPOne *)&apml_data.WrData;
		wrdata->ecx_value = 0x01;
	} else {
		cpuid_WrData *wrdata = (cpuid_WrData *)&apml_data.WrData;
		wrdata->ecx_value = 0x01;
	}
	apml_read(&apml_data);
}

const uint8_t *get_cpuid()
{
	return cpuid;
}
