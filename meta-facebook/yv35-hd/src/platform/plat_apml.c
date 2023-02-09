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
#include "plat_apml.h"
#include "ipmi.h"
#include "libutil.h"

LOG_MODULE_REGISTER(plat_apml);

static bool is_threshold_set = false;
static uint8_t cpuid[16];

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

static void read_cpuid_callback(apml_msg *msg)
{
	CHECK_NULL_ARG(msg);
	cpuid_RdData *rd_data = (cpuid_RdData *)msg->RdData;
	uint8_t exc_value = (uint8_t)msg->ui32_arg;
	memcpy(&cpuid[exc_value * 8], rd_data->data_out, 8);
}

static void read_cpuid_error_callback(apml_msg *msg)
{
	CHECK_NULL_ARG(msg);
	uint8_t exc_value = (uint8_t)msg->ui32_arg;
	LOG_ERR("Failed to read CPU ID, exc_value %d", exc_value);
}

void read_cpuid()
{
	memset(cpuid, 0, sizeof(cpuid));

	/* read eax&ebx */
	apml_msg apml_data = { 0 };
	apml_data.msg_type = APML_MSG_TYPE_CPUID;
	apml_data.bus = APML_BUS;
	apml_data.target_addr = SB_RMI_ADDR;
	apml_data.cb_fn = read_cpuid_callback;
	apml_data.error_cb_fn = read_cpuid_error_callback;
	apml_data.ui32_arg = 0x00;
	cpuid_WrData *wrdata = (cpuid_WrData *)&apml_data.WrData;
	wrdata->ecx_value = 0x00;
	apml_read(&apml_data);

	/* read ecx&edx */
	memset(&apml_data, 0x00, sizeof(apml_msg));
	apml_data.msg_type = APML_MSG_TYPE_CPUID;
	apml_data.bus = APML_BUS;
	apml_data.target_addr = SB_RMI_ADDR;
	apml_data.cb_fn = read_cpuid_callback;
	apml_data.error_cb_fn = read_cpuid_error_callback;
	apml_data.ui32_arg = 0x01;
	wrdata = (cpuid_WrData *)&apml_data.WrData;
	wrdata->ecx_value = 0x01;
	apml_read(&apml_data);
}

void send_apml_alert_to_bmc(uint8_t ras_status)
{
	ipmi_msg *msg = (ipmi_msg *)malloc(sizeof(ipmi_msg));
	if (msg == NULL) {
		LOG_ERR("Failed to allocate memory");
		return;
	}
	memset(msg, 0, sizeof(*msg));

	msg->data_len = 3 + sizeof(addc_trigger_info);
	msg->InF_source = SELF;
	msg->InF_target = BMC_IPMB;
	msg->netfn = NETFN_OEM_1S_REQ;
	msg->cmd = CMD_OEM_1S_SEND_APML_ALERT_TO_BMC;

	msg->data[0] = IANA_ID & 0xFF;
	msg->data[1] = (IANA_ID >> 8) & 0xFF;
	msg->data[2] = (IANA_ID >> 16) & 0xFF;
	addc_trigger_info *addc_info = (addc_trigger_info *)&msg->data[3];
	addc_info->event_version = 0x00;
	addc_info->RAS_status = ras_status;
	addc_info->total_socket = 1;
	addc_info->apml_index = 0;

	memcpy(addc_info->cpuid, cpuid, sizeof(addc_info->cpuid));

	ipmb_error status = ipmb_read(msg, IPMB_inf_index_map[msg->InF_target]);
	if (status != IPMB_ERROR_SUCCESS)
		LOG_ERR("Failed to send RAS status 0x%02x to BMC, return %d", ras_status, status);

	SAFE_FREE(msg);
}
