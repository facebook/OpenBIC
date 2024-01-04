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
#include "libipmi.h"
#include "libutil.h"
#include "ipmi.h"
#include "ipmb.h"
#include "plat_ipmi.h"
#include "plat_ipmb.h"
#include "plat_gpio.h"
#include "plat_isr.h"
#include "plat_sensor_table.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(plat_ipmi);

void OEM_CABLE_DETECTION(ipmi_msg *msg)
{
	if (msg == NULL) {
		LOG_ERR("Failed due to parameter *msg is NULL");
		return;
	}

	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	msg->completion_code = CC_SUCCESS;

	common_addsel_msg_t *sel_msg = (common_addsel_msg_t *)malloc(sizeof(common_addsel_msg_t));
	if (sel_msg == NULL) {
		LOG_ERR("Memory allocation failed!");
		return;
	}

	sel_msg->sensor_type = IPMI_OEM_SENSOR_TYPE_OEM;
	sel_msg->event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
	sel_msg->sensor_number = CMD_OEM_CABLE_DETECTION;
	sel_msg->event_data2 = 0xFF;
	sel_msg->event_data3 = 0xFF;

	if (msg->InF_source == SLOT1_BIC) {
		ISR_SLOT3_PRESENT();
		if (((gpio_get(SLOT1_ID1_DETECT_BIC_N) << 1) |
		     (gpio_get(SLOT1_ID0_DETECT_BIC_N) << 0)) != BB_CABLE_MATCH_SLOT1) {
			sel_msg->event_data1 = IPMI_OEM_EVENT_OFFSET_SLOT3_INSERT_SLOT1;
			sel_msg->InF_target = SLOT1_BIC;
			common_add_sel_evt_record(sel_msg);
		}
	} else if (msg->InF_source == SLOT3_BIC) {
		ISR_SLOT1_PRESENT();
		if (((gpio_get(SLOT3_ID1_DETECT_BIC_N) << 1) |
		     (gpio_get(SLOT3_ID0_DETECT_BIC_N) << 0)) != BB_CABLE_MATCH_SLOT3) {
			sel_msg->event_data1 = IPMI_OEM_EVENT_OFFSET_SLOT1_INSERT_SLOT3;
			sel_msg->InF_target = SLOT3_BIC;
			common_add_sel_evt_record(sel_msg);
		}
	} else {
		LOG_ERR("Unknown request");
	}

	SAFE_FREE(sel_msg);
	return;
}

void OEM_1S_INFORM_PEER_SLED_CYCLE(ipmi_msg *msg)
{
	if (msg == NULL) {
		LOG_ERR("Failed due to parameter *msg is NULL");
		return;
	}

	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t peer_slot_interface = 0;
	common_addsel_msg_t *sel_msg;
	bool ret = false;

	if (msg->InF_source == SLOT1_BIC) {
		peer_slot_interface = SLOT3_BIC;
	} else if (msg->InF_source == SLOT3_BIC) {
		peer_slot_interface = SLOT1_BIC;
	} else {
		msg->completion_code = CC_PARAM_OUT_OF_RANGE;
		return;
	}

	sel_msg = (common_addsel_msg_t *)malloc(sizeof(common_addsel_msg_t));
	if (sel_msg == NULL) {
		LOG_ERR("Failed to malloc sel_msg");
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}

	sel_msg->InF_target = peer_slot_interface;
	sel_msg->sensor_type = IPMI_SENSOR_TYPE_POWER_UNIT;
	sel_msg->sensor_number = SENSOR_NUM_POWER_DETECT;
	sel_msg->event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
	sel_msg->event_data1 = 0x00;
	sel_msg->event_data2 = 0x00;
	sel_msg->event_data3 = 0x00;

	ret = common_add_sel_evt_record(sel_msg);
	if (ret == false) {
		LOG_ERR("Failed to add SEL to interface: 0x%x", sel_msg->InF_target);
		msg->completion_code = CC_UNSPECIFIED_ERROR;
	} else {
		msg->completion_code = CC_SUCCESS;
	}

	SAFE_FREE(sel_msg);
	return;
}
