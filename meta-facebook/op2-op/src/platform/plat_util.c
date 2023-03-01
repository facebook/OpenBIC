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
#include "ipmi.h"
#include "libutil.h"
#include "libipmi.h"
#include "plat_class.h"
#include "plat_sensor_table.h"
#include "plat_util.h"

LOG_MODULE_REGISTER(plat_util);

void send_system_status_event(uint8_t event_type, uint8_t error_type, uint8_t device_index)
{
	common_addsel_msg_t *sel_msg = malloc(sizeof(common_addsel_msg_t));
	CHECK_NULL_ARG(sel_msg);

	uint8_t card_position = get_card_position();

	switch (card_position) {
	case CARD_POSITION_2OU:
		sel_msg->InF_target = EXP1_IPMB;
		break;
	case CARD_POSITION_4OU:
		sel_msg->InF_target = EXP3_IPMB;
		break;
	default:
		sel_msg->InF_target = HD_BIC_IPMB;
		break;
	}
	sel_msg->sensor_type = IPMI_OEM_SENSOR_TYPE_OEM;
	sel_msg->sensor_number = SENSOR_NUM_SYSTEM_STATUS;
	sel_msg->event_type = event_type;
	sel_msg->event_data1 = error_type;
	sel_msg->event_data2 = card_position;
	sel_msg->event_data3 = device_index;

	if (!common_add_sel_evt_record(sel_msg)) {
		LOG_ERR("Failed to add SEL to BMC, type: 0x%x data: 0x%02x%02x%02x",
			sel_msg->event_type, sel_msg->event_data1, sel_msg->event_data2,
			sel_msg->event_data3);
	}

	SAFE_FREE(sel_msg);
}
