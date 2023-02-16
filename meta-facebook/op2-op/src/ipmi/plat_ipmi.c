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
#include <logging/log.h>
#include "ipmi.h"
#include "libutil.h"
#include "pt5161l.h"
#include "power_status.h"
#include "plat_class.h"
#include "plat_i2c.h"
#include "plat_ipmi.h"
#include "plat_power_seq.h"
#include "plat_sensor_table.h"

LOG_MODULE_DECLARE(plat_ipmi);

uint8_t get_add_sel_target_interface()
{
	return HD_BIC_IPMB;
}

void OEM_1S_GET_SET_M2(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 2) {
		msg->data_len = 0;
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	msg->completion_code = CC_INVALID_DATA_FIELD;
	msg->data_len = 0;
	uint8_t action = msg->data[1];
	uint8_t device_index = msg->data[0] - 1; //BMC send 1 base device number

	if (device_index >= MAX_E1S_IDX) {
		return;
	}

	switch (action) {
	case DEVICE_POWER_OFF:
		notify_cpld_e1s_present(device_index, GPIO_HIGH);
		abort_e1s_power_thread(device_index);
		e1s_power_off_thread(device_index);
		msg->completion_code = CC_SUCCESS;
		break;
	case DEVICE_POWER_ON:
		notify_cpld_e1s_present(device_index, GPIO_LOW);
		abort_e1s_power_thread(device_index);
		e1s_power_on_thread(device_index);
		msg->completion_code = CC_SUCCESS;
		break;
	case DEVICE_PRESENT:
		msg->data[0] = get_e1s_present(device_index);
		msg->data_len = 1;
		msg->completion_code = CC_SUCCESS;
		break;
	case DEVICE_POWER_GOOD:
		msg->data[0] = get_e1s_power_good(device_index);
		msg->data_len = 1;
		msg->completion_code = CC_SUCCESS;
		break;
	default:
		msg->completion_code = CC_INVALID_DATA_FIELD;
		msg->data_len = 0;
	}
}

void OEM_1S_GET_FW_VERSION(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 1) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t component;
	component = msg->data[0];

	uint8_t card_type = get_card_type();
	I2C_MSG *i2c_msg;
	if (card_type == CARD_TYPE_OPA) {
		i2c_msg = malloc(sizeof(I2C_MSG));
	}

	switch (component) {
	case OL2_COMPNT_BIC:
		msg->data[0] = BIC_FW_YEAR_MSB;
		msg->data[1] = BIC_FW_YEAR_LSB;
		msg->data[2] = BIC_FW_WEEK;
		msg->data[3] = BIC_FW_VER;
		msg->data[4] = BIC_FW_platform_0;
		msg->data[5] = BIC_FW_platform_1;
		msg->data[6] = BIC_FW_platform_2;
		msg->data_len = 7;
		msg->completion_code = CC_SUCCESS;
		break;
	case OL2_COMPNT_RETIMER:
		if (card_type == CARD_TYPE_OPA) {
			if (get_DC_status()) {
				i2c_msg->bus = I2C_BUS4;
				i2c_msg->target_addr = EXPA_RETIMER_ADDR;
				if (get_retimer_fw_version(i2c_msg, msg->data)) {
					msg->data_len = 4;
					msg->completion_code = CC_SUCCESS;
				} else {
					msg->completion_code = CC_UNSPECIFIED_ERROR;
				}
			} else {
				msg->completion_code = CC_NOT_SUPP_IN_CURR_STATE;
			}

			SAFE_FREE(i2c_msg);
		} else {
			msg->completion_code = CC_NOT_SUPP_IN_CURR_STATE;
		}
		break;
	default:
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		break;
	}
	return;
}
