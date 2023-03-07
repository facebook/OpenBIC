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
#include "rg3mxxb12.h"
#include "power_status.h"
#include "plat_class.h"
#include "plat_i2c.h"
#include "plat_ipmi.h"
#include "plat_power_seq.h"
#include "plat_sensor_table.h"
#include "plat_hook.h"

LOG_MODULE_REGISTER(plat_ipmi);

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

/* Request:
 * Byte 0: E1.S device ID
 * Byte 1: Target address (8 bits)
 * Byte 2: Read length
 * Byte 3~: Write data
*/
void OEM_1S_SAFE_WRITE_READ_M2_DATA(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	uint8_t retry = 3;
	uint8_t device_id = 0, mux_channel = 0;
	I2C_MSG i2c_msg;

	// at least include device_id, addr, rx_len, offset
	if (msg->data_len < 4) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	device_id = msg->data[0];
	if (!get_e1s_present(device_id)) {
		msg->completion_code = CC_SENSOR_NOT_PRESENT;
		return;
	}

	switch (device_id) {
	case E1S_0:
		mux_channel = I2C_HUB_CHANNEL_0;
		break;
	case E1S_1:
		mux_channel = I2C_HUB_CHANNEL_1;
		break;
	case E1S_2:
		mux_channel = I2C_HUB_CHANNEL_2;
		break;
	case E1S_3:
		mux_channel = I2C_HUB_CHANNEL_3;
		break;
	case E1S_4:
		mux_channel = I2C_HUB_CHANNEL_4;
		break;
	default:
		msg->completion_code = CC_INVALID_DATA_FIELD;
		return;
	}

	i2c_msg.bus = I2C_BUS2;
	// 8 bit address to 7 bit
	i2c_msg.target_addr = msg->data[1] >> 1;
	i2c_msg.rx_len = msg->data[2];
	i2c_msg.tx_len = msg->data_len - 3;

	if (i2c_msg.tx_len == 0) {
		msg->completion_code = CC_INVALID_DATA_FIELD;
		return;
	}

	// Lock I3C HUB (I2C mode) before to change channel
	if (k_mutex_lock(&i2c_hub_mutex, K_MSEC(I2C_HUB_MUTEX_TIMEOUT_MS))) {
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		LOG_ERR("Failed to lock Mutex, bus%d addr 0x%x", i2c_msg.bus, i2c_msg.target_addr);
		return;
	}

	// Change channel
	if (!rg3mxxb12_select_slave_port_connect(i2c_msg.bus, mux_channel)) {
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		k_mutex_unlock(&i2c_hub_mutex);
		return;
	}

	memcpy(&i2c_msg.data[0], &msg->data[3], i2c_msg.tx_len);
	msg->data_len = i2c_msg.rx_len;

	if (i2c_msg.rx_len == 0) {
		if (!i2c_master_write(&i2c_msg, retry)) {
			msg->completion_code = CC_SUCCESS;
		} else {
			msg->completion_code = CC_I2C_BUS_ERROR;
		}
	} else {
		if (!i2c_master_read(&i2c_msg, retry)) {
			memcpy(&msg->data[0], &i2c_msg.data, i2c_msg.rx_len);
			msg->completion_code = CC_SUCCESS;
		} else {
			msg->completion_code = CC_I2C_BUS_ERROR;
		}
	}

	// Close all channels after write read
	if (!rg3mxxb12_select_slave_port_connect(i2c_msg.bus, RG3MXXB12_SSPORTS_ALL_DISCONNECT)) {
		LOG_ERR("Failed to close I3C HUB (I2C mode) channel");
	}

	if (k_mutex_unlock(&i2c_hub_mutex)) {
		LOG_ERR("Failed to unlock mutex");
	}

	return;
}
