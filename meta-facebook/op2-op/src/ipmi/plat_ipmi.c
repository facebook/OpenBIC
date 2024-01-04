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
#include "m88rt51632.h"
#include "rg3mxxb12.h"
#include "p3h284x.h"
#include "power_status.h"
#include "plat_class.h"
#include "plat_i2c.h"
#include "plat_isr.h"
#include "plat_ipmi.h"
#include "plat_power_seq.h"
#include "plat_sensor_table.h"
#include "plat_hook.h"
#include "plat_led.h"
#include "util_spi.h"

LOG_MODULE_REGISTER(plat_ipmi);

#define IS_SECTOR_END_MASK 0x80
#define BIC_UPDATE_MAX_OFFSET 0x50000

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
		e1s_power_on_thread(device_index, E1S_POWER_ON_STAGE0);
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
	I2C_MSG *i2c_msg = NULL;
	uint8_t retimer_type;
	uint32_t retimer_version = RETIMER_UNKNOWN_VERSION;

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
			retimer_type = get_pcie_retimer_type();
			i2c_msg = malloc(sizeof(I2C_MSG));
			if (is_retimer_done()) {
				i2c_msg->bus = I2C_BUS4;
				i2c_msg->target_addr = EXPA_RETIMER_ADDR;
				retimer_version = get_pcie_retimer_version();
				switch (retimer_type) {
				case RETIMER_TYPE_PT5161L:
					if (retimer_version == RETIMER_UNKNOWN_VERSION) {
						if (get_retimer_fw_version(i2c_msg, msg->data)) {
							msg->data_len = 4;
							msg->completion_code = CC_SUCCESS;
						} else {
							msg->completion_code = CC_UNSPECIFIED_ERROR;
						}
					} else {
						convert_uint32_t_to_uint8_t_pointer(
							retimer_version, msg->data, 4, BIG_ENDIAN);
						msg->data_len = 4;
						msg->completion_code = CC_SUCCESS;
					}
					break;
				case RETIMER_TYPE_M88RT51632:
					if (retimer_version == RETIMER_UNKNOWN_VERSION) {
						if (m88rt51632_get_fw_version(i2c_msg,
									      &retimer_version)) {
							convert_uint32_t_to_uint8_t_pointer(
								retimer_version, msg->data, 4,
								BIG_ENDIAN);
							msg->data_len = 4;
							msg->completion_code = CC_SUCCESS;
						} else {
							msg->completion_code = CC_UNSPECIFIED_ERROR;
						}
					} else {
						convert_uint32_t_to_uint8_t_pointer(
							retimer_version, msg->data, 4, BIG_ENDIAN);
						msg->data_len = 4;
						msg->completion_code = CC_SUCCESS;
					}
					break;
				default:
					msg->completion_code = CC_UNSPECIFIED_ERROR;
					break;
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

void OEM_1S_FW_UPDATE(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	/*********************************
	* Request Data
	*
	* Byte   0: [6:0] fw update target, [7] indicate last packet
	* Byte 1-4: offset, lsb first
	* Byte 5-6: length, lsb first
	* Byte 7-N: data
	***********************************/
	if (msg->data_len < 8) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t target = msg->data[0];
	uint8_t status = -1;
	uint8_t card_type = get_card_type();
	uint8_t retimer_type;
	uint32_t offset =
		((msg->data[4] << 24) | (msg->data[3] << 16) | (msg->data[2] << 8) | msg->data[1]);
	uint16_t length = ((msg->data[6] << 8) | msg->data[5]);

	if ((length == 0) || (length != msg->data_len - 7)) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	switch (target) {
	case OL2_COMPNT_BIC:
	case (OL2_COMPNT_BIC | IS_SECTOR_END_MASK):
		// Expect BIC firmware size not bigger than 320k
		if (offset > BIC_UPDATE_MAX_OFFSET) {
			msg->completion_code = CC_PARAM_OUT_OF_RANGE;
			return;
		}
		status = fw_update(offset, length, &msg->data[7], (target & IS_SECTOR_END_MASK),
				   DEVSPI_FMC_CS0);
		break;
	case OL2_COMPNT_RETIMER:
	case (OL2_COMPNT_RETIMER | IS_SECTOR_END_MASK):
		if (!get_DC_status()) {
			msg->completion_code = CC_NOT_SUPP_IN_CURR_STATE;
			return;
		}

		if (card_type == CARD_TYPE_OPA) {
			retimer_type = get_pcie_retimer_type();
		} else {
			msg->completion_code = CC_NOT_SUPP_IN_CURR_STATE;
			return;
		}

		if (offset > PCIE_RETIMER_UPDATE_MAX_OFFSET) {
			msg->completion_code = CC_PARAM_OUT_OF_RANGE;
			return;
		}
		I2C_MSG i2c_msg;
		i2c_msg.bus = I2C_BUS4;
		i2c_msg.target_addr = EXPA_RETIMER_ADDR;

		switch (retimer_type) {
		case RETIMER_TYPE_PT5161L:
			status = pcie_retimer_fw_update(&i2c_msg, offset, length, &msg->data[7],
							(target & IS_SECTOR_END_MASK));
			break;
		case RETIMER_TYPE_M88RT51632:
			status = m88rt51632_fw_update(&i2c_msg, offset, length, &msg->data[7],
						      (target & IS_SECTOR_END_MASK));
			break;
		default:
			LOG_ERR("firmware update unknown pcie retimer type");
			break;
		}
		break;
	default:
		msg->completion_code = CC_INVALID_DATA_FIELD;
		return;
	}

	msg->data_len = 0;

	switch (status) {
	case FWUPDATE_SUCCESS:
		msg->completion_code = CC_SUCCESS;
		break;
	case FWUPDATE_OUT_OF_HEAP:
		msg->completion_code = CC_LENGTH_EXCEEDED;
		break;
	case FWUPDATE_OVER_LENGTH:
		msg->completion_code = CC_OUT_OF_SPACE;
		break;
	case FWUPDATE_REPEATED_UPDATED:
		msg->completion_code = CC_INVALID_DATA_FIELD;
		break;
	case FWUPDATE_UPDATE_FAIL:
		msg->completion_code = CC_TIMEOUT;
		break;
	case FWUPDATE_ERROR_OFFSET:
		msg->completion_code = CC_PARAM_OUT_OF_RANGE;
		break;
	case FWUPDATE_NOT_SUPPORT:
		msg->completion_code = CC_INVALID_PARAM;
		break;
	default:
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		break;
	}
	if (status != FWUPDATE_SUCCESS) {
		LOG_ERR("firmware (0x%02X) update failed cc: %x", target, msg->completion_code);
	}

	return;
}

void OEM_1S_PRE_POWER_OFF_CONTROL(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 0) {
		msg->data_len = 0;
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	if (gpio_get(FM_EXP_MAIN_PWR_EN) == GPIO_HIGH) {
		abort_power_thread();
		init_power_off_thread();
		msg->completion_code = CC_SUCCESS;
	} else {
		LOG_ERR("Already power off");
		msg->data_len = 0;
		msg->completion_code = CC_INVALID_LENGTH;
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
	uint8_t card_type = get_card_type();
	uint16_t i3c_hub_type = get_i3c_hub_type();

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
	if (card_type == CARD_TYPE_OPA) {
		if (mux_channel >= sizeof(e1s_mux_channel_opa)) {
			msg->completion_code = CC_INVALID_DATA_FIELD;
			return;
		}
		mux_channel = e1s_mux_channel_opa[device_id];
	} else {
		if (mux_channel >= sizeof(e1s_mux_channel_opb)) {
			msg->completion_code = CC_INVALID_DATA_FIELD;
			return;
		}
		mux_channel = e1s_mux_channel_opb[device_id];
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
	if (i3c_hub_type == RG3M87B12_DEVICE_INFO) {
		if (!rg3mxxb12_select_slave_port_connect(i2c_msg.bus, mux_channel)) {
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			k_mutex_unlock(&i2c_hub_mutex);
			return;
		}
	} else {
		if (!p3h284x_select_slave_port_connect(i2c_msg.bus, mux_channel)) {
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			k_mutex_unlock(&i2c_hub_mutex);
			return;
		}
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
	if (i3c_hub_type == RG3M87B12_DEVICE_INFO) {
		if (!rg3mxxb12_select_slave_port_connect(i2c_msg.bus,
							 RG3MXXB12_SSPORTS_ALL_DISCONNECT)) {
			LOG_ERR("Failed to close I3C HUB (I2C mode) channel");
		}
	} else {
		if (!p3h284x_select_slave_port_connect(i2c_msg.bus,
						       P3H284X_SSPORTS_ALL_DISCONNECT)) {
			LOG_ERR("Failed to close I3C HUB (I2C mode) channel");
		}
	}

	if (k_mutex_unlock(&i2c_hub_mutex)) {
		LOG_ERR("Failed to unlock mutex");
	}

	return;
}

/* Control amber LED behavior
 * Request:
 * Data 0: Device ID
 *   00h E1S 0
 *   01h E1S 1
 *   02h E1S 2
 *   03h E1S 3
 *   04h E1S 4
 * Data 1: Control option
 *   00h LED turn off
 *   01h LED turn on
 *   02h LED start blink
 *   03h LED stop blink
*/
void OEM_1S_SET_SSD_LED(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 2) {
		msg->data_len = 0;
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t device = msg->data[0];

	msg->data_len = 0;

	if (!get_e1s_present(device)) {
		msg->completion_code = CC_SENSOR_NOT_PRESENT;
		return;
	}

	if (control_e1s_amber_led(device, msg->data[1]) < 0) {
		msg->completion_code = CC_UNSPECIFIED_ERROR;
	} else {
		msg->completion_code = CC_SUCCESS;
	}

	return;
}

/* Get amber LED status
 * Request:
 * Data 0: Device ID
 *   00h E1S 0
 *   01h E1S 1
 *   02h E1S 2
 *   03h E1S 3
 *   04h E1S 4
 * Response
 * Data 0: Status
 *   00h LED off
 *   01h LED on
 *   02h LED blink
*/
void OEM_1S_GET_SSD_STATUS(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 1) {
		msg->data_len = 0;
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t device = msg->data[0];

	if (!get_e1s_present(device)) {
		msg->data_len = 0;
		msg->completion_code = CC_SENSOR_NOT_PRESENT;
		return;
	}

	msg->data_len = 1;
	msg->data[0] = get_e1s_amber_led_status(device);
	msg->completion_code = CC_SUCCESS;

	return;
}

void OEM_1S_GET_PCIE_RETIMER_TYPE(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 0) {
		msg->data_len = 0;
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	msg->data_len = 1;
	msg->data[0] = get_pcie_retimer_type();
	msg->completion_code = CC_SUCCESS;

	return;
}
