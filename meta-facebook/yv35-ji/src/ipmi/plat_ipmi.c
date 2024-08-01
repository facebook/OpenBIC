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

#include "plat_ipmi.h"

#include <stdio.h>
#include <stdlib.h>
#include <logging/log.h>
#include "libutil.h"
#include "ipmi.h"
#include "hal_gpio.h"
#include "oem_1s_handler.h"
#include "plat_fru.h"
#include "plat_class.h"

LOG_MODULE_REGISTER(plat_ipmi);

#define WORK_AROUND_BIOS_DEBUG_PIN_IDX 62
#define WORK_AROUND_BIOS_DEBUG_PIN_IDX_REAL 23

#define VIRTUAL_E1S_PRSNT_PIN_IDX 63
#define VIRTUAL_E1S_PRSNT_PIN_IDX_REAL 55

#define VIRTUAL_RETIMER_PRSNT_PIN_IDX 64
#define VIRTUAL_RETIMER_PRSNT_PIN_IDX_REAL 58

bool pal_request_msg_to_BIC_from_HOST(uint8_t netfn, uint8_t cmd)
{
	if (netfn == NETFN_OEM_1S_REQ) {
		if ((cmd == CMD_OEM_1S_FW_UPDATE) || (cmd == CMD_OEM_1S_RESET_BMC) ||
		    (cmd == CMD_OEM_1S_GET_BIC_STATUS) || (cmd == CMD_OEM_1S_RESET_BIC) ||
		    (cmd == CMD_OEM_1S_GET_BIC_FW_INFO || cmd == CMD_OEM_1S_SEND_MCTP_PLDM_COMMAND))
			return true;
	} else if (netfn == NETFN_APP_REQ) {
		if (cmd == CMD_APP_GET_SYSTEM_GUID)
			return true;
	} else if (netfn == NETFN_DCMI_REQ) {
		if ((cmd == CMD_DCMI_SEND_BOOT_PROGRESS_CODE) ||
		    (cmd == CMD_DCMI_GET_BOOT_PROGRESS_CODE))
			return true;
	}

	return false;
}

bool pal_immediate_respond_from_HOST(uint8_t netfn, uint8_t cmd)
{
	if (netfn == NETFN_STORAGE_REQ) {
		if (cmd == CMD_STORAGE_ADD_SEL)
			return true;
	} else if (netfn == NETFN_SENSOR_REQ) {
		if (cmd == CMD_SENSOR_PLATFORM_EVENT)
			return true;
	} else if (netfn == NETFN_OEM_REQ) {
		if (cmd == CMD_OEM_POST_END)
			return true;
	}

	return false;
}

int pal_record_bios_fw_version(uint8_t *buf, uint8_t size)
{
	CHECK_NULL_ARG_WITH_RETURN(buf, -1);

	int ret = -1;
	EEPROM_ENTRY set_bios_ver = { 0 };
	EEPROM_ENTRY get_bios_ver = { 0 };

	const uint8_t block_index = buf[3];
	if (block_index >= BIOS_FW_VERSION_BLOCK_NUM) {
		LOG_ERR("BIOS version block index is out of range");
		return -1;
	}

	ret = get_bios_version(&get_bios_ver, block_index);
	if (ret == -1) {
		LOG_ERR("Get BIOS version failed");
		return -1;
	}

	set_bios_ver.data_len = size - 3; // skip netfn, cmd and command code
	memcpy(&set_bios_ver.data[0], &buf[3], set_bios_ver.data_len);

	// Check the written BIOS version is the same with the stored
	ret = memcmp(&get_bios_ver.data[0], &set_bios_ver.data[0],
		     BIOS_FW_VERSION_BLOCK_MAX_SIZE * sizeof(uint8_t));
	if (ret == 0) {
		LOG_DBG("The Written BIOS version is the same as the stored BIOS version in EEPROM");
	} else {
		LOG_DBG("BIOS version set successfully");

		ret = set_bios_version(&set_bios_ver, block_index);
		if (ret == -1) {
			LOG_ERR("Set BIOS version failed");
			return -1;
		}
	}

	return 0;
}

void OEM_1S_GET_CARD_TYPE(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg)

	if (msg->data_len != 1) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	CARD_STATUS _1ou_status = get_1ou_status();
	CARD_STATUS _2ou_status = get_2ou_status();
	switch (msg->data[0]) {
	case GET_1OU_CARD_TYPE:
		msg->data_len = 2;
		msg->completion_code = CC_SUCCESS;
		msg->data[0] = GET_1OU_CARD_TYPE;
		if (_1ou_status.present) {
			msg->data[1] = _1ou_status.card_type;
		} else {
			msg->data[1] = TYPE_1OU_ABSENT;
		}
		break;
	case GET_2OU_CARD_TYPE:
		msg->data_len = 2;
		msg->completion_code = CC_SUCCESS;
		msg->data[0] = GET_2OU_CARD_TYPE;
		if (_2ou_status.present) {
			msg->data[1] = _2ou_status.card_type;
		} else {
			msg->data[1] = TYPE_2OU_ABSENT;
		}
		break;
	default:
		msg->data_len = 0;
		msg->completion_code = CC_INVALID_DATA_FIELD;
		break;
	}

	return;
}

/* work around - Modify gpio index */
void OEM_1S_GET_GPIO(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	// only input enable status
	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t eight_bit_value = 0, gpio_value, gpio_cnt, data_len;
	// Bump up the gpio_ind_to_num_table_cnt to multiple of 8.
	gpio_cnt = gpio_ind_to_num_table_cnt + (8 - (gpio_ind_to_num_table_cnt % 8));
	data_len = gpio_cnt / 8;
	msg->data_len = data_len;

	int tmp_gpio_idx = 0;
	for (uint8_t i = 0; i < gpio_cnt; i++) {
		if (i == VIRTUAL_RETIMER_PRSNT_PIN_IDX)
			tmp_gpio_idx = VIRTUAL_RETIMER_PRSNT_PIN_IDX_REAL;
		else if (i == VIRTUAL_E1S_PRSNT_PIN_IDX)
			tmp_gpio_idx = VIRTUAL_E1S_PRSNT_PIN_IDX_REAL;
		else if (i == WORK_AROUND_BIOS_DEBUG_PIN_IDX)
			tmp_gpio_idx = WORK_AROUND_BIOS_DEBUG_PIN_IDX_REAL;
		else if (i >= WORK_AROUND_BIOS_DEBUG_PIN_IDX_REAL &&
			 i < (VIRTUAL_E1S_PRSNT_PIN_IDX_REAL - 1))
			tmp_gpio_idx = i + 1;
		else if (i >= (VIRTUAL_E1S_PRSNT_PIN_IDX_REAL - 1) &&
			 i < (VIRTUAL_RETIMER_PRSNT_PIN_IDX_REAL - 2))
			tmp_gpio_idx = i + 2;
		else if (i >= (VIRTUAL_RETIMER_PRSNT_PIN_IDX_REAL - 2) &&
			 i < WORK_AROUND_BIOS_DEBUG_PIN_IDX)
			tmp_gpio_idx = i + 3;
		else
			tmp_gpio_idx = i;

		gpio_value = (tmp_gpio_idx >= gpio_ind_to_num_table_cnt) ?
				     0 :
				     gpio_get(gpio_ind_to_num_table[tmp_gpio_idx]);

		// clear temporary variable to avoid return wrong GPIO value
		if (i % 8 == 0) {
			eight_bit_value = 0;
		}
		eight_bit_value = eight_bit_value | (gpio_value << (i % 8));
		msg->data[i / 8] = eight_bit_value;
	}
	msg->completion_code = CC_SUCCESS;

	return;
}

/* work around - Modify gpio index */
uint8_t gpio_idx_exchange(ipmi_msg *msg)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, 1);

	if (msg->data_len < 2)
		return 1;

	int need_change = 1;
	switch (msg->data[0]) {
	case GET_GPIO_STATUS:
	case GET_GPIO_DIRECTION_STATUS:
		if (msg->data_len == 3) {
			if (msg->data[2] == GLOBAL_GPIO_IDX_KEY) {
				need_change = 0;
			}
			// Ignore last data byte if provided.
			msg->data_len--;
		}
		break;
	case SET_GPIO_OUTPUT_STATUS:
	case SET_GPIO_DIRECTION_STATUS:
		if (msg->data_len == 4) {
			if (msg->data[3] == GLOBAL_GPIO_IDX_KEY) {
				need_change = 0;
			}
			// Ignore last data byte if provided.
			msg->data_len--;
		}
		break;
	default:
		break;
	}

	if (need_change) {
		if (msg->data[1] == VIRTUAL_RETIMER_PRSNT_PIN_IDX)
			msg->data[1] = VIRTUAL_RETIMER_PRSNT_PIN_IDX_REAL;
		else if (msg->data[1] == VIRTUAL_E1S_PRSNT_PIN_IDX)
			msg->data[1] = VIRTUAL_E1S_PRSNT_PIN_IDX_REAL;
		else if (msg->data[1] == WORK_AROUND_BIOS_DEBUG_PIN_IDX)
			msg->data[1] = WORK_AROUND_BIOS_DEBUG_PIN_IDX_REAL;
		else if (msg->data[1] >= WORK_AROUND_BIOS_DEBUG_PIN_IDX_REAL &&
			 msg->data[1] < (VIRTUAL_E1S_PRSNT_PIN_IDX_REAL - 1))
			msg->data[1]++;
		else if (msg->data[1] >= (VIRTUAL_E1S_PRSNT_PIN_IDX_REAL - 1) &&
			 msg->data[1] < (VIRTUAL_RETIMER_PRSNT_PIN_IDX_REAL - 2))
			msg->data[1] += 2;
		else if (msg->data[1] >= (VIRTUAL_RETIMER_PRSNT_PIN_IDX_REAL - 2) &&
			 msg->data[1] < WORK_AROUND_BIOS_DEBUG_PIN_IDX)
			msg->data[1] += 3;

		msg->data[1] = gpio_ind_to_num_table[msg->data[1]];
	}

	return 0;
}
