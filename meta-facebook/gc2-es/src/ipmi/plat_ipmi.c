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

#include "eeprom.h"
#include "fru.h"
#include "hal_gpio.h"
#include "ipmi.h"
#include "libutil.h"
#include "plat_class.h"
#include "plat_fru.h"
#include "plat_ipmb.h"
#include <logging/log.h>
#include <stdio.h>
#include <stdlib.h>

LOG_MODULE_REGISTER(plat_ipmi);

int pal_record_bios_fw_version(uint8_t *buf, uint8_t size)
{
	CHECK_NULL_ARG_WITH_RETURN(buf, -1);

	int ret = -1;
	EEPROM_ENTRY set_bios_ver = { 0 };
	EEPROM_ENTRY get_bios_ver = { 0 };

	const uint8_t block_index = buf[3];
	if (block_index >= BIOS_FW_VERSION_BLOCK_NUM) {
		LOG_ERR("bios version block index is out of range");
		return -1;
	}

	ret = get_bios_version(&get_bios_ver, block_index);
	if (ret == -1) {
		LOG_ERR("Get version fail");
		return -1;
	}

	set_bios_ver.data_len = size - 3; // skip netfn, cmd and command code
	memcpy(&set_bios_ver.data[0], &buf[3], set_bios_ver.data_len);

	// Check the written BIOS version is the same with the stored
	ret = memcmp(&get_bios_ver.data[0], &set_bios_ver.data[0],
		     BIOS_FW_VERSION_BLOCK_MAX_SIZE * sizeof(uint8_t));
	if (ret == 0) {
		LOG_DBG("The Written bios version is the same with the stored bios version in EEPROM");
	} else {
		LOG_DBG("Set bios version");

		ret = set_bios_version(&set_bios_ver, block_index);
		if (ret == -1) {
			LOG_ERR("Set version fail");
			return -1;
		}
	}

	return 0;
}

void OEM_1S_GET_BIOS_VERSION(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	msg->data_len = 0;

	for (uint8_t block_index = 0; block_index < BIOS_FW_VERSION_BLOCK_NUM; block_index++) {
		EEPROM_ENTRY get_bios_ver = { 0 };
		int ret = get_bios_version(&get_bios_ver, block_index);
		if (ret == -1) {
			LOG_ERR("Get version fail");
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			return;
		}

		memcpy(msg->data + msg->data_len, get_bios_ver.data, get_bios_ver.data_len);
		msg->data_len += get_bios_ver.data_len;
	}

	msg->completion_code = CC_SUCCESS;
	return;
}

void OEM_1S_GET_CARD_TYPE(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

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

void OEM_1S_GET_GPIO_CONFIG(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);
	if (msg->data_len == 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t idx = 0;
	uint8_t bitmap_len = msg->data_len;
	uint8_t *gpio_bitmap = (uint8_t *)malloc(bitmap_len * sizeof(uint8_t));
	if (gpio_bitmap == NULL) {
		return;
	}
	memcpy(gpio_bitmap, &msg->data[0], bitmap_len);

	for (uint8_t num = 0; num < gpio_ind_to_num_table_cnt; num++) {
		if (num / BITS_PER_BYTE >= bitmap_len) {
			break;
		}

		uint8_t byte = msg->data[num / BITS_PER_BYTE];
		uint8_t pin_mask = (1 << (num % BITS_PER_BYTE));
		if (!(byte & pin_mask)) {
			continue;
		}

		uint8_t gpio_num = gpio_ind_to_num_table[num];
		uint8_t cfg_byte = 0;

		uint8_t dir = (uint8_t)gpio_get_direction(gpio_num);
		cfg_byte |= (dir & 0x1u) << GPIO_CONF_SET_DIR;

		uint8_t interrupt_en = gpio_get_reg_value(gpio_num, REG_INTERRUPT_ENABLE_OFFSET);
		cfg_byte |= (interrupt_en & 0x1u) << GPIO_CONF_SET_INT;

		uint8_t interrupt_type1 = gpio_get_reg_value(gpio_num, REG_INTERRUPT_TYPE1_OFFSET);
		cfg_byte |= (interrupt_type1 & 0x1u) << GPIO_CONF_SET_TRG_TYPE;

		uint8_t interrupt_type2 = gpio_get_reg_value(gpio_num, REG_INTERRUPT_TYPE2_OFFSET);
		if (interrupt_type2) {
			cfg_byte |= (interrupt_type2 & 0x1u) << GPIO_CONF_SET_TRG_BOTH;
		} else {
			uint8_t interrupt_type0 =
				gpio_get_reg_value(gpio_num, REG_INTERRUPT_TYPE0_OFFSET);
			cfg_byte |= (interrupt_type0 & 0x1u) << GPIO_CONF_SET_TRG_EDGE;
		}

		msg->data[idx++] = cfg_byte;
	}
	free(gpio_bitmap);
	msg->data_len = idx;
	msg->completion_code = CC_SUCCESS;
}

void OEM_1S_SET_GPIO_CONFIG(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);
	uint8_t bitmap_len = (gpio_ind_to_num_table_cnt + BITS_PER_BYTE - 1) / BITS_PER_BYTE;
	if (msg->data_len < bitmap_len + 1) {
		msg->completion_code = CC_INVALID_LENGTH;
		msg->data_len = 0;
		return;
	}

	uint8_t idx = bitmap_len;
	for (uint8_t num = 0; num < gpio_ind_to_num_table_cnt; num++) {
		if (num / BITS_PER_BYTE >= bitmap_len) {
			break;
		}

		uint8_t byte = msg->data[num / BITS_PER_BYTE];
		uint8_t pin_mask = (uint8_t)(1u << (num % BITS_PER_BYTE));
		if (!(byte & pin_mask)) {
			continue;
		}

		uint8_t cfg = msg->data[idx];
		uint8_t gpio_num = gpio_ind_to_num_table[num];

		if (gpio_cfg[gpio_num].is_init == DISABLE) {
			msg->data_len = 0;
			msg->completion_code = CC_INVALID_DATA_FIELD;
			return;
		}

		if (cfg & BIT(GPIO_CONF_SET_DIR)) {
			gpio_conf(gpio_num, GPIO_OUTPUT);
		} else {
			gpio_conf(gpio_num, GPIO_INPUT);
		}

		if (cfg & BIT(GPIO_CONF_SET_INT)) {
			if (cfg & BIT(GPIO_CONF_SET_TRG_BOTH)) {
				gpio_interrupt_conf(gpio_num, GPIO_INT_EDGE_BOTH);
			} else {
				if (cfg & BIT(GPIO_CONF_SET_TRG_TYPE)) {
					// level
					if (cfg & BIT(GPIO_CONF_SET_TRG_EDGE)) {
						gpio_interrupt_conf(gpio_num, GPIO_INT_LEVEL_HIGH);
					} else {
						gpio_interrupt_conf(gpio_num, GPIO_INT_LEVEL_LOW);
					}
				} else {
					// edge
					if (cfg & BIT(GPIO_CONF_SET_TRG_EDGE)) {
						gpio_interrupt_conf(gpio_num, GPIO_INT_EDGE_RISING);
					} else {
						gpio_interrupt_conf(gpio_num,
								    GPIO_INT_EDGE_FALLING);
					}
				}
			}
		} else {
			gpio_interrupt_conf(gpio_num, GPIO_INT_DISABLE);
		}

		idx++;
	}

	msg->data_len = 0;
	msg->completion_code = CC_SUCCESS;
}
