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
#include <string.h>
#include <logging/log.h>

#include "libutil.h"
#include "hal_i2c.h"
#include "hal_gpio.h"
#include "plat_gpio.h"
#include "plat_i2c.h"
#include "plat_class.h"
#include "plat_sensor_table.h"

LOG_MODULE_REGISTER(plat_class);

struct PCIE_CARD_INFO {
	uint8_t cpld_offset;
	uint8_t value_bit;
	uint8_t value_shift_bit;
	uint8_t card_device_type;
};

struct PCIE_CARD_INFO pcie_card_info[] = {
	[0] = { .cpld_offset = 0x20,
		.value_bit = 0x0F,
		.value_shift_bit = 4,
		.card_device_type = UNKNOWN_CARD },
	[1] = { .cpld_offset = 0x20,
		.value_bit = 0x0F,
		.value_shift_bit = 0,
		.card_device_type = UNKNOWN_CARD },
	[2] = { .cpld_offset = 0x21,
		.value_bit = 0x0F,
		.value_shift_bit = 4,
		.card_device_type = UNKNOWN_CARD },
	[3] = { .cpld_offset = 0x21,
		.value_bit = 0x0F,
		.value_shift_bit = 0,
		.card_device_type = UNKNOWN_CARD },
	[4] = { .cpld_offset = 0x22,
		.value_bit = 0x01,
		.value_shift_bit = 3,
		.card_device_type = UNKNOWN_CARD },
	[5] = { .cpld_offset = 0x22,
		.value_bit = 0x01,
		.value_shift_bit = 2,
		.card_device_type = UNKNOWN_CARD },
	[6] = { .cpld_offset = 0x22,
		.value_bit = 0x01,
		.value_shift_bit = 1,
		.card_device_type = UNKNOWN_CARD },
	[7] = { .cpld_offset = 0x22,
		.value_bit = 0x01,
		.value_shift_bit = 0,
		.card_device_type = UNKNOWN_CARD },
	[8] = { .cpld_offset = 0x23,
		.value_bit = 0x0F,
		.value_shift_bit = 4,
		.card_device_type = UNKNOWN_CARD },
	[9] = { .cpld_offset = 0x23,
		.value_bit = 0x0F,
		.value_shift_bit = 0,
		.card_device_type = UNKNOWN_CARD },
	[10] = { .cpld_offset = 0x24,
		 .value_bit = 0x0F,
		 .value_shift_bit = 4,
		 .card_device_type = UNKNOWN_CARD },
	[11] = { .cpld_offset = 0x24,
		 .value_bit = 0x0F,
		 .value_shift_bit = 0,
		 .card_device_type = UNKNOWN_CARD },
	[12] = { .cpld_offset = 0x25,
		 .value_bit = 0x07,
		 .value_shift_bit = 3,
		 .card_device_type = UNKNOWN_CARD },
	[13] = { .cpld_offset = 0x25,
		 .value_bit = 0x07,
		 .value_shift_bit = 0,
		 .card_device_type = UNKNOWN_CARD },
};

int pcie_card_id_to_cxl_id(uint8_t pcie_card_id, uint8_t *cxl_id)
{
	CHECK_NULL_ARG_WITH_RETURN(cxl_id, -1);

	switch (pcie_card_id) {
	case CARD_1_INDEX:
		*cxl_id = CXL_CARD_8;
		break;
	case CARD_2_INDEX:
		*cxl_id = CXL_CARD_7;
		break;
	case CARD_3_INDEX:
		*cxl_id = CXL_CARD_6;
		break;
	case CARD_4_INDEX:
		*cxl_id = CXL_CARD_5;
		break;
	case CARD_9_INDEX:
		*cxl_id = CXL_CARD_3;
		break;
	case CARD_10_INDEX:
		*cxl_id = CXL_CARD_4;
		break;
	case CARD_11_INDEX:
		*cxl_id = CXL_CARD_1;
		break;
	case CARD_12_INDEX:
		*cxl_id = CXL_CARD_2;
		break;
	default:
		LOG_ERR("Invalid pcie_card_id: %d", pcie_card_id);
		return -1;
	}

	return 0;
}

int cxl_id_to_pcie_card_id(uint8_t cxl_id, uint8_t *pcie_card_id)
{
	CHECK_NULL_ARG_WITH_RETURN(pcie_card_id, -1);

	switch (cxl_id) {
	case CXL_CARD_1:
		*pcie_card_id = CARD_11_INDEX;
		break;
	case CXL_CARD_2:
		*pcie_card_id = CARD_12_INDEX;
		break;
	case CXL_CARD_3:
		*pcie_card_id = CARD_9_INDEX;
		break;
	case CXL_CARD_4:
		*pcie_card_id = CARD_10_INDEX;
		break;
	case CXL_CARD_5:
		*pcie_card_id = CARD_4_INDEX;
		break;
	case CXL_CARD_6:
		*pcie_card_id = CARD_3_INDEX;
		break;
	case CXL_CARD_7:
		*pcie_card_id = CARD_2_INDEX;
		break;
	case CXL_CARD_8:
		*pcie_card_id = CARD_1_INDEX;
		break;
	default:
		LOG_ERR("Invalid cxl id: %d", cxl_id);
		return -1;
	}

	return 0;
}

void check_pcie_card_type()
{
	int index = 0;
	uint8_t val = 0;
	uint8_t retry = 3;
	I2C_MSG i2c_msg = { 0 };

	for (index = 0; index < ARRAY_SIZE(pcie_card_info); ++index) {
		/* Workaround: Skip to check JCN 13 and 14 card type */
		if (index == CARD_13_INDEX || index == CARD_14_INDEX) {
			pcie_card_info[index].card_device_type = CARD_NOT_PRESENT;
			continue;
		}

		memset(&i2c_msg, 0, sizeof(I2C_MSG));
		i2c_msg.bus = CPLD_BUS;
		i2c_msg.target_addr = CPLD_ADDR;
		i2c_msg.tx_len = 1;
		i2c_msg.rx_len = 1;
		i2c_msg.data[0] = pcie_card_info[index].cpld_offset;

		if (i2c_master_read(&i2c_msg, retry)) {
			LOG_ERR("Initial pcie card %d info fail", index);
			continue;
		}

		val = ((i2c_msg.data[0]) >> pcie_card_info[index].value_shift_bit) &
		      pcie_card_info[index].value_bit;

		pcie_card_info[index].card_device_type = prsnt_status_to_card_type(val);
	}
}

uint8_t prsnt_status_to_card_type(uint8_t presence_status)
{
	switch (presence_status) {
	case E1S_PRESENT:
		return E1S_CARD;

	case E1S_NOT_PRESENT:
		return CARD_NOT_PRESENT;

	case E1S_0_PRESENT:
		return E1S_0_CARD;

	case E1S_1_PRESENT:
		return E1S_1_CARD;

	case E1S_0_1_PRESENT:
		return E1S_0_1_CARD;

	case NIC_PRESENT:
		return NIC_CARD;

	case CXL_PRESENT:
		return CXL_CARD;

	case NO_DEVICE_PRESENT:
		return CARD_NOT_PRESENT;

	default:
		return UNKNOWN_CARD;
	}
}

int get_pcie_card_type(uint8_t card_id, uint8_t *card_type)
{
	CHECK_NULL_ARG_WITH_RETURN(card_type, -1);

	if (card_id >= ARRAY_SIZE(pcie_card_info)) {
		LOG_ERR("Get card type fail, card_id: %d", card_id);
		return -1;
	}

	*card_type = pcie_card_info[card_id].card_device_type;
	return 0;
}

int get_pcie_device_type(uint8_t card_id, uint8_t device_id, uint8_t *device_type)
{
	CHECK_NULL_ARG_WITH_RETURN(device_type, -1);

	uint8_t dev_type = pcie_card_info[card_id].card_device_type;
	*device_type = CARD_NOT_PRESENT;

	switch (card_id) {
	case CARD_1_INDEX:
	case CARD_2_INDEX:
	case CARD_3_INDEX:
	case CARD_4_INDEX:
	case CARD_9_INDEX:
	case CARD_10_INDEX:
	case CARD_11_INDEX:
	case CARD_12_INDEX:
	case CARD_13_INDEX:
	case CARD_14_INDEX:
		switch (device_id) {
		case PCIE_DEVICE_ID1:
			if (dev_type != CARD_NOT_PRESENT) {
				*device_type = dev_type;
			}
			break;
		case PCIE_DEVICE_ID2:
			if ((dev_type == E1S_0_CARD) || (dev_type == E1S_0_1_CARD)) {
				*device_type = dev_type;
			}
			break;
		case PCIE_DEVICE_ID3:
			if ((dev_type == E1S_1_CARD) || (dev_type == E1S_0_1_CARD)) {
				*device_type = dev_type;
			}
			break;
		default:
			LOG_ERR("Invalid pcie device id: 0x%x", device_id);
			return -1;
		}
		break;
	case CARD_5_INDEX:
	case CARD_6_INDEX:
	case CARD_7_INDEX:
	case CARD_8_INDEX:
		switch (device_id) {
		case PCIE_DEVICE_ID1:
			if (dev_type == E1S_PRESENT) {
				*device_type = pcie_card_info[card_id].card_device_type;
			}
			break;
		default:
			*device_type = CARD_NOT_PRESENT;
		}
		break;
	default:
		LOG_ERR("Invalid pcie card id: 0x%x", card_id);
		return -1;
	}

	return 0;
}

bool is_cxl_present()
{
	int ret = -1;
	uint8_t index = 0;
	uint8_t card_type = 0;

	for (index = 0; index < CARD_12_INDEX; ++index) {
		ret = get_pcie_card_type(index, &card_type);
		if (ret < 0) {
			continue;
		}

		if (card_type == CXL_CARD) {
			return true;
		}
	}

	return false;
}

uint8_t get_board_revision()
{
	return (gpio_get(REV_ID0) << 2) | (gpio_get(REV_ID1) << 1) | gpio_get(REV_ID2);
}
