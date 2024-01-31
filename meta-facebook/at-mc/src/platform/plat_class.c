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
#include "plat_pldm_monitor.h"

LOG_MODULE_REGISTER(plat_class);

#define PRESENCE_CHECK_STACK_SIZE 1024
#define PRESENCE_CHECK_DELAY_MS 5000

K_THREAD_STACK_DEFINE(presence_check_thread, PRESENCE_CHECK_STACK_SIZE);
struct k_thread presence_check_thread_handler;
k_tid_t presence_check_tid;

uint8_t board_revision = REV_UNKNOWN;

struct PCIE_CARD_INFO pcie_card_info[] = {
	[0] = { .cpld_offset = 0x20,
		.power_status_offset = 0x11,
		.value_bit = 0x0F,
		.value_shift_bit = 4,
		.card_device_type = UNKNOWN_CARD },
	[1] = { .cpld_offset = 0x20,
		.power_status_offset = 0x12,
		.value_bit = 0x0F,
		.value_shift_bit = 0,
		.card_device_type = UNKNOWN_CARD },
	[2] = { .cpld_offset = 0x21,
		.power_status_offset = 0x13,
		.value_bit = 0x0F,
		.value_shift_bit = 4,
		.card_device_type = UNKNOWN_CARD },
	[3] = { .cpld_offset = 0x21,
		.power_status_offset = 0x14,
		.value_bit = 0x0F,
		.value_shift_bit = 0,
		.card_device_type = UNKNOWN_CARD },
	[4] = { .cpld_offset = 0x22,
		.power_status_offset = 0x15,
		.value_bit = 0x01,
		.value_shift_bit = 3,
		.card_device_type = UNKNOWN_CARD },
	[5] = { .cpld_offset = 0x22,
		.power_status_offset = 0x16,
		.value_bit = 0x01,
		.value_shift_bit = 2,
		.card_device_type = UNKNOWN_CARD },
	[6] = { .cpld_offset = 0x22,
		.power_status_offset = 0x17,
		.value_bit = 0x01,
		.value_shift_bit = 1,
		.card_device_type = UNKNOWN_CARD },
	[7] = { .cpld_offset = 0x22,
		.power_status_offset = 0x18,
		.value_bit = 0x01,
		.value_shift_bit = 0,
		.card_device_type = UNKNOWN_CARD },
	[8] = { .cpld_offset = 0x23,
		.power_status_offset = 0x19,
		.value_bit = 0x0F,
		.value_shift_bit = 4,
		.card_device_type = UNKNOWN_CARD },
	[9] = { .cpld_offset = 0x23,
		.power_status_offset = 0x1A,
		.value_bit = 0x0F,
		.value_shift_bit = 0,
		.card_device_type = UNKNOWN_CARD },
	[10] = { .cpld_offset = 0x24,
		 .power_status_offset = 0x1B,
		 .value_bit = 0x0F,
		 .value_shift_bit = 4,
		 .card_device_type = UNKNOWN_CARD },
	[11] = { .cpld_offset = 0x24,
		 .power_status_offset = 0x1C,
		 .value_bit = 0x0F,
		 .value_shift_bit = 0,
		 .card_device_type = UNKNOWN_CARD },
	[12] = { .cpld_offset = 0x25,
		 .power_status_offset = 0x1D,
		 .value_bit = 0x07,
		 .value_shift_bit = 3,
		 .card_device_type = UNKNOWN_CARD },
	[13] = { .cpld_offset = 0x25,
		 .power_status_offset = 0x1E,
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

int check_pcie_card_presence_status(uint8_t card_id, uint8_t *card_type)
{
	CHECK_NULL_ARG_WITH_RETURN(card_type, -1);

	if (card_id >= ARRAY_SIZE(pcie_card_info)) {
		LOG_ERR("Fail to get pcie card presence status because of invalid card id: 0x%x",
			card_id);
		return -1;
	}

	int ret = -1;
	uint8_t val = 0;
	uint8_t retry = 3;
	I2C_MSG i2c_msg = { 0 };

	i2c_msg.bus = CPLD_BUS;
	i2c_msg.target_addr = CPLD_ADDR;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 1;
	i2c_msg.data[0] = pcie_card_info[card_id].cpld_offset;

	ret = i2c_master_read(&i2c_msg, retry);
	if (ret != 0) {
		LOG_ERR("Fail to get pcid card info, card id: 0x%x, offset: 0x%x", card_id,
			pcie_card_info[card_id].cpld_offset);
		return ret;
	}

	val = ((i2c_msg.data[0]) >> pcie_card_info[card_id].value_shift_bit) &
	      pcie_card_info[card_id].value_bit;

	if ((card_id == CARD_13_INDEX) || (card_id == CARD_14_INDEX)) {
		/* CPLD register reads back the presence value of PCIE card 13/14 doesn't include present_1 bit */
		/* Add present_1 bit value to map card type from presence status */
		val = (val << 1) + 1;
	}

	*card_type = prsnt_status_to_card_type(val);
	return 0;
}

void check_pcie_card_type()
{
	int ret = 0;
	int index = 0;

	for (index = 0; index < ARRAY_SIZE(pcie_card_info); ++index) {
		ret = check_pcie_card_presence_status(index,
						      &pcie_card_info[index].card_device_type);
		if (ret != 0) {
			LOG_ERR("Fail to check pcie card type");
			continue;
		}
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
	return board_revision;
}

int get_pcie_card_power_status(uint8_t pcie_card_id)
{
	if (pcie_card_id > CARD_14_INDEX) {
		return -1;
	}

	uint8_t retry = 3;
	uint8_t power_status_offset = pcie_card_info[pcie_card_id].power_status_offset;
	I2C_MSG i2c_msg = { 0 };

	i2c_msg.bus = CPLD_BUS;
	i2c_msg.target_addr = CPLD_ADDR;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 1;
	i2c_msg.data[0] = power_status_offset;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Get PCIE card power status fail, pcie card id: 0x%x", pcie_card_id);
		return -1;
	}

	return i2c_msg.data[0];
}

int init_platform_config()
{
	init_board_rev_gpio();

	// Need dymic loading GPIO table, using aspeed GPIO API to get GPIO value
	const struct device *gpio_dev;
	gpio_dev = device_get_binding("GPIO0_M_P");

	board_revision = gpio_pin_get(gpio_dev, (REV_ID2 % GPIO_GROUP_SIZE));
	board_revision |= gpio_pin_get(gpio_dev, (REV_ID1 % GPIO_GROUP_SIZE)) << 1;
	board_revision |= gpio_pin_get(gpio_dev, (REV_ID0 % GPIO_GROUP_SIZE)) << 2;

	return 0;
}

void set_reset_smb4_mux_pin()
{
	if (gpio_get(MEB_NORMAL_PWRGD_BIC)) {
		gpio_set(RST_SMB_4_MUX_N, GPIO_HIGH);
	} else {
		gpio_set(RST_SMB_4_MUX_N, GPIO_LOW);
	}
}

void presence_check_handler()
{
	int ret = 0;
	uint8_t index = 0;
	uint8_t card_type = 0;

	while (1) {
		for (index = CARD_8_INDEX; index >= CARD_5_INDEX; index--) {
			ret = check_pcie_card_presence_status(index, &card_type);
			if (ret != 0) {
				LOG_ERR("Fail to get pcie card presence status on presence_check_handler");
				continue;
			}

			if (card_type != pcie_card_info[index].card_device_type) {
				if (card_type == CARD_NOT_PRESENT) {
					plat_send_ssd_present_event(CARD_8_INDEX - index);
				}
			}

			pcie_card_info[index].card_device_type = card_type;
		}

		k_msleep(PRESENCE_CHECK_DELAY_MS);
	}
}

void init_accl_presence_check_work()
{
	if (presence_check_tid != NULL &&
	    (strcmp(k_thread_state_str(presence_check_tid), "dead") != 0) &&
	    (strcmp(k_thread_state_str(presence_check_tid), "unknown") != 0)) {
		return;
	}
	presence_check_tid = k_thread_create(&presence_check_thread_handler, presence_check_thread,
					     K_THREAD_STACK_SIZEOF(presence_check_thread),
					     presence_check_handler, NULL, NULL, NULL,
					     CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&presence_check_thread_handler, "presence_check_thread");
}
