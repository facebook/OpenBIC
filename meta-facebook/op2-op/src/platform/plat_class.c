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
#include "libutil.h"
#include "hal_gpio.h"
#include "plat_class.h"
#include "plat_gpio.h"
#include "plat_sensor_table.h"
#include "plat_i2c.h"
#include "pt5161l.h"
#include "rg3mxxb12.h"
#include "p3h284x.h"
#include "m88rt51632.h"

LOG_MODULE_REGISTER(plat_class);

static uint8_t card_position = CARD_POSITION_UNKNOWN;
static uint8_t card_type = CARD_TYPE_UNKNOWN;
static uint8_t pcie_retimer_type = RETIMER_TYPE_UNKNOWN;
static uint8_t board_revision = UNKNOWN_STAGE;
static uint16_t i3c_hub_type = I3C_HUB_TYPE_UNKNOWN;
static uint32_t pcie_retimer_version = RETIMER_UNKNOWN_VERSION;

uint8_t get_card_position()
{
	return card_position;
}

uint8_t get_card_type()
{
	return card_type;
}

uint16_t get_i3c_hub_type()
{
	return i3c_hub_type;
}

uint8_t get_pcie_retimer_type(void)
{
	return pcie_retimer_type;
}

uint8_t get_board_revision()
{
	return board_revision;
}

uint32_t get_pcie_retimer_version()
{
	return pcie_retimer_version;
}

int init_platform_config()
{
	init_card_position_gpio();

	// Need dymic loading GPIO table, using aspeed GPIO API to get GPIO value
	const struct device *gpio_dev;
	gpio_dev = device_get_binding("GPIO0_M_P");

	char *card_type_name[] = { "A", "B", "UNKNOWN" };
	uint8_t board_id = gpio_pin_get(gpio_dev, (BIC_BOARD_ID % GPIO_GROUP_SIZE));
	uint8_t exp_id = gpio_pin_get(gpio_dev, (BIC_EXP_ID % GPIO_GROUP_SIZE));

	/* BOARD ID high, EXP ID low:  1OU
	 * BOARD ID high, EXP ID high: 3OU
	 * BOARD ID low,  EXP ID low:  2OU
	 * BOARD ID low,  EXP ID high: 4OU
	 */
	if (board_id == GPIO_HIGH) {
		card_type = CARD_TYPE_OPA;

		if (exp_id == GPIO_LOW) {
			card_position = CARD_POSITION_1OU;
		} else {
			card_position = CARD_POSITION_3OU;
		}

	} else {
		card_type = CARD_TYPE_OPB;

		if (exp_id == GPIO_LOW) {
			card_position = CARD_POSITION_2OU;
		} else {
			card_position = CARD_POSITION_4OU;
		}
	}

	LOG_INF("OP %s BIC position: %dOU", card_type_name[card_type], card_position + 1);
	return 1;
}

int check_pcie_retimer_type(void)
{
	uint8_t ret = 0;
	I2C_MSG msg = { 0 };
	msg.bus = I2C_BUS4;
	msg.target_addr = EXPA_RETIMER_ADDR;

	if (!pt5161l_get_vendor_id(&msg)) {
		LOG_ERR("PCIE RETIMER get vendor id fail");
		return -1;
	}

	if (memcmp(msg.data, PT5161L_VENDOR_ID, PT5161L_VENDOR_ID_LENGTH) == 0) {
		LOG_INF("PCIE RETIMER type: PT5161L");
		pcie_retimer_type = RETIMER_TYPE_PT5161L;
		ret = RETIMER_TYPE_PT5161L;
	} else {
		LOG_INF("PCIE RETIMER type: M88RT51632");
		pcie_retimer_type = RETIMER_TYPE_M88RT51632;
		ret = RETIMER_TYPE_M88RT51632;
	}

	return ret;
}

void cache_pcie_retimer_version(void)
{
	uint8_t retimer_type = get_pcie_retimer_type();
	I2C_MSG *i2c_msg = NULL;
	i2c_msg = malloc(sizeof(I2C_MSG));
	i2c_msg->bus = I2C_BUS4;
	i2c_msg->target_addr = EXPA_RETIMER_ADDR;
	uint8_t data[RETIMER_VERSION_MAX_LENGTH];
	memset(data, 0, RETIMER_VERSION_MAX_LENGTH);

	switch (retimer_type) {
	case RETIMER_TYPE_PT5161L:
		if (get_retimer_fw_version(i2c_msg, data)) {
			convert_uint8_t_pointer_to_uint32_t(&pcie_retimer_version, data, 4,
							    BIG_ENDIAN);
		} else {
			LOG_ERR("PCIE RETIMER get version fail");
		}
		break;
	case RETIMER_TYPE_M88RT51632:
		if (!m88rt51632_get_fw_version(i2c_msg, &pcie_retimer_version)) {
			LOG_ERR("PCIE RETIMER get version fail");
		}
		break;
	default:
		LOG_ERR("Unknown PCIE RETIMER");
		break;
	}
	SAFE_FREE(i2c_msg);
}

void init_board_revision(void)
{
	if (card_type == CARD_TYPE_OPA) {
		board_revision = gpio_get(OPA_BOARD_REV_0);
		board_revision |= gpio_get(OPA_BOARD_REV_1) << 1;
		board_revision |= gpio_get(OPA_BOARD_REV_2) << 2;
	} else {
		board_revision = gpio_get(OPB_BOARD_REV_0);
		board_revision |= gpio_get(OPB_BOARD_REV_1) << 1;
		board_revision |= gpio_get(OPB_BOARD_REV_2) << 2;
	}
}

void init_i3c_hub_type(void)
{
	if (rg3mxxb12_get_device_info(I2C_BUS2, &i3c_hub_type) &&
	    (i3c_hub_type == RG3M87B12_DEVICE_INFO)) {
		LOG_INF("I3C hub type: rg3mxxb12");
	} else if (p3h284x_get_device_info(I2C_BUS2, &i3c_hub_type)) {
		LOG_INF("I3C hub type: p3h284x");
	} else {
		LOG_ERR("I3C hub get device type fail");
		return;
	}
}
