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

#include <logging/log.h>
#include "libutil.h"
#include "hal_gpio.h"
#include "plat_class.h"
#include "plat_gpio.h"
#include "plat_sensor_table.h"
#include "plat_i2c.h"
#include "pt5161l.h"

LOG_MODULE_REGISTER(plat_class);

static uint8_t card_position = CARD_POSITION_UNKNOWN;
static uint8_t card_type = CARD_TYPE_UNKNOWN;
static uint8_t pcie_retimer_type = RETIMER_TYPE_UNKNOWN;

uint8_t get_card_position()
{
	return card_position;
}

uint8_t get_card_type()
{
	return card_type;
}

uint8_t get_pcie_retimer_type(void)
{
	return pcie_retimer_type;
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
