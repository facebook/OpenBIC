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
#include "plat_fru.h"
#include "plat_class.h"
#include "common_i2c_mux.h"
#include "pex89000.h"
#include "plat_sensor_table.h"
#include "hal_gpio.h"
#include "plat_gpio.h"
#include "ioexp_pca9555.h"

LOG_MODULE_REGISTER(plat_class);

#define NUMBER_OF_ADC_CHANNEL 16
#define AST1030_ADC_BASE_ADDR 0x7e6e9000

static uint8_t board_revision = UNKNOWN_STAGE;
static uint8_t hsc_module = HSC_MODULE_UNKNOWN;
static uint8_t pwr_brick_module = POWER_BRICK_UNKNOWN;
static bool is_power_good = false;

struct ASIC_CARD_INFO asic_card_info[ASIC_CARD_COUNT] = {
  [0] = { .bus = I2C_BUS8,
    .mux_addr = ASIC_CARD_1_6_MUX_ADDR,
    .mux_channel = PCA9548A_CHANNEL_5,
    .card_status = ASIC_CARD_UNKNOWN_STATUS,
    .device_mux_addr = ASIC_CARD_DEVICE_MUX_ADDR,
    .device_channel = PCA9546A_CHANNEL_0,
    .asic_1_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS,
    .asic_2_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS, },
  [1] = { .bus = I2C_BUS8,
    .mux_addr = ASIC_CARD_1_6_MUX_ADDR,
    .mux_channel = PCA9548A_CHANNEL_4,
    .card_status = ASIC_CARD_UNKNOWN_STATUS,
    .device_mux_addr = ASIC_CARD_DEVICE_MUX_ADDR,
    .device_channel = PCA9546A_CHANNEL_0,
    .asic_1_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS,
    .asic_2_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS, },
  [2] = { .bus = I2C_BUS8,
    .mux_addr = ASIC_CARD_1_6_MUX_ADDR,
    .mux_channel = PCA9548A_CHANNEL_3,
    .card_status = ASIC_CARD_UNKNOWN_STATUS,
    .device_mux_addr = ASIC_CARD_DEVICE_MUX_ADDR,
    .device_channel = PCA9546A_CHANNEL_0,
    .asic_1_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS,
    .asic_2_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS, },
  [3] = { .bus = I2C_BUS8,
    .mux_addr = ASIC_CARD_1_6_MUX_ADDR,
    .mux_channel = PCA9548A_CHANNEL_2,
    .card_status = ASIC_CARD_UNKNOWN_STATUS,
    .device_mux_addr = ASIC_CARD_DEVICE_MUX_ADDR,
    .device_channel = PCA9546A_CHANNEL_0,
    .asic_1_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS,
    .asic_2_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS, },
  [4] = { .bus = I2C_BUS8,
    .mux_addr = ASIC_CARD_1_6_MUX_ADDR,
    .mux_channel = PCA9548A_CHANNEL_1,
    .card_status = ASIC_CARD_UNKNOWN_STATUS,
    .device_mux_addr = ASIC_CARD_DEVICE_MUX_ADDR,
    .device_channel = PCA9546A_CHANNEL_0,
    .asic_1_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS,
    .asic_2_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS, },
  [5] = { .bus = I2C_BUS8,
    .mux_addr = ASIC_CARD_1_6_MUX_ADDR,
    .mux_channel = PCA9548A_CHANNEL_0,
    .card_status = ASIC_CARD_UNKNOWN_STATUS,
    .device_mux_addr = ASIC_CARD_DEVICE_MUX_ADDR,
    .device_channel = PCA9546A_CHANNEL_0,
    .asic_1_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS,
    .asic_2_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS, },
  [6] = { .bus = I2C_BUS7,
    .mux_addr = ASIC_CARD_7_12_MUX_ADDR,
    .mux_channel = PCA9548A_CHANNEL_5,
    .card_status = ASIC_CARD_UNKNOWN_STATUS,
    .device_mux_addr = ASIC_CARD_DEVICE_MUX_ADDR,
    .device_channel = PCA9546A_CHANNEL_0,
    .asic_1_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS,
    .asic_2_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS, },
  [7] = { .bus = I2C_BUS7,
    .mux_addr = ASIC_CARD_7_12_MUX_ADDR,
    .mux_channel = PCA9548A_CHANNEL_4,
    .card_status = ASIC_CARD_UNKNOWN_STATUS,
    .device_mux_addr = ASIC_CARD_DEVICE_MUX_ADDR,
    .device_channel = PCA9546A_CHANNEL_0,
    .asic_1_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS,
    .asic_2_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS, },
  [8] = { .bus = I2C_BUS7,
    .mux_addr = ASIC_CARD_7_12_MUX_ADDR,
    .mux_channel = PCA9548A_CHANNEL_3,
    .card_status = ASIC_CARD_UNKNOWN_STATUS,
    .device_mux_addr = ASIC_CARD_DEVICE_MUX_ADDR,
    .device_channel = PCA9546A_CHANNEL_0,
    .asic_1_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS,
    .asic_2_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS, },
  [9] = { .bus = I2C_BUS7,
    .mux_addr = ASIC_CARD_7_12_MUX_ADDR,
    .mux_channel = PCA9548A_CHANNEL_2,
    .card_status = ASIC_CARD_UNKNOWN_STATUS,
    .device_mux_addr = ASIC_CARD_DEVICE_MUX_ADDR,
    .device_channel = PCA9546A_CHANNEL_0,
    .asic_1_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS,
    .asic_2_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS, },
  [10] = { .bus = I2C_BUS7,
     .mux_addr = ASIC_CARD_7_12_MUX_ADDR,
     .mux_channel = PCA9548A_CHANNEL_1,
     .card_status = ASIC_CARD_UNKNOWN_STATUS,
     .device_mux_addr = ASIC_CARD_DEVICE_MUX_ADDR,
     .device_channel = PCA9546A_CHANNEL_0,
     .asic_1_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS,
     .asic_2_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS, },
  [11] = { .bus = I2C_BUS7,
     .mux_addr = ASIC_CARD_7_12_MUX_ADDR,
     .mux_channel = PCA9548A_CHANNEL_0,
     .card_status = ASIC_CARD_UNKNOWN_STATUS,
     .device_mux_addr = ASIC_CARD_DEVICE_MUX_ADDR,
     .device_channel = PCA9546A_CHANNEL_0,
     .asic_1_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS,
     .asic_2_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS, },
};

uint8_t reverse_ioexp_val(uint8_t val)
{
	uint8_t index = 0;
	uint8_t reverse_val = 0;
	uint8_t first_four = 0;
	uint8_t last_four = 0;

	for (index = 0; index < 8; ++index) {
		reverse_val = reverse_val << 1;
		if ((val & 1) != 0) {
			reverse_val += 1;
		}
		val = val >> 1;
	}
	first_four = (reverse_val & 0xF0) >> 4;
	last_four = (reverse_val & 0x0F) << 4;
	return (first_four | last_four);
}

void check_accl_device_presence_status_via_ioexp()
{
	int ret = -1;
	int retry = 5;
	uint8_t addr_index = 0;
	uint8_t card_index = 0;
	uint8_t ioexp_index = 0;
	uint8_t presence_val = 0;
	uint16_t reg_val = 0;
	uint8_t ioexp_addr[] = { IOEXP_U228_ADDR, IOEXP_U229_ADDR, IOEXP_U230_ADDR };
	I2C_MSG msg = { 0 };

	for (addr_index = 0; addr_index < ARRAY_SIZE(ioexp_addr); ++addr_index) {
		card_index = 11 - (addr_index * IOEXP_CARD_PRESENCE_COUNT);

		memset(&msg, 0, sizeof(I2C_MSG));
		msg.bus = I2C_BUS13;
		msg.target_addr = ioexp_addr[addr_index];
		msg.rx_len = 1;
		msg.tx_len = 1;
		msg.data[0] = PCA9555_INPUT_PORT_REG_1;

		ret = i2c_master_read(&msg, retry);
		if (ret != 0) {
			LOG_ERR("Fail to read ioexp addr: 0x%x, offset: 0x%x", msg.target_addr,
				msg.data[0]);
			continue;
		}

		// Reverse IOexp bit value in IOexp input register 1
		reg_val = (reverse_ioexp_val(msg.data[0]) << 8);

		memset(&msg, 0, sizeof(I2C_MSG));
		msg.bus = I2C_BUS13;
		msg.target_addr = ioexp_addr[addr_index];
		msg.rx_len = 1;
		msg.tx_len = 1;
		msg.data[0] = PCA9555_INPUT_PORT_REG_0;

		ret = i2c_master_read(&msg, retry);
		if (ret != 0) {
			LOG_ERR("Fail to read ioexp addr: 0x%x, offset: 0x%x", msg.target_addr,
				msg.data[0]);
			continue;
		}

		reg_val |= msg.data[0];

		for (ioexp_index = 0; ioexp_index < IOEXP_CARD_PRESENCE_COUNT; ++ioexp_index) {
			presence_val = (reg_val >> ((IOEXP_CARD_PRESENCE_COUNT - 1 - ioexp_index) *
						    IOEXP_CARD_PRESENCE_PIN_COUNT)) &
				       IOEXP_CARD_PRESENCE_MAP_VAL;

			if ((presence_val & IOEXP_CARD_PRESENT_VAL) == LOW_ACTIVE) {
				asic_card_info[card_index].card_status = ASIC_CARD_PRESENT;
			} else {
				asic_card_info[card_index].card_status = ASIC_CARD_NOT_PRESENT;
			}

			if ((presence_val & IOEXP_DEV_1_PRESENT_VAL) == LOW_ACTIVE) {
				asic_card_info[card_index].asic_1_status = ASIC_CARD_DEVICE_PRESENT;
			} else {
				asic_card_info[card_index].asic_1_status =
					ASIC_CARD_DEVICE_NOT_PRESENT;
			}

			if ((presence_val & IOEXP_DEV_2_PRESENT_VAL) == LOW_ACTIVE) {
				asic_card_info[card_index].asic_2_status = ASIC_CARD_DEVICE_PRESENT;
			} else {
				asic_card_info[card_index].asic_2_status =
					ASIC_CARD_DEVICE_NOT_PRESENT;
			}

			card_index -= 1;
		}
	}
}

void init_platform_config()
{
	board_revision = gpio_get(REV_ID0);
	board_revision |= gpio_get(REV_ID1) << 1;
	board_revision |= gpio_get(REV_ID2) << 2;

	if (gpio_get(HSC_MODULE_PIN_NUM)) {
		hsc_module = HSC_MODULE_LTC4286;
	} else {
		hsc_module = HSC_MODULE_ADM1272;
	}

	if (gpio_get(POWER_BRICK_MODULE_PIN_NUM)) {
		pwr_brick_module = POWER_BRICK_BMR3512202;
	} else {
		pwr_brick_module = POWER_BRICK_Q50SN120A1;
	}
}

uint8_t get_board_revision()
{
	return board_revision;
}

uint8_t get_hsc_module()
{
	return hsc_module;
}

uint8_t get_pwr_brick_module()
{
	return pwr_brick_module;
}

bool get_acb_power_status()
{
	int ret = -1;
	int retry = 5;
	I2C_MSG msg = { 0 };

	msg.bus = I2C_BUS3;
	msg.target_addr = CPLD_ADDR;
	msg.rx_len = 1;
	msg.tx_len = 1;
	msg.data[0] = CPLD_PWRGD_1_OFFSET;

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Fail to read cpld offset: 0x%x", CPLD_PWRGD_1_OFFSET);
		return false;
	}

	if (msg.data[0] & CPLD_PWRGD_BIT) {
		memset(&msg, 0, sizeof(I2C_MSG));

		msg.bus = I2C_BUS3;
		msg.target_addr = CPLD_ADDR;
		msg.rx_len = 1;
		msg.tx_len = 1;
		msg.data[0] = CPLD_PWRGD_2_OFFSET;

		ret = i2c_master_read(&msg, retry);
		if (ret != 0) {
			LOG_ERR("Fail to read cpld offset: 0x%x", CPLD_PWRGD_2_OFFSET);
			return false;
		}

		if (msg.data[0] & CPLD_PWRGD_BIT) {
			is_power_good = true;
		} else {
			is_power_good = false;
		}
	} else {
		is_power_good = false;
	}

	return true;
}

bool get_acb_power_good_flag()
{
	return is_power_good;
}
