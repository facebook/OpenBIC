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
  [0] = { .bus = I2C_BUS7,
    .mux_addr = ASIC_CARD_1_6_MUX_ADDR,
    .mux_channel = PCA9548A_CHANNEL_0,
    .card_status = ASIC_CARD_UNKNOWN_STATUS,
    .device_mux_addr = ASIC_CARD_DEVICE_MUX_ADDR,
    .device_channel = PCA9546A_CHANNEL_0,
    .device_reg_offset = 0x1D,
    .asic_1_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS,
    .asic_2_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS, },
  [1] = { .bus = I2C_BUS7,
    .mux_addr = ASIC_CARD_1_6_MUX_ADDR,
    .mux_channel = PCA9548A_CHANNEL_1,
    .card_status = ASIC_CARD_UNKNOWN_STATUS,
    .device_mux_addr = ASIC_CARD_DEVICE_MUX_ADDR,
    .device_channel = PCA9546A_CHANNEL_0,
    .device_reg_offset = 0x1A,
    .asic_1_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS,
    .asic_2_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS, },
  [2] = { .bus = I2C_BUS7,
    .mux_addr = ASIC_CARD_1_6_MUX_ADDR,
    .mux_channel = PCA9548A_CHANNEL_2,
    .card_status = ASIC_CARD_UNKNOWN_STATUS,
    .device_mux_addr = ASIC_CARD_DEVICE_MUX_ADDR,
    .device_channel = PCA9546A_CHANNEL_0,
    .device_reg_offset = 0x17,
    .asic_1_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS,
    .asic_2_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS, },
  [3] = { .bus = I2C_BUS7,
    .mux_addr = ASIC_CARD_1_6_MUX_ADDR,
    .mux_channel = PCA9548A_CHANNEL_3,
    .card_status = ASIC_CARD_UNKNOWN_STATUS,
    .device_mux_addr = ASIC_CARD_DEVICE_MUX_ADDR,
    .device_channel = PCA9546A_CHANNEL_0,
    .device_reg_offset = 0x14,
    .asic_1_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS,
    .asic_2_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS, },
  [4] = { .bus = I2C_BUS7,
    .mux_addr = ASIC_CARD_1_6_MUX_ADDR,
    .mux_channel = PCA9548A_CHANNEL_4,
    .card_status = ASIC_CARD_UNKNOWN_STATUS,
    .device_mux_addr = ASIC_CARD_DEVICE_MUX_ADDR,
    .device_channel = PCA9546A_CHANNEL_0,
    .device_reg_offset = 0x11,
    .asic_1_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS,
    .asic_2_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS, },
  [5] = { .bus = I2C_BUS7,
    .mux_addr = ASIC_CARD_1_6_MUX_ADDR,
    .mux_channel = PCA9548A_CHANNEL_5,
    .card_status = ASIC_CARD_UNKNOWN_STATUS,
    .device_mux_addr = ASIC_CARD_DEVICE_MUX_ADDR,
    .device_channel = PCA9546A_CHANNEL_0,
    .device_reg_offset = 0x0E,
    .asic_1_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS,
    .asic_2_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS, },
  [6] = { .bus = I2C_BUS8,
    .mux_addr = ASIC_CARD_7_12_MUX_ADDR,
    .mux_channel = PCA9548A_CHANNEL_0,
    .card_status = ASIC_CARD_UNKNOWN_STATUS,
    .device_mux_addr = ASIC_CARD_DEVICE_MUX_ADDR,
    .device_channel = PCA9546A_CHANNEL_0,
    .device_reg_offset = 0x1D,
    .asic_1_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS,
    .asic_2_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS, },
  [7] = { .bus = I2C_BUS8,
    .mux_addr = ASIC_CARD_7_12_MUX_ADDR,
    .mux_channel = PCA9548A_CHANNEL_1,
    .card_status = ASIC_CARD_UNKNOWN_STATUS,
    .device_mux_addr = ASIC_CARD_DEVICE_MUX_ADDR,
    .device_channel = PCA9546A_CHANNEL_0,
    .device_reg_offset = 0x1A,
    .asic_1_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS,
    .asic_2_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS, },
  [8] = { .bus = I2C_BUS8,
    .mux_addr = ASIC_CARD_7_12_MUX_ADDR,
    .mux_channel = PCA9548A_CHANNEL_2,
    .card_status = ASIC_CARD_UNKNOWN_STATUS,
    .device_mux_addr = ASIC_CARD_DEVICE_MUX_ADDR,
    .device_channel = PCA9546A_CHANNEL_0,
    .device_reg_offset = 0x17,
    .asic_1_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS,
    .asic_2_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS, },
  [9] = { .bus = I2C_BUS8,
    .mux_addr = ASIC_CARD_7_12_MUX_ADDR,
    .mux_channel = PCA9548A_CHANNEL_3,
    .card_status = ASIC_CARD_UNKNOWN_STATUS,
    .device_mux_addr = ASIC_CARD_DEVICE_MUX_ADDR,
    .device_channel = PCA9546A_CHANNEL_0,
    .device_reg_offset = 0x14,
    .asic_1_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS,
    .asic_2_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS, },
  [10] = { .bus = I2C_BUS8,
     .mux_addr = ASIC_CARD_7_12_MUX_ADDR,
     .mux_channel = PCA9548A_CHANNEL_4,
     .card_status = ASIC_CARD_UNKNOWN_STATUS,
     .device_mux_addr = ASIC_CARD_DEVICE_MUX_ADDR,
     .device_channel = PCA9546A_CHANNEL_0,
     .device_reg_offset = 0x11,
     .asic_1_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS,
     .asic_2_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS, },
  [11] = { .bus = I2C_BUS8,
     .mux_addr = ASIC_CARD_7_12_MUX_ADDR,
     .mux_channel = PCA9548A_CHANNEL_5,
     .card_status = ASIC_CARD_UNKNOWN_STATUS,
     .device_mux_addr = ASIC_CARD_DEVICE_MUX_ADDR,
     .device_channel = PCA9546A_CHANNEL_0,
     .device_reg_offset = 0x0E,
     .asic_1_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS,
     .asic_2_status = ASIC_CARD_DEVICE_UNKNOWN_STATUS, },
};

void check_accl_device_presence_status_via_pex(uint8_t pex_id)
{
	uint8_t ret = 0;
	uint8_t val = 0;
	uint8_t index = 0;
	uint8_t offset = 0;
	uint8_t start_card_id = 0;
	uint32_t reg = PEX_ACCL_DEV_PRESENT_REG;

	switch (pex_id) {
	case PEX_0_INDEX:
		start_card_id = PEX_0_START_ACCL_ID;
		ret = pex_access_engine(PEX_0_BUS, PEX_ADDR, pex_id, pex_access_register, &reg);
		break;
	case PEX_1_INDEX:
		start_card_id = PEX_1_START_ACCL_ID;
		ret = pex_access_engine(PEX_1_BUS, PEX_ADDR, pex_id, pex_access_register, &reg);
		break;
	default:
		LOG_ERR("PEX id: %d is invalid", pex_id);
		return;
	}

	if (ret != pex_api_success) {
		LOG_ERR("Access ACCL register fail");
		return;
	}

	for (offset = 0; offset < PEX_ACCL_DEV_PRESENT_REG_COUNT; ++offset) {
		index = start_card_id + offset;
		if (index >= ASIC_CARD_COUNT) {
			LOG_ERR("ACCL card id: %d is invalid", index);
			break;
		}

		val = (reg >> asic_card_info[index].device_reg_offset) & PEX_ACCL_PRESENT_MAP_VAL;
		switch (val) {
		case ASIC_CARD_NOT_PRESENT_VAL:
		case ASIC_DEV_NOT_PRESENT_VAL:
			asic_card_info[index].asic_1_status = ASIC_CARD_DEVICE_NOT_PRESENT;
			asic_card_info[index].asic_2_status = ASIC_CARD_DEVICE_NOT_PRESENT;
			break;
		case ASIC_DEV_1_PRESENT_VAL:
			asic_card_info[index].asic_1_status = ASIC_CARD_DEVICE_PRESENT;
			asic_card_info[index].asic_2_status = ASIC_CARD_DEVICE_NOT_PRESENT;
			break;
		case ASIC_DEV_2_PRESENT_VAL:
			asic_card_info[index].asic_1_status = ASIC_CARD_DEVICE_NOT_PRESENT;
			asic_card_info[index].asic_2_status = ASIC_CARD_DEVICE_PRESENT;
			break;
		case ASIC_DEV_1_2_PRESENT_VAL:
			asic_card_info[index].asic_1_status = ASIC_CARD_DEVICE_PRESENT;
			asic_card_info[index].asic_2_status = ASIC_CARD_DEVICE_PRESENT;
			break;
		default:
			LOG_ERR("Invalid register val: 0x%x", val);
			break;
		}

		if ((asic_card_info[index].asic_1_status == ASIC_CARD_DEVICE_PRESENT) ||
		    (asic_card_info[index].asic_2_status == ASIC_CARD_DEVICE_PRESENT)) {
			pal_init_drive(plat_accl_sensor_config, ACCL_SENSOR_CONFIG_SIZE, index);
		}
	}
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
		card_index = addr_index * IOEXP_CARD_PRESENCE_COUNT;

		memset(&msg, 0, sizeof(I2C_MSG));
		msg.bus = I2C_BUS13;
		msg.target_addr = ioexp_addr[addr_index];
		msg.rx_len = 1;
		msg.tx_len = 1;
		msg.data[0] = PCA9555_OUTPUT_PORT_REG_1;

		ret = i2c_master_read(&msg, retry);
		if (ret != 0) {
			LOG_ERR("Fail to read ioexp addr: 0x%x, offset: 0x%x", msg.target_addr,
				msg.data[0]);
			continue;
		}

		reg_val = msg.data[0] << 8;

		memset(&msg, 0, sizeof(I2C_MSG));
		msg.bus = I2C_BUS13;
		msg.target_addr = ioexp_addr[addr_index];
		msg.rx_len = 1;
		msg.tx_len = 1;
		msg.data[0] = PCA9555_OUTPUT_PORT_REG_0;

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
			switch (presence_val) {
			case IOEXP_DEV_NOT_PRESENT_VAL:
				asic_card_info[card_index].asic_1_status =
					ASIC_CARD_DEVICE_NOT_PRESENT;
				asic_card_info[card_index].asic_2_status =
					ASIC_CARD_DEVICE_NOT_PRESENT;
				break;
			case IOEXP_DEV_1_PRESENT_VAL:
				asic_card_info[card_index].asic_1_status = ASIC_CARD_DEVICE_PRESENT;
				asic_card_info[card_index].asic_2_status =
					ASIC_CARD_DEVICE_NOT_PRESENT;
				break;
			case IOEXP_DEV_2_PRESENT_VAL:
				asic_card_info[card_index].asic_1_status =
					ASIC_CARD_DEVICE_NOT_PRESENT;
				asic_card_info[card_index].asic_2_status = ASIC_CARD_DEVICE_PRESENT;
				break;
			case IOEXP_DEV_1_2_PRESENT_VAL:
				asic_card_info[card_index].asic_1_status = ASIC_CARD_DEVICE_PRESENT;
				asic_card_info[card_index].asic_2_status = ASIC_CARD_DEVICE_PRESENT;
				break;
			default:
				LOG_ERR("Invalid presence val: 0x%x, addr index: 0x%x, ioexp index: 0x%x, reg val: 0x%x",
					presence_val, addr_index, ioexp_index, reg_val);
				break;
			}

			if ((asic_card_info[card_index].asic_1_status ==
			     ASIC_CARD_DEVICE_PRESENT) ||
			    (asic_card_info[card_index].asic_2_status ==
			     ASIC_CARD_DEVICE_PRESENT)) {
				pal_init_drive(plat_accl_sensor_config, ACCL_SENSOR_CONFIG_SIZE,
					       card_index);
			}

			card_index += 1;
		}
	}
}

void check_asic_card_status()
{
	int ret = 0;
	int index = 0;
	int device_index = 0;
	int mutex_status = 0;
	mux_config i2c_mux = { 0 };
	uint8_t i2c_dev[I2C_BUFF_SIZE] = { 0 };
	uint8_t dev_count = 0;

	for (index = 0; index < ASIC_CARD_COUNT; ++index) {
		memset(&i2c_mux, 0, sizeof(i2c_mux));
		memset(&i2c_dev, 0, sizeof(i2c_dev));

		i2c_mux.bus = asic_card_info[index].bus;
		i2c_mux.target_addr = asic_card_info[index].mux_addr;
		i2c_mux.channel = asic_card_info[index].mux_channel;

		struct k_mutex *mutex = get_i2c_mux_mutex(asic_card_info[index].bus);
		mutex_status = k_mutex_lock(mutex, K_MSEC(MUTEX_LOCK_INTERVAL_MS));
		if (mutex_status != 0) {
			LOG_ERR("Mutex lock fail, index: 0x%x, status: %d", index, mutex_status);
			continue;
		}

		ret = set_mux_channel(i2c_mux, MUTEX_LOCK_ENABLE);
		if (ret != true) {
			LOG_ERR("Switch ASIC%d mux fail", index);
			k_mutex_unlock(mutex);
			continue;
		}

		i2c_scan(i2c_mux.bus, i2c_dev, &dev_count);
		k_mutex_unlock(mutex);

		for (device_index = 0; device_index < dev_count; ++device_index) {
			if (i2c_dev[device_index] == (ACCL_FRU_ADDR << 1)) {
				asic_card_info[index].card_status = ASIC_CARD_PRESENT;
				pal_init_drive(plat_accl_sensor_config, ACCL_SENSOR_CONFIG_SIZE,
					       index);
				break;
			}
		}

		if (device_index >= dev_count) {
			asic_card_info[index].card_status = ASIC_CARD_NOT_PRESENT;
		}
	}

	if (board_revision > EVT1_STAGE) {
		if (board_revision == UNKNOWN_STAGE) {
			LOG_ERR("Check accl device presence status failed because board revision is in unknown stage");
		} else {
			check_accl_device_presence_status_via_ioexp();
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
