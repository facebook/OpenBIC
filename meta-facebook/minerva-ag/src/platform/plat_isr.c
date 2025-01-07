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

#include <zephyr.h>
#include <stdio.h>
#include <stdlib.h>
#include <logging/log.h>
#include "libutil.h"
#include "libipmi.h"
#include "power_status.h"
#include "sensor.h"

#include "plat_gpio.h"
#include "plat_i2c.h"
#include "plat_class.h"
#include "plat_isr.h"
#include "plat_hwmon.h"
#include "plat_event.h"

LOG_MODULE_REGISTER(plat_isr);

#define AEGIS_CPLD_ADDR (0x4C >> 1)
#define AEGIS_312M_CLK_GEN_ADDR (0x12 >> 1)
#define AEGIS_EUSB_REPEATER_ADDR (0x86 >> 1)

void check_clk_handler();

K_TIMER_DEFINE(check_ubc_delayed_timer, check_ubc_delayed, NULL);
K_WORK_DELAYABLE_DEFINE(check_clk_work, check_clk_handler);

void ISR_GPIO_FM_ASIC_0_THERMTRIP_R_N()
{
	LOG_DBG("gpio_%d_isr called, val=%d , dir= %d", FM_ASIC_0_THERMTRIP_R_N,
		gpio_get(FM_ASIC_0_THERMTRIP_R_N), gpio_get_direction(FM_ASIC_0_THERMTRIP_R_N));
}

void ISR_GPIO_RST_ATH_PWR_ON_PLD_R1_N()
{
	LOG_DBG("gpio_%d_isr called, val=%d , dir= %d", RST_ATH_PWR_ON_PLD_R1_N,
		gpio_get(RST_ATH_PWR_ON_PLD_R1_N), gpio_get_direction(RST_ATH_PWR_ON_PLD_R1_N));
}

void ISR_GPIO_ATH_CURRENT_SENSE_0_NPCM_R()
{
	LOG_DBG("gpio_%d_isr called, val=%d , dir= %d", ATH_CURRENT_SENSE_0_NPCM_R,
		gpio_get(ATH_CURRENT_SENSE_0_NPCM_R),
		gpio_get_direction(ATH_CURRENT_SENSE_0_NPCM_R));
}

void ISR_GPIO_ATH_CURRENT_SENSE_1_NPCM_R()
{
	LOG_DBG("gpio_%d_isr called, val=%d , dir= %d", ATH_CURRENT_SENSE_1_NPCM_R,
		gpio_get(ATH_CURRENT_SENSE_1_NPCM_R),
		gpio_get_direction(ATH_CURRENT_SENSE_1_NPCM_R));
}

void ISR_GPIO_FM_ATH_HBM3_CATTRIP_ALARM_LV33_R()
{
	LOG_DBG("gpio_%d_isr called, val=%d , dir= %d", FM_ATH_HBM3_CATTRIP_ALARM_LV33_R,
		gpio_get(FM_ATH_HBM3_CATTRIP_ALARM_LV33_R),
		gpio_get_direction(FM_ATH_HBM3_CATTRIP_ALARM_LV33_R));
}

void ISR_GPIO_ALL_VR_PM_ALERT_R_N()
{
	LOG_DBG("gpio_%d_isr called, val=%d , dir= %d", ALL_VR_PM_ALERT_R_N,
		gpio_get(ALL_VR_PM_ALERT_R_N), gpio_get_direction(ALL_VR_PM_ALERT_R_N));
}

void ISR_GPIO_ATH_SMB_ALERT_NPCM_LVC33_R_N()
{
	LOG_DBG("gpio_%d_isr called, val=%d , dir= %d", ATH_SMB_ALERT_NPCM_LVC33_R_N,
		gpio_get(ATH_SMB_ALERT_NPCM_LVC33_R_N),
		gpio_get_direction(ATH_SMB_ALERT_NPCM_LVC33_R_N));
}

void ISR_GPIO_FM_PLD_UBC_EN_R()
{
	LOG_DBG("gpio_%d_isr called, val=%d , dir= %d", FM_PLD_UBC_EN_R, gpio_get(FM_PLD_UBC_EN_R),
		gpio_get_direction(FM_PLD_UBC_EN_R));

	if (gpio_get(FM_PLD_UBC_EN_R) == GPIO_HIGH) {
		plat_clock_init();
	}

	k_timer_start(&check_ubc_delayed_timer, K_MSEC(3000), K_NO_WAIT);
	set_dc_status_changing_status(true);
}

bool plat_i2c_read(uint8_t bus, uint8_t addr, uint8_t offset, uint8_t *data, uint8_t len)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);

	memset(data, 0, len);

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = len;
	i2c_msg.data[0] = offset;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Failed to read reg, bus: %d, addr: 0x%x, reg: 0x%x", bus, addr, offset);
		return false;
	}

	memcpy(data, i2c_msg.data, len);
	return true;
}

bool plat_i2c_write(uint8_t bus, uint8_t addr, uint8_t offset, uint8_t *data, uint8_t len)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.tx_len = len + 1;
	i2c_msg.data[0] = offset;

	if (len > 0)
		memcpy(&i2c_msg.data[1], data, len);

	if (i2c_master_write(&i2c_msg, retry)) {
		LOG_ERR("Failed to write reg, bus: %d, addr: 0x%x, reg: 0x%x", bus, addr, offset);
		return false;
	}
	return true;
}

void check_clk_handler()
{
	uint8_t data[4] = { 0 };

	if (!plat_i2c_read(I2C_BUS5, AEGIS_CPLD_ADDR, 0x0B, data, 1)) {
		LOG_ERR("Failed to read cpld");
		return;
	}

	uint8_t pwrgd_p3v3_and_p1v8 = (data[0] & GENMASK(4, 3)) >> 3;
	if (pwrgd_p3v3_and_p1v8 == 3) {
		memset(data, 0, sizeof(data));
		memcpy(data, (uint8_t[]){ 0x00, 0x01, 0x00, 0x00 }, 4);
		if (!plat_i2c_write(I2C_BUS1, AEGIS_312M_CLK_GEN_ADDR, 0xFC, data, 4)) {
			LOG_ERR("Failed to write 312M CLK GEN");
			return;
		}
		memset(data, 0, sizeof(data));
		memcpy(data, (uint8_t[]){ 0x84 }, 1);
		if (!plat_i2c_write(I2C_BUS1, AEGIS_312M_CLK_GEN_ADDR, 0x04, data, 1)) {
			LOG_ERR("Failed to write 312M CLK GEN");
			return;
		}
		memset(data, 0, sizeof(data));
		memcpy(data, (uint8_t[]){ 0x00, 0x01, 0x00, 0x00 }, 4);
		if (!plat_i2c_write(I2C_BUS1, AEGIS_312M_CLK_GEN_ADDR, 0xFC, data, 4)) {
			LOG_ERR("Failed to write 312M CLK GEN");
			return;
		}
		memset(data, 0, sizeof(data));
		memcpy(data, (uint8_t[]){ 0x74 }, 1);
		if (!plat_i2c_write(I2C_BUS1, AEGIS_312M_CLK_GEN_ADDR, 0x14, data, 1)) {
			LOG_ERR("Failed to write 312M CLK GEN");
			return;
		}
		LOG_INF("Init 312M CLK GEN finish");

	} else {
		LOG_ERR("pwrgd_p3v3_and_p1v8 fail= 0x%02X", pwrgd_p3v3_and_p1v8);
	}
}

void plat_clock_init(void)
{
	LOG_DBG("plat_clock_init started");
	uint8_t board_stage = get_board_stage();
	if (board_stage == FAB2_DVT || board_stage == FAB3_PVT || board_stage == FAB4_MP) {
		k_work_schedule(&check_clk_work, K_MSEC(100));
	}
}

void plat_eusb_init(void)
{
	LOG_DBG("plat_eusb_init started");
	uint8_t board_stage = get_board_stage();
	if (board_stage == FAB2_DVT || board_stage == FAB3_PVT || board_stage == FAB4_MP) {
		uint8_t data[1] = { 0 };
		if (!plat_i2c_read(I2C_BUS1, AEGIS_EUSB_REPEATER_ADDR, 0x05, data, 1)) {
			LOG_ERR("Failed to read eUSB Repeater");
			return;
		}

		uint8_t eusb_default_setting = data[0];
		if (eusb_default_setting == 0x10) {
			memset(data, 0, sizeof(data));
			memcpy(data, (uint8_t[]){ 0x20 }, 1);
			if (!plat_i2c_write(I2C_BUS1, AEGIS_EUSB_REPEATER_ADDR, 0x05, data, 1)) {
				LOG_ERR("Failed to write eUSB Repeater");
				return;
			}
			LOG_INF("Init eUSB Repeater, set offset 0x05 to 0x20, default seeting=0x%02X",
				eusb_default_setting);
		} else {
			LOG_INF("eUSB Repeater offset 0x05 default seeting=0x%02X",
				eusb_default_setting);
		}
	}
}
