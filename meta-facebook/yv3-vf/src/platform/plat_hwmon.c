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
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "plat_m2.h"
#include "plat_gpio.h"
#include "plat_power_seq.h"
#include "plat_class.h"
#include "plat_util.h"
#include "plat_i2c.h"
#include "plat_sensor_table.h"

#include "plat_hwmon.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(plat_hwmon);

static bool init_dev_prsnt_status(void)
{
	uint8_t i;
	for (i = M2_IDX_E_A; i < M2_IDX_E_MAX; i++) {
		if (!mb_cpld_dev_prsnt_set(i, m2_prsnt(i)))
			return false;
	}

	return true;
}

void BICup1secTickHandler(struct k_work *work)
{
	if (!work) {
		LOG_ERR("BICup1secTickHandler get NULL work handler!");
		return;
	}

	if (!init_dev_prsnt_status()) {
		k_work_schedule((struct k_work_delayable *)work, K_SECONDS(1));
	}
}

int8_t mb_cpld_dev_prsnt_set(uint32_t idx, uint32_t val)
{
#define MB_CPLD_PRSNT_REG_1OU 0x05
	/*
 * device present offset
 * dev0, offset 4
 * dev1, offset 3
 * dev2, offset 2
 * dev3, offset 1
 */

	uint8_t reg = MB_CPLD_PRSNT_REG_1OU;
	uint8_t prsnt_ofs = idx + 1;
	uint8_t buf;

	uint8_t retry = 5;
	I2C_MSG msg;

	msg.bus = I2C_BUS_MB_CPLD;
	msg.target_addr = I2C_ADDR_MB_CPLD;
	msg.tx_len = sizeof(reg);
	msg.rx_len = sizeof(buf);
	memcpy(&msg.data[0], &reg, sizeof(reg));

	if (i2c_master_read(&msg, retry)) {
		LOG_ERR("MB CPLD read failed!");
		return false;
	}

	buf = msg.data[0];
	WRITE_BIT(buf, prsnt_ofs, !val);

	msg.tx_len = sizeof(buf) + 1;
	msg.data[0] = reg;
	memcpy(&msg.data[1], &buf, sizeof(buf));

	if (i2c_master_write(&msg, retry)) {
		return false;
	}

	return true;
}

void set_exp_pwrgd_pin(void)
{
	uint8_t val;

	switch (get_e1s_hsc_config()) {
	case CONFIG_HSC_ADM1278:
	case CONFIG_HSC_MAXIN:
	case CONFIG_HSC_MPS:
		val = gpio_get(PWRGD_P12V_AUX);
		break;
	case CONFIG_HSC_BYPASS:
	default:
		val = gpio_get(FM_POWER_EN);
		break;
	}

	gpio_set(PWRGD_EXP_PWROK, val);
}

void plat_nvme_bus_reset(uint8_t bus) //NVMe_smb_rst
{
	gpio_set(m2_bus2rst(bus), GPIO_LOW);
	k_msleep(2);
	gpio_set(m2_bus2rst(bus), GPIO_HIGH);
}