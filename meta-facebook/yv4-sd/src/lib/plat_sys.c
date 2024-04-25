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

#include "plat_sys.h"

#include "util_sys.h"
#include "hal_gpio.h"
#include "plat_gpio.h"
#include "hal_i2c.h"
#include "plat_i2c.h"

/* BMC reset */

void BMC_reset_handler()
{
	gpio_set(RST_BMC_R_N, GPIO_LOW);
	k_msleep(BMC_COLD_RESET_PULL_GPIO_INTERVAL_MS);
	gpio_set(RST_BMC_R_N, GPIO_HIGH);
}

K_WORK_DELAYABLE_DEFINE(BMC_reset_work, BMC_reset_handler);
int pal_submit_bmc_cold_reset()
{
	return k_work_schedule(&BMC_reset_work, K_MSEC(BMC_COLD_RESET_DELAY_MS));
}

uint8_t pal_get_bmc_interface()
{
	#define BMC_INTERFACE_PRSNT_BIT 0x1

	uint8_t bmc_interface = BMC_INTERFACE_I2C;
	int retry = 5;
	I2C_MSG msg = { 0 };

	// Check BMC interface setting from SD CPLD register
	msg.bus = CPLD_IO_I2C_BUS;
	msg.target_addr = CPLD_IO_I2C_ADDR;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = CPLD_REG_BMC_INTERFACE;
	if (i2c_master_read(&msg, retry) != 0) {
		// Failed to get BMC interface from cpld, use I2C as default.
		return bmc_interface;
	}

	bmc_interface = msg.data[0] & BMC_INTERFACE_PRSNT_BIT;

	return bmc_interface;
}
