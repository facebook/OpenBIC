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

#include "hal_gpio.h"
#include "hal_i2c.h"
#include "plat_gpio.h"
#include "plat_i2c.h"
#include "util_spi.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(plat_spi);

#define CPLD_REG_SPI_MUX 0x0B

int pal_get_bios_flash_position()
{
	return DEVSPI_SPI1_CS0;
}

bool pal_switch_bios_spi_mux(int gpio_status)
{
	I2C_MSG msg = { 0 };
	int retry = 3;

	msg.bus = I2C_BUS1;
	msg.target_addr = (CPLD_I2C_ADDR >> 1);
	msg.tx_len = 2;
	msg.data[0] = CPLD_REG_SPI_MUX;
	msg.data[1] = (gpio_status == GPIO_HIGH) ? 0:1;

	int ret = i2c_master_write(&msg, retry);
	if (ret) {
		LOG_ERR("Failed to switch spi mux, ret: %d", ret);
		return false;
	}

	return true;
}
