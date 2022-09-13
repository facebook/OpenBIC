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

#include "plat_i2c.h"
#include "hal_i2c.h"
#include "util_spi.h"
#include "hal_gpio.h"
#include "libutil.h"

#define CPLD_ADDR 0x21
#define CPLD_SPI_CONTROL_REG 0x00

int pal_get_bios_flash_position()
{
	return DEVSPI_SPI1_CS0;
}

bool pal_switch_bios_spi_mux(int gpio_status)
{
	uint8_t retry = 5;
	I2C_MSG msg;

	msg.bus = I2C_BUS2;
	msg.target_addr = CPLD_ADDR;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = CPLD_SPI_CONTROL_REG;
	//Read data from the cpld register to avoid modifying the original data
	if (i2c_master_read(&msg, retry)) {
		return false;
	}

	//set the spi mux status to bit 2;
	msg.tx_len = 2;
	if (gpio_status == GPIO_HIGH) {
		//switch mux to bic
		msg.data[1] = SETBIT(msg.data[0], 2);
	} else {
		//switch mux to host
		msg.data[1] = CLEARBIT(msg.data[0], 2);
	}
	msg.data[0] = CPLD_SPI_CONTROL_REG;

	if (i2c_master_write(&msg, retry)) {
		return false;
	}
	return true;
}
