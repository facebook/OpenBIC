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
#include "plat_i2c.h"
#include "util_spi.h"

int pal_get_bios_flash_position()
{
	return DEVSPI_SPI1_CS0;
}

int pal_get_prot_flash_position()
{
	return DEVSPI_SPI2_CS0;
}

bool pal_switch_bios_spi_mux(int gpio_status)
{
	uint8_t retry = 5;
	I2C_MSG msg;

	/* BIC switch the MUX selection pin by setting CPLD register.
	 * Serverboard CPLD register:
	 * Offset 0Ch - BIC SPI OOB0 control
	 * bit[3:0]=0010b, boot from CPU(default)
	 * bit[3:0]=1011b, boot from BIC
	 */
	msg.bus = SB_CPLD_BUS;
	msg.target_addr = SB_CPLD_ADDR;
	msg.tx_len = 2;
	msg.data[0] = SB_CPLD_REG_SPI_OOB_CONTROL;
	if (gpio_status == GPIO_HIGH) {
		msg.data[1] = SB_CPLD_SPI_OOB_FROM_BIC;
	} else {
		msg.data[1] = SB_CPLD_SPI_OOB_FROM_CPU;
	}

	if (i2c_master_write(&msg, retry)) {
		return false;
	}
	return true;
}