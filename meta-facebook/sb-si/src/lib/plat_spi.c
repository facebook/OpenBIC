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

#include "plat_spi.h"
#include "hal_gpio.h"
#include "plat_gpio.h"
#include "util_spi.h"
#include "plat_i2c.h"
#include "plat_i3c.h"

int pal_get_pcie_switch_flash_position()
{
	return DEVSPI_SPI1_CS1;
}

bool pal_switch_pcie_switch_spi_mux(int gpio_status)
{
	if (gpio_set(SPI_MUX_SEL, gpio_status) == -1) {
		return false;
	};
	return true;
}
