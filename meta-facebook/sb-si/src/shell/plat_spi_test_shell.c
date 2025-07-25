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
#include <stdlib.h>
#include <shell/shell.h>
#include "plat_spi_test_shell.h"
#include "plat_gpio.h"

void cmd_enable_spi(const struct shell *shell, size_t argc, char **argv)
{
	if (gpio_set(SPI_MUX_SEL, true) == -1) {
		shell_warn(shell, "SPI_MUX_SEL set fail");
	};
}

void cmd_disable_spi(const struct shell *shell, size_t argc, char **argv)
{
	if (gpio_set(SPI_MUX_SEL, false) == -1) {
		shell_warn(shell, "SPI_MUX_SEL set fail");
	};
}