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

#include <stdlib.h>
#include <shell/shell.h>

#include "plat_gpio.h"

void cmd_spi_disable(const struct shell *shell, size_t argc, char **argv)
{
	gpio_set(SPI_HAMSA_MUX_IN1, 0);
	gpio_set(SPI_MEDHA0_MUX_IN1, 0);
	gpio_set(SPI_MEDHA1_MUX_IN1, 0);
	gpio_set(QSPI_CPLD_SEL_0, 0);
	gpio_set(QSPI_CPLD_SEL_1, 0);
}

void cmd_spi_enable_hamsa(const struct shell *shell, size_t argc, char **argv)
{
	gpio_set(SPI_HAMSA_MUX_IN1, 1);
	gpio_set(QSPI_CPLD_SEL_0, 0);
	gpio_set(QSPI_CPLD_SEL_1, 0);
}

void cmd_spi_enable_medha0(const struct shell *shell, size_t argc, char **argv)
{
	gpio_set(SPI_MEDHA0_MUX_IN1, 1);
	gpio_set(QSPI_CPLD_SEL_0, 1);
	gpio_set(QSPI_CPLD_SEL_1, 0);
}

void cmd_spi_enable_medha1(const struct shell *shell, size_t argc, char **argv)
{
	gpio_set(SPI_MEDHA1_MUX_IN1, 1);
	gpio_set(QSPI_CPLD_SEL_0, 0);
	gpio_set(QSPI_CPLD_SEL_1, 1);
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_spu_enable_cmds,
	SHELL_CMD(hamsa, NULL, "spi enable hamsa test command", cmd_spi_enable_hamsa),
	SHELL_CMD(medha0, NULL, "spi enable medha0 test command", cmd_spi_enable_medha0),
	SHELL_CMD(medha1, NULL, "spi enable medha1 test command", cmd_spi_enable_medha1),
	SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_spi_test_cmds, SHELL_CMD(enable, &sub_spu_enable_cmds, "spi enable test command", NULL),
	SHELL_CMD(disable, NULL, "spi disable test command", cmd_spi_disable),
	SHELL_SUBCMD_SET_END);

/* Root of command spi test */
SHELL_CMD_REGISTER(spi_test, &sub_spi_test_cmds, "spi test commands", NULL);
