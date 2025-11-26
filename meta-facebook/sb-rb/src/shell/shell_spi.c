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
#include "plat_cpld.h"
#include "plat_gpio.h"

#define IRIS_BOARD_POWER_ENABLE 0xFF

#define HAMSA_POWER_ON_RESET_PLD_L_BIT 5
#define MEDHA0_POWER_ON_RESET_PLD_L_BIT 4
#define MEDHA1_POWER_ON_RESET_PLD_L_BIT 3
#define HAMSA_SYS_RST_PLD_L_BIT 2
#define MEDHA0_SYS_RST_PLD_L_BIT 1
#define MEDHA1_SYS_RST_PLD_L_BIT 0

void cmd_spi_disable(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "set All SPI_MUX to ASIC");
	gpio_set(SPI_HAMSA_MUX_IN1, 0);
	gpio_set(SPI_MEDHA0_MUX_IN1, 0);
	gpio_set(SPI_MEDHA1_MUX_IN1, 0);
	gpio_set(QSPI_CPLD_SEL_0, 0);
	gpio_set(QSPI_CPLD_SEL_1, 0);
	uint8_t reset_release = IRIS_BOARD_POWER_ENABLE;
	if (!plat_write_cpld(RESET, &reset_release)) {
		shell_error(shell, "failed to write 0xFF to CPLD reset register");
		return;
	}
	shell_print(shell, "all reset signals are inactive");
}

void cmd_spi_enable_hamsa(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "set SPI_MUX_HAMSA to MMC");
	gpio_set(SPI_HAMSA_MUX_IN1, 1);
	gpio_set(QSPI_CPLD_SEL_0, 0);
	gpio_set(QSPI_CPLD_SEL_1, 0);
	if (!set_cpld_bit(RESET, HAMSA_POWER_ON_RESET_PLD_L_BIT, 0) ||
	    !set_cpld_bit(RESET, HAMSA_SYS_RST_PLD_L_BIT, 0)) {
		shell_error(shell, "failed to drive HAMSA reset bits low");
		return;
	}
	shell_print(shell, "HAMSA is held in reset");
}

void cmd_spi_enable_medha0(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "set SPI_MUX_MEDHA0 to MMC");
	gpio_set(SPI_MEDHA0_MUX_IN1, 1);
	gpio_set(QSPI_CPLD_SEL_0, 1);
	gpio_set(QSPI_CPLD_SEL_1, 0);
	if (!set_cpld_bit(RESET, MEDHA0_POWER_ON_RESET_PLD_L_BIT, 0) ||
	    !set_cpld_bit(RESET, MEDHA0_SYS_RST_PLD_L_BIT, 0)) {
		shell_error(shell, "failed to drive MEDHA0 reset bits low");
		return;
	}
	shell_print(shell, "MEDHA0 is held in reset");
}

void cmd_spi_enable_medha1(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "set SPI_MUX_MEDHA1 to MMC");
	gpio_set(SPI_MEDHA1_MUX_IN1, 1);
	gpio_set(QSPI_CPLD_SEL_0, 0);
	gpio_set(QSPI_CPLD_SEL_1, 1);
	if (!set_cpld_bit(RESET, MEDHA1_POWER_ON_RESET_PLD_L_BIT, 0) ||
	    !set_cpld_bit(RESET, MEDHA1_SYS_RST_PLD_L_BIT, 0)) {
		shell_error(shell, "failed to drive MEDHA1 reset bits low");
		return;
	}
	shell_print(shell, "MEDHA1 is held in reset");
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_spu_enable_cmds,
	SHELL_CMD(hamsa, NULL, "spi enable hamsa test command", cmd_spi_enable_hamsa),
	SHELL_CMD(medha0, NULL, "spi enable medha0 test command", cmd_spi_enable_medha0),
	SHELL_CMD(medha1, NULL, "spi enable medha1 test command", cmd_spi_enable_medha1),
	SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_set_spimux_oob_cmds,
			       SHELL_CMD(enable, &sub_spu_enable_cmds, "spi enable test command",
					 NULL),
			       SHELL_CMD(disable, NULL, "spi disable test command",
					 cmd_spi_disable),
			       SHELL_SUBCMD_SET_END);

/* Root of command spi test */
SHELL_CMD_REGISTER(set_spimux_oob, &sub_set_spimux_oob_cmds, "spi test commands", NULL);
