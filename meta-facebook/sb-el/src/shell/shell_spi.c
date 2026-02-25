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

#define ARKE_BOARD_POWER_ENABLE 0xFF

#define HAMSA_POWER_ON_RESET_PLD_L_BIT 5
#define NUWA0_POWER_ON_RESET_PLD_L_BIT 4
#define NUWA1_POWER_ON_RESET_PLD_L_BIT 3
#define HAMSA_SYS_RST_PLD_L_BIT 2
#define NUWA0_SYS_RST_PLD_L_BIT 1
#define NUWA1_SYS_RST_PLD_L_BIT 0

void cmd_spi_disable(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "set All SPI_MUX to ASIC");
	gpio_set(SPI_HAMSA_MUX_IN1, 0);
	gpio_set(SPI_NUWA0_MUX_IN1, 0);
	gpio_set(SPI_NUWA1_MUX_IN1, 0);
	gpio_set(QSPI_CPLD_SEL_0, 0);
	gpio_set(QSPI_CPLD_SEL_1, 0);
	uint8_t reset_release = ARKE_BOARD_POWER_ENABLE;
	if (!plat_write_cpld(CPLD_OFFSET_ASIC_RESET, &reset_release)) {
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
	if (!set_cpld_bit(CPLD_OFFSET_ASIC_RESET, HAMSA_POWER_ON_RESET_PLD_L_BIT, 0) ||
	    !set_cpld_bit(CPLD_OFFSET_ASIC_RESET, HAMSA_SYS_RST_PLD_L_BIT, 0)) {
		shell_error(shell, "failed to drive HAMSA reset bits low");
		return;
	}
	shell_print(shell, "HAMSA is held in reset");
}

void cmd_spi_enable_nuwa0(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "set SPI_MUX_NUWA0 to MMC");
	gpio_set(SPI_NUWA0_MUX_IN1, 1);
	gpio_set(QSPI_CPLD_SEL_0, 1);
	gpio_set(QSPI_CPLD_SEL_1, 0);
	if (!set_cpld_bit(CPLD_OFFSET_ASIC_RESET, NUWA0_POWER_ON_RESET_PLD_L_BIT, 0) ||
	    !set_cpld_bit(CPLD_OFFSET_ASIC_RESET, NUWA0_SYS_RST_PLD_L_BIT, 0)) {
		shell_error(shell, "failed to drive NUWA0 reset bits low");
		return;
	}
	shell_print(shell, "NUWA0 is held in reset");
}

void cmd_spi_enable_nuwa1(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "set SPI_MUX_NUWA1 to MMC");
	gpio_set(SPI_NUWA1_MUX_IN1, 1);
	gpio_set(QSPI_CPLD_SEL_0, 0);
	gpio_set(QSPI_CPLD_SEL_1, 1);
	if (!set_cpld_bit(CPLD_OFFSET_ASIC_RESET, NUWA1_POWER_ON_RESET_PLD_L_BIT, 0) ||
	    !set_cpld_bit(CPLD_OFFSET_ASIC_RESET, NUWA1_SYS_RST_PLD_L_BIT, 0)) {
		shell_error(shell, "failed to drive NUWA1 reset bits low");
		return;
	}
	shell_print(shell, "NUWA1 is held in reset");
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_spu_enable_cmds,
	SHELL_CMD(hamsa, NULL, "spi enable hamsa test command", cmd_spi_enable_hamsa),
	SHELL_CMD(nuwa0, NULL, "spi enable nuwa0 test command", cmd_spi_enable_nuwa0),
	SHELL_CMD(nuwa1, NULL, "spi enable nuwa1 test command", cmd_spi_enable_nuwa1),
	SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_set_spimux_oob_cmds,
			       SHELL_CMD(enable, &sub_spu_enable_cmds, "spi enable test command",
					 NULL),
			       SHELL_CMD(disable, NULL, "spi disable test command",
					 cmd_spi_disable),
			       SHELL_SUBCMD_SET_END);

/* Root of command spi test */
SHELL_CMD_REGISTER(set_spimux_oob, &sub_set_spimux_oob_cmds, "spi test commands", NULL);