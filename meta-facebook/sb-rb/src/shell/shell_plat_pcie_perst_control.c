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

#include <shell/shell.h>
#include <stdlib.h>
#include <string.h>
#include <logging/log.h>

#include "plat_cpld.h"

LOG_MODULE_REGISTER(pcie_perst_control, LOG_LEVEL_DBG);

#define PCIE_PERST_CPLD_OFFSET 0xA3
#define PCIE_PERST_OVERRIDE_ON 0xFF
#define PCIE_PERST_OVERRIDE_OFF 0xF0

/* bit mapping */
#define HAMSA_PCIE0_PERST_BIT 0
#define HAMSA_PCIE1_PERST_BIT 1
#define HAMSA_PCIE2_PERST_BIT 2
#define HAMSA_PCIE3_PERST_BIT 3

static bool pcie_perst_resolve_bit(const char *name, uint8_t *bit)
{
	if (!strcmp(name, "HAMSA_PCIE0_PERST_B_PLD_N")) {
		*bit = HAMSA_PCIE0_PERST_BIT;
	} else if (!strcmp(name, "HAMSA_PCIE1_PERST_B_PLD_N")) {
		*bit = HAMSA_PCIE1_PERST_BIT;
	} else if (!strcmp(name, "HAMSA_PCIE2_PERST_B_PLD_N")) {
		*bit = HAMSA_PCIE2_PERST_BIT;
	} else if (!strcmp(name, "HAMSA_PCIE3_PERST_B_PLD_N")) {
		*bit = HAMSA_PCIE3_PERST_BIT;
	} else {
		return false;
	}
	return true;
}

static int cmd_pcie_perst_override(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_print(shell, "Usage: pcie_perst_control override <0|1>");
		return -1;
	}

	long val = strtol(argv[1], NULL, 10);
	if (val != 0 && val != 1) {
		shell_error(shell, "Value must be 0 or 1");
		return -1;
	}

	uint8_t write_val = (val == 1) ? PCIE_PERST_OVERRIDE_ON : PCIE_PERST_OVERRIDE_OFF;

	if (!plat_write_cpld(PCIE_PERST_CPLD_OFFSET, &write_val)) {
		shell_error(shell, "write CPLD failed");
		return -1;
	}

	shell_print(shell, "override all PERST bits to %ld done", val);
	return 0;
}

static int cmd_pcie_perst_get(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_print(shell, "Usage:");
		shell_print(shell, "  pcie_perst_control get all");
		shell_print(shell, "  pcie_perst_control get <NAME>");
		return -1;
	}

	uint8_t reg_val = 0;
	if (!plat_read_cpld(PCIE_PERST_CPLD_OFFSET, &reg_val, 1)) {
		shell_error(shell, "read CPLD failed");
		return -1;
	}

	if (!strcmp(argv[1], "all")) {
		shell_print(shell, "raw [0x%02X] = 0x%02X", PCIE_PERST_CPLD_OFFSET, reg_val);

		shell_print(shell, "HAMSA_PCIE0_PERST_B_PLD_N : %d",
			    (reg_val >> HAMSA_PCIE0_PERST_BIT) & 0x1);
		shell_print(shell, "HAMSA_PCIE1_PERST_B_PLD_N : %d",
			    (reg_val >> HAMSA_PCIE1_PERST_BIT) & 0x1);
		shell_print(shell, "HAMSA_PCIE2_PERST_B_PLD_N : %d",
			    (reg_val >> HAMSA_PCIE2_PERST_BIT) & 0x1);
		shell_print(shell, "HAMSA_PCIE3_PERST_B_PLD_N : %d",
			    (reg_val >> HAMSA_PCIE3_PERST_BIT) & 0x1);

		return 0;
	}

	uint8_t bit = 0;
	if (!pcie_perst_resolve_bit(argv[1], &bit)) {
		shell_error(shell, "Unknown name: %s", argv[1]);
		return -1;
	}

	uint8_t bit_val = (reg_val >> bit) & 0x1;
	shell_print(shell, "%s : %d", argv[1], bit_val);
	return 0;
}

static int cmd_pcie_perst_set(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_print(shell, "Usage: pcie_perst_control set <NAME> <0|1>");
		return -1;
	}

	long val = strtol(argv[2], NULL, 10);
	if (val != 0 && val != 1) {
		shell_error(shell, "Value must be 0 or 1");
		return -1;
	}

	uint8_t bit = 0;
	if (!pcie_perst_resolve_bit(argv[1], &bit)) {
		shell_error(shell, "Unknown name: %s", argv[1]);
		return -1;
	}

	if (!set_cpld_bit(PCIE_PERST_CPLD_OFFSET, bit, (uint8_t)val)) {
		shell_error(shell, "set_cpld_bit failed");
		return -1;
	}

	shell_print(shell, "set %s to %ld done", argv[1], val);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	pcie_perst_subcmds,
	SHELL_CMD(override, NULL, "pcie_perst_control override <0|1>", cmd_pcie_perst_override),
	SHELL_CMD(get, NULL, "pcie_perst_control get all | get <NAME>", cmd_pcie_perst_get),
	SHELL_CMD(set, NULL, "pcie_perst_control set <NAME> <0|1>", cmd_pcie_perst_set),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(pcie_perst_control, &pcie_perst_subcmds, "PCIe PERST control via CPLD", NULL);
