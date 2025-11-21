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

#include "plat_i2c.h"
#include "plat_cpld.h"

LOG_MODULE_REGISTER(pcie_perst_control, LOG_LEVEL_DBG);

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

#define PCIE_PERST_CPLD_OFFSET 0xA3
#define PCIE_PERST_OVERRIDE_ON 0xFF
#define PCIE_PERST_OVERRIDE_OFF 0xF0

/* bit mapping */
#define HAMSA_PCIE0_PERST_BIT 0
#define HAMSA_PCIE1_PERST_BIT 1
#define HAMSA_PCIE2_PERST_BIT 2
#define HAMSA_PCIE3_PERST_BIT 3

#define PCIE_PERST_SET_BIT(orig, bit) ((uint8_t)((orig) | (1u << (bit))))
#define PCIE_PERST_CLR_BIT(orig, bit) ((uint8_t)((orig) & ~(1u << (bit))))

typedef struct {
	const char *name;
	uint8_t bit;
} pcie_perst_item_t;

static const pcie_perst_item_t perst_list[] = {
	{ "HAMSA_PCIE0_PERST_B_PLD_N", HAMSA_PCIE0_PERST_BIT },
	{ "HAMSA_PCIE1_PERST_B_PLD_N", HAMSA_PCIE1_PERST_BIT },
	{ "HAMSA_PCIE2_PERST_B_PLD_N", HAMSA_PCIE2_PERST_BIT },
	{ "HAMSA_PCIE3_PERST_B_PLD_N", HAMSA_PCIE3_PERST_BIT },
};

static const pcie_perst_item_t *get_perst_item(const char *name)
{
	for (int i = 0; i < ARRAY_SIZE(perst_list); i++) {
		if (!strcmp(name, perst_list[i].name)) {
			return &perst_list[i];
		}
	}
	return NULL;
}

static int cmd_pcie_perst_override(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_print(shell, "Usage: pcie_perst_control override <0|1>");
		return -1;
	}

	long set_val = strtol(argv[1], NULL, 10);
	if (set_val != 0 && set_val != 1) {
		shell_error(shell, "Value must be 0 or 1");
		return -1;
	}

	uint8_t write_val = (set_val == 1) ? PCIE_PERST_OVERRIDE_ON : PCIE_PERST_OVERRIDE_OFF;

	if (!plat_write_cpld(PCIE_PERST_CPLD_OFFSET, &write_val)) {
		shell_error(shell, "write CPLD failed");
		return -1;
	}

	shell_print(shell, "override all PERST bits to %ld done", set_val);
	return 0;
}

static int cmd_pcie_perst_get(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_print(shell, "Usage:");
		shell_print(shell, "  pcie_perst_control get all");
		shell_print(shell, "  pcie_perst_control get <NAME>");
		LOG_DBG("invalid argc in get: %d", (int)argc);
		return -1;
	}

	uint8_t reg_val = 0;

	if (!plat_read_cpld(PCIE_PERST_CPLD_OFFSET, &reg_val, 1)) {
		LOG_DBG("plat_read_cpld failed: offset=0x%02x", PCIE_PERST_CPLD_OFFSET);
		shell_error(shell, "read CPLD failed");
		return -1;
	}

	if (!strcmp(argv[1], "all")) {
		for (int i = 0; i < ARRAY_SIZE(perst_list); i++) {
			const pcie_perst_item_t *item = &perst_list[i];
			uint8_t bit_val = (reg_val >> item->bit) & 0x1;
			shell_print(shell, "%s : %d", item->name, bit_val);
		}
		return 0;
	}

	const pcie_perst_item_t *item = get_perst_item(argv[1]);
	if (!item) {
		shell_error(shell, "Unknown name: %s", argv[1]);
		LOG_DBG("unknown name in get: %s", argv[1]);
		return -1;
	}

	uint8_t bit_val = (reg_val >> item->bit) & 0x1;
	shell_print(shell, "%s : %d", item->name, bit_val);
	return 0;
}

static int cmd_pcie_perst_set(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_print(shell, "Usage: pcie_perst_control set <NAME> <0|1>");
		LOG_DBG("invalid argc in set: %d", (int)argc);
		return -1;
	}

	const char *name = argv[1];
	long set_val = strtol(argv[2], NULL, 10);

	if (set_val != 0 && set_val != 1) {
		shell_error(shell, "Value must be 0 or 1");
		LOG_DBG("invalid value in set: %ld", set_val);
		return -1;
	}

	const pcie_perst_item_t *item = get_perst_item(name);
	if (!item) {
		shell_error(shell, "Unknown name: %s", name);
		LOG_DBG("unknown name in set: %s", name);
		return -1;
	}

	uint8_t reg_val = 0;

	if (!plat_read_cpld(PCIE_PERST_CPLD_OFFSET, &reg_val, 1)) {
		LOG_DBG("plat_read_cpld failed: offset=0x%02x", PCIE_PERST_CPLD_OFFSET);
		shell_error(shell, "read CPLD failed");
		return -1;
	}

	if (set_val == 1) {
		reg_val = PCIE_PERST_SET_BIT(reg_val, item->bit);
	} else {
		reg_val = PCIE_PERST_CLR_BIT(reg_val, item->bit);
	}

	if (!plat_write_cpld(PCIE_PERST_CPLD_OFFSET, &reg_val)) {
		LOG_DBG("plat_write_cpld failed: offset=0x%02x val=0x%02x", PCIE_PERST_CPLD_OFFSET,
			reg_val);
		shell_error(shell, "write CPLD failed");
		return -1;
	}

	shell_print(shell, "set %s to %ld done", name, set_val);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	pcie_perst_subcmds,
	SHELL_CMD(override, NULL, "pcie_perst_control override <0|1>", cmd_pcie_perst_override),
	SHELL_CMD(get, NULL, "pcie_perst_control get all | get <NAME>", cmd_pcie_perst_get),
	SHELL_CMD(set, NULL, "pcie_perst_control set <NAME> <0|1>", cmd_pcie_perst_set),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(pcie_perst_control, &pcie_perst_subcmds, "PCIe PERST control via CPLD", NULL);
