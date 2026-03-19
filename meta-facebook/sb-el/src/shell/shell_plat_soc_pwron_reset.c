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
#include <logging/log.h>
#include <string.h>
#include "plat_isr.h"
#include "plat_i2c.h"
#include "plat_cpld.h"

LOG_MODULE_REGISTER(plat_soc_pwron_reset_shell);

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

#define ARKE_BOARD_POWER_ENABLE 0xFF
#define ARKE_BOARD_POWER_DISABLE 0xC7

#define HAMSA_POWER_ON_RESET_BIT 5
#define NUWA0_POWER_ON_RESET_BIT 4
#define NUWA1_POWER_ON_RESET_BIT 3
#define HAMSA_SYS_RST_BIT 2
#define NUWA0_SYS_RST_BIT 1
#define NUWA1_SYS_RST_BIT 0

typedef struct {
	const char *name;
	uint8_t bit;
} soc_pwron_reset_item_t;

static const soc_pwron_reset_item_t soc_pwron_reset_list[] = {
	{ "HAMSA_POWER_ON_RESET_L", HAMSA_POWER_ON_RESET_BIT },
	{ "NUWA0_POWER_ON_RESET_L", NUWA0_POWER_ON_RESET_BIT },
	{ "NUWA1_POWER_ON_RESET_L", NUWA1_POWER_ON_RESET_BIT },
	{ "HAMSA_SYS_RST_L", HAMSA_SYS_RST_BIT },
	{ "NUWA0_SYS_RST_L", NUWA0_SYS_RST_BIT },
	{ "NUWA1_SYS_RST_L", NUWA1_SYS_RST_BIT },
};

#define SOC_PWRON_RESET_CNT ARRAY_SIZE(soc_pwron_reset_list)

static const soc_pwron_reset_item_t *get_item_by_name(const char *name)
{
	for (size_t i = 0; i < SOC_PWRON_RESET_CNT; i++) {
		if (strcmp(name, soc_pwron_reset_list[i].name) == 0) {
			return &soc_pwron_reset_list[i];
		}
	}
	return NULL;
}

static int cmd_soc_pwron_reset_override(const struct shell *shell, size_t argc, char **argv)
{
	/* soc_pwron_reset override <0|1>
	 * soc_pwron_reset passthru
	 *
	 * This path writes a full byte, so it must keep bit7/6 = 1.
	 * ARKE_BOARD_POWER_ENABLE/DISABLE already satisfy that.
	 */
	uint8_t data = 0;

	if (argc < 1) {
		shell_warn(shell, "Usage: soc_pwron_reset <override|passthru> [0|1]");
		return -1;
	}

	if (!strcmp(argv[0], "override")) {
		if (argc != 2) {
			shell_warn(shell, "Usage: soc_pwron_reset override <0|1>");
			return -1;
		}

		if (!strcmp(argv[1], "1")) {
			data = ARKE_BOARD_POWER_ENABLE; /* includes base mask */
		} else if (!strcmp(argv[1], "0")) {
			data = ARKE_BOARD_POWER_DISABLE; /* includes base mask */
		} else {
			shell_warn(shell, "override <0|1> only");
			return -1;
		}

		if (!plat_write_cpld(CPLD_OFFSET_ASIC_RESET, &data)) {
			shell_error(shell, "write CPLD failed");
			return -1;
		}

		shell_print(shell, "soc_pwron_reset override=%s done (0x%02x)", argv[1], data);
		return 0;
	}

	if (!strcmp(argv[0], "passthru")) {
		if (argc != 1) {
			shell_warn(shell, "Usage: soc_pwron_reset passthru");
			return -1;
		}

		data = ARKE_BOARD_POWER_ENABLE; /* includes base mask */
		if (!plat_write_cpld(CPLD_OFFSET_ASIC_RESET, &data)) {
			shell_error(shell, "write CPLD failed");
			return -1;
		}

		shell_print(shell, "soc_pwron_reset passthru done (0x%02x)", data);
		return 0;
	}

	shell_warn(shell, "Usage: soc_pwron_reset <override|passthru> [0|1]");
	return -1;
}

static int cmd_soc_pwron_reset_get(const struct shell *shell, size_t argc, char **argv)
{
	/* soc_pwron_reset get all
	 * soc_pwron_reset get <NAME>
	 */
	if (argc != 2) {
		shell_print(shell, "Usage:");
		shell_print(shell, "  soc_pwron_reset get all");
		shell_print(shell, "  soc_pwron_reset get <NAME>");
		return -1;
	}

	uint8_t reg_val = 0;
	if (!plat_read_cpld(CPLD_OFFSET_ASIC_RESET, &reg_val, 1)) {
		shell_error(shell, "read CPLD failed");
		return -1;
	}

	if (!strcmp(argv[1], "all")) {
		for (size_t i = 0; i < SOC_PWRON_RESET_CNT; i++) {
			const soc_pwron_reset_item_t *item = &soc_pwron_reset_list[i];
			uint8_t bit_val = (reg_val >> item->bit) & 0x1;
			shell_print(shell, "%s : %d", item->name, bit_val);
		}
		/* optional: show raw register */
		shell_print(shell, "CPLD[0x%02x]=0x%02x", CPLD_OFFSET_ASIC_RESET, reg_val);
		return 0;
	}

	const soc_pwron_reset_item_t *item = get_item_by_name(argv[1]);
	if (!item) {
		shell_error(shell, "Unknown name: %s", argv[1]);
		return -1;
	}

	uint8_t bit_val = (reg_val >> item->bit) & 0x1;
	shell_print(shell, "%s : %d", item->name, bit_val);
	return 0;
}

static int cmd_soc_pwron_reset_set(const struct shell *shell, size_t argc, char **argv)
{
	/* soc_pwron_reset set <NAME> <0|1> */
	if (argc != 3) {
		shell_print(shell, "Usage: soc_pwron_reset set <NAME> <0|1>");
		return -1;
	}

	const char *name = argv[1];
	long set_val = strtol(argv[2], NULL, 10);

	if ((set_val != 0) && (set_val != 1)) {
		shell_error(shell, "Value must be 0 or 1");
		return -1;
	}

	const soc_pwron_reset_item_t *item = get_item_by_name(name);
	if (!item) {
		shell_error(shell, "Unknown name: %s", name);
		return -1;
	}

	if (!set_cpld_bit(CPLD_OFFSET_ASIC_RESET, item->bit, (uint8_t)set_val)) {
		shell_error(shell, "set_cpld_bit failed");
		return -1;
	}

	shell_print(shell, "set %s to %ld done", item->name, set_val);
	return 0;
}

static bool soc_pwron_reset_name_get(uint8_t idx, uint8_t **name)
{
	if (!name) {
		return false;
	}

	if (idx >= SOC_PWRON_RESET_CNT) {
		*name = NULL;
		return false;
	}

	*name = (uint8_t *)soc_pwron_reset_list[idx].name;
	return true;
}

/* for: soc_pwron_reset set <NAME> <0|1> */
static void soc_pwron_reset_set_name_get_(size_t idx, struct shell_static_entry *entry)
{
	uint8_t *name = NULL;
	soc_pwron_reset_name_get((uint8_t)idx, &name);

	entry->syntax = (name) ? (const char *)name : NULL;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;
}
SHELL_DYNAMIC_CMD_CREATE(soc_pwron_reset_set_dynamic, soc_pwron_reset_set_name_get_);

/* for: soc_pwron_reset get all | get <NAME> */
static void soc_pwron_reset_get_name_get_(size_t idx, struct shell_static_entry *entry)
{
	/* idx=0 => "all", then idx-1 maps to list[] */
	if (idx == 0) {
		entry->syntax = "all";
		entry->handler = NULL;
		entry->help = NULL;
		entry->subcmd = NULL;
		return;
	}

	uint8_t *name = NULL;
	soc_pwron_reset_name_get((uint8_t)(idx - 1), &name);

	entry->syntax = (name) ? (const char *)name : NULL;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;
}
SHELL_DYNAMIC_CMD_CREATE(soc_pwron_reset_get_dynamic, soc_pwron_reset_get_name_get_);

/* level 1 */
SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_soc_pwron_reset_cmds,
	SHELL_CMD_ARG(override, NULL, "soc_pwron_reset override <0|1>",
		      cmd_soc_pwron_reset_override, 2, 1),
	SHELL_CMD(passthru, NULL, "soc_pwron_reset passthru", cmd_soc_pwron_reset_override),
	SHELL_CMD(get, &soc_pwron_reset_get_dynamic, "soc_pwron_reset get all | get <NAME>",
		  cmd_soc_pwron_reset_get),
	SHELL_CMD(set, &soc_pwron_reset_set_dynamic, "soc_pwron_reset set <NAME> <0|1>",
		  cmd_soc_pwron_reset_set),
	SHELL_SUBCMD_SET_END);

/* Root */
SHELL_CMD_REGISTER(soc_pwron_reset, &sub_soc_pwron_reset_cmds, "soc_pwron_reset commands", NULL);