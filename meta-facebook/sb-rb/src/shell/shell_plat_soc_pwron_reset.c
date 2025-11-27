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

#define IRIS_POWER_RESET_OFFSET 0x00
#define IRIS_BOARD_POWER_ENABLE 0xFF
#define IRIS_BOARD_POWER_DISABLE 0xC7
#define HAMSA_POWER_ON_RESET_BIT 5
#define MEDHA0_POWER_ON_RESET_BIT 4
#define MEDHA1_POWER_ON_RESET_BIT 3
#define HAMSA_SYS_RST_BIT 2
#define MEDHA0_SYS_RST_BIT 1
#define MEDHA1_SYS_RST_BIT 0

/* bit7, bit6 must be 1 */
#define RESET_BASE_MASK 0xC0 /* 1100_0000 */

#define RESET_ON_BYTE(orig, bit) ((uint8_t)((orig) | (1u << (bit)) | RESET_BASE_MASK))
#define RESET_OFF_BYTE(orig, bit) ((uint8_t)(((orig) & ~(1u << (bit))) | RESET_BASE_MASK))

LOG_MODULE_REGISTER(plat_soc_pwron_reset_shell);
typedef enum {
	HAMSA_POWER_ON_RESET_L_ID = 0,
	MEDHA0_POWER_ON_RESET_L_ID,
	MEDHA1_POWER_ON_RESET_L_ID,
	HAMSA_SYS_RST_L_ID,
	MEDHA0_SYS_RST_L_ID,
	MEDHA1_SYS_RST_L_ID,
	SOC_PWRON_RESET_MAX_ID,
} soc_pwron_reset_id_t;

typedef struct {
	soc_pwron_reset_id_t id;
	const char *name;
	uint8_t bit;
} soc_pwron_reset_item_t;

static const soc_pwron_reset_item_t soc_pwron_reset_list[SOC_PWRON_RESET_MAX_ID] = {
	{ HAMSA_POWER_ON_RESET_L_ID, "HAMSA_POWER_ON_RESET_L", HAMSA_POWER_ON_RESET_BIT },
	{ MEDHA0_POWER_ON_RESET_L_ID, "MEDHA0_POWER_ON_RESET_L", MEDHA0_POWER_ON_RESET_BIT },
	{ MEDHA1_POWER_ON_RESET_L_ID, "MEDHA1_POWER_ON_RESET_L", MEDHA1_POWER_ON_RESET_BIT },
	{ HAMSA_SYS_RST_L_ID, "HAMSA_SYS_RST_L", HAMSA_SYS_RST_BIT },
	{ MEDHA0_SYS_RST_L_ID, "MEDHA0_SYS_RST_L", MEDHA0_SYS_RST_BIT },
	{ MEDHA1_SYS_RST_L_ID, "MEDHA1_SYS_RST_L", MEDHA1_SYS_RST_BIT },
};

static int cmd_soc_pwron_reset_override(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t data = 0;
	if (!strcmp(argv[0], "override")) {
		if (!strcmp(argv[1], "1")) {
			data = IRIS_BOARD_POWER_ENABLE;
			if (!plat_write_cpld(IRIS_POWER_RESET_OFFSET, &data)) {
				shell_error(shell, "plat soc_pwron_reset set failed");
				return -1;
			}
		} else if (!strcmp(argv[1], "0")) {
			data = IRIS_BOARD_POWER_DISABLE;
			if (!plat_write_cpld(IRIS_POWER_RESET_OFFSET, &data)) {
				shell_error(shell, "plat soc_pwron_reset set failed");
				return -1;
			}
		} else {
			shell_warn(
				shell,
				"Help: soc_pwron_reset override <drive-level> should only accept 0 or 1");
			return -1;
		}
	} else if (!strcmp(argv[0], "passthru")) {
		data = IRIS_BOARD_POWER_ENABLE;
		if (!plat_write_cpld(IRIS_POWER_RESET_OFFSET, &data)) {
			shell_error(shell, "plat soc_pwron_reset set failed");
			return -1;
		}
	} else {
		shell_warn(shell, "Help: soc_pwron_reset <override/passthru> <drive-level>");
		return -1;
	}
	shell_print(shell, "plat soc_pwron_reset control finish");
	return 0;
}

static const soc_pwron_reset_item_t *get_item_by_id(soc_pwron_reset_id_t id)
{
	if (id < SOC_PWRON_RESET_MAX_ID)
		return &soc_pwron_reset_list[id];
	return NULL;
}

static const soc_pwron_reset_item_t *get_item_by_name(const char *name)
{
	for (int id = 0; id < SOC_PWRON_RESET_MAX_ID; id++) {
		if (strcmp(name, soc_pwron_reset_list[id].name) == 0)
			return &soc_pwron_reset_list[id];
	}
	return NULL;
}

static int cmd_soc_pwron_reset_get(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_print(shell, "Usage:");
		shell_print(shell, "  soc_pwron_reset get all");
		shell_print(shell, "  soc_pwron_reset get <NAME>");
		LOG_DBG("invalid argc in get: %d", (int)argc);
		return -1;
	}

	uint8_t reg_val = 0;

	if (!plat_read_cpld(IRIS_POWER_RESET_OFFSET, &reg_val, 1)) {
		LOG_DBG("plat_read_cpld failed: offset=0x%02x", IRIS_POWER_RESET_OFFSET);
		shell_print(shell, "read CPLD failed");
		return -1;
	}

	if (!strcmp(argv[1], "all")) {
		for (int id = 0; id < SOC_PWRON_RESET_MAX_ID; id++) {
			const soc_pwron_reset_item_t *item = get_item_by_id(id);
			if (!item)
				continue;

			uint8_t bit_val = (reg_val >> item->bit) & 0x1;
			shell_print(shell, "%s : %d", item->name, bit_val);
		}
		return 0;
	}

	const soc_pwron_reset_item_t *item = get_item_by_name(argv[1]);
	if (!item) {
		shell_print(shell, "Unknown name: %s", argv[1]);
		LOG_DBG("unknown name in get: %s", argv[1]);
		return -1;
	}

	uint8_t bit_val = (reg_val >> item->bit) & 0x1;
	shell_print(shell, "%s : %d", item->name, bit_val);
	return 0;
}

static int cmd_soc_pwron_reset_set(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_print(shell, "Usage: soc_pwron_reset set <NAME> <0|1>");
		LOG_DBG("invalid argc in set: %d", (int)argc);
		return -1;
	}

	const char *name = argv[1];
	long set_val = strtol(argv[2], NULL, 10);

	if ((set_val != 0) && (set_val != 1)) {
		shell_print(shell, "Value must be 0 or 1");
		LOG_DBG("invalid value in set: %ld", set_val);
		return -1;
	}

	const soc_pwron_reset_item_t *item = get_item_by_name(name);
	if (!item) {
		shell_print(shell, "Unknown name: %s", name);
		LOG_DBG("unknown name in set: %s", name);
		return -1;
	}

	uint8_t reg_val = 0;

	if (!plat_read_cpld(IRIS_POWER_RESET_OFFSET, &reg_val, 1)) {
		LOG_DBG("plat_read_cpld failed: offset=0x%02x", IRIS_POWER_RESET_OFFSET);
		shell_print(shell, "read CPLD failed");
		return -1;
	}

	if (set_val == 1) {
		reg_val = RESET_ON_BYTE(reg_val, item->bit);
	} else {
		reg_val = RESET_OFF_BYTE(reg_val, item->bit);
	}

	if (!plat_write_cpld(IRIS_POWER_RESET_OFFSET, &reg_val)) {
		LOG_DBG("plat_write_cpld failed: offset=0x%02x val=0x%02x", IRIS_POWER_RESET_OFFSET,
			reg_val);
		shell_print(shell, "write CPLD failed");
		return -1;
	}

	shell_print(shell, "set %s to %ld done", item->name, set_val);
	return 0;
}

bool soc_pwron_reset_name_get(uint8_t idx, uint8_t **name)
{
	if (idx >= SOC_PWRON_RESET_MAX_ID) {
		*name = NULL;
		return false;
	}

	*name = (uint8_t *)soc_pwron_reset_list[idx].name;
	return true;
}

static void soc_pwron_reset_rname_get_(size_t idx, struct shell_static_entry *entry)
{
	uint8_t *name = NULL;
	soc_pwron_reset_name_get((uint8_t)idx, &name);

	entry->syntax = (name) ? (const char *)name : NULL;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;
}

SHELL_DYNAMIC_CMD_CREATE(oc_pwron_reset_name, soc_pwron_reset_rname_get_);

/* level 1 */
SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_soc_pwron_reset_cmds,
	SHELL_CMD_ARG(override, NULL, "soc_pwron_reset override <drive-level>",
		      cmd_soc_pwron_reset_override, 2, 1),
	SHELL_CMD(passthru, NULL, "soc_pwron_reset passthru", cmd_soc_pwron_reset_override),
	SHELL_CMD(get, NULL, "soc_pwron_reset get all | get <NAME>", cmd_soc_pwron_reset_get),
	SHELL_CMD(set, &oc_pwron_reset_name, "soc_pwron_reset set <NAME> <0|1>",
		  cmd_soc_pwron_reset_set),
	SHELL_SUBCMD_SET_END);

/* Root of command soc_pwron_reset */
SHELL_CMD_REGISTER(soc_pwron_reset, &sub_soc_pwron_reset_cmds, "soc_pwron_reset commands", NULL);
