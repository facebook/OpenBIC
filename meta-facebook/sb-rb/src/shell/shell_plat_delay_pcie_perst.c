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
#include "plat_isr.h"
#include "plat_i2c.h"
#include "plat_hook.h"
#include "plat_cpld.h"
#include "plat_user_setting.h"

LOG_MODULE_REGISTER(plat_delay_pcie_perst_shell, LOG_LEVEL_DBG);

typedef enum {
	HAMSA_PCIE0 = 0,
	HAMSA_PCIE1,
	HAMSA_PCIE2,
	HAMSA_PCIE3,
	HAMSA_PCIE_MAX_ID,
} hamsa_pcie_id_t;

typedef struct {
	hamsa_pcie_id_t id;
	const char *name;
	uint8_t cpld_offset;
	uint8_t user_setting_offset;
} hamsa_pcie_item_t;

static const hamsa_pcie_item_t hamsa_pcie_list[HAMSA_PCIE_MAX_ID] = {
	{ HAMSA_PCIE0, "HAMSA_PCIE0_PERST_B_PLD_N", 0x9D, 0 },
	{ HAMSA_PCIE1, "HAMSA_PCIE1_PERST_B_PLD_N", 0xB3, 4 },
	{ HAMSA_PCIE2, "HAMSA_PCIE2_PERST_B_PLD_N", 0xB4, 8 },
	{ HAMSA_PCIE3, "HAMSA_PCIE3_PERST_B_PLD_N", 0xB5, 12 },
};

static int cmd_delay_pcie_perst_set(const struct shell *shell, size_t argc, char **argv)
{
	bool is_perm = false;

	if (argc == 4) {
		if (!strcmp(argv[3], "perm")) {
			is_perm = true;
		} else {
			shell_error(shell, "The last argument must be <perm>");
			return -1;
		}
	}

	uint8_t idx = 0xff;
	for (hamsa_pcie_id_t id = 0; id < HAMSA_PCIE_MAX_ID; id++) {
		if (!strcmp(argv[1], hamsa_pcie_list[id].name)) {
			idx = id;
			break;
		}
	}

	if (idx == 0xff) {
		shell_error(shell, "only support HAMSA_PCIE[X]_PERST_B_PLD_N, x should be 0 to 3");
		return -1;
	}

	uint8_t setting_value = 0;
	if (!strcmp(argv[2], "default")) {
		shell_info(shell, "Set %s delay to default, %svolatile\n",
			   hamsa_pcie_list[idx].name, (argc == 4) ? "non-" : "");
	} else {
		setting_value = strtol(argv[2], NULL, 10);
		if (setting_value < 0 || setting_value > 200) {
			shell_warn(shell, "Help: N range from 0 to 200");
			return -1;
		}
		uint16_t delay_time = setting_value * 10;
		shell_info(shell, "Set %s delay to %d ms, %svolatile\n", hamsa_pcie_list[idx].name,
			   delay_time, (argc == 4) ? "non-" : "");
	}

	if (!plat_write_cpld(hamsa_pcie_list[idx].cpld_offset, &setting_value)) {
		shell_error(shell, "plat delay_pcie_perst set failed");
		return -1;
	}

	//write to eeprom
	if (is_perm) {
		uint8_t perm_value[4] = { 0xff, 0xff, 0xff, 0xff };
		perm_value[0] = setting_value;
		if (!set_user_settings_delay_pcie_perst_to_eeprom(
			    &perm_value[0], sizeof(perm_value),
			    hamsa_pcie_list[idx].user_setting_offset)) {
			shell_error(shell, "write %s delay to eeprom error",
				    hamsa_pcie_list[idx].name);
			return -1;
		}
	}

	return 0;
}
static int cmd_delay_pcie_perst_get(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 1 || strcmp(argv[0], "all")) {
		shell_print(shell, "Usage:");
		shell_print(shell, "  delay_pcie_perst get all");
		return -1;
	}

	for (int id = 0; id < HAMSA_PCIE_MAX_ID; id++) {
		uint8_t setting_value = 0;
		plat_read_cpld(hamsa_pcie_list[id].cpld_offset, &setting_value, 1);
		shell_print(shell, "%s : %d ms", hamsa_pcie_list[id].name, setting_value * 10, 1);
	}
	return 0;
}
static void hamsa_pcie_rname_get_for_set_cmd(size_t idx, struct shell_static_entry *entry)
{
	entry->syntax = NULL;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;

	if (idx < HAMSA_PCIE_MAX_ID) {
		entry->syntax = hamsa_pcie_list[idx].name;
	}
}

SHELL_DYNAMIC_CMD_CREATE(hamsa_pcie_name_set, hamsa_pcie_rname_get_for_set_cmd);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_delay_pcie_perst_get_cmds,
			       SHELL_CMD(all, NULL, "get all", cmd_delay_pcie_perst_get),
			       SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_delay_pcie_perst_cmds,
	SHELL_CMD_ARG(set, &hamsa_pcie_name_set,
		      "set <HAMSA_PCIE[X]_PERST_B_PLD_N > [N (* 10ms)]|default [perm]",
		      cmd_delay_pcie_perst_set, 3, 1),
	SHELL_CMD(get, &sub_delay_pcie_perst_get_cmds, "get all", NULL), SHELL_SUBCMD_SET_END);

/* Root of command delay_pcie_perst */
SHELL_CMD_REGISTER(delay_pcie_perst, &sub_delay_pcie_perst_cmds,
		   "delay_pcie_perst set <HAMSA_PCIE[X]_PERST_B_PLD_N > [N (* 10ms)]|default [perm]",
		   NULL);
