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
#include <logging/log.h>
#include "plat_hook.h"
#include "plat_class.h"
#include "hal_i2c.h"
#include "plat_i2c.h"

LOG_MODULE_REGISTER(plat_throttle_switch_shell, LOG_LEVEL_DBG);

#define AEGIS_CPLD_ADDR (0x4C >> 1)
#define I2C_BUS_CPLD I2C_BUS5
#define CPLD_THROTTLE_SWITCH_ADDR 0x32

static bool get_throttle_status(const struct shell *shell, uint8_t *throttle_status_reg)
{
	CHECK_NULL_ARG_WITH_RETURN(throttle_status_reg, false);

	I2C_MSG msg = { 0 };
	msg.bus = I2C_BUS_CPLD;
	msg.target_addr = AEGIS_CPLD_ADDR;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = CPLD_THROTTLE_SWITCH_ADDR;

	if (i2c_master_read(&msg, 3)) {
		shell_error(shell, "Failed to read from bus %d device: %x", msg.bus,
			    msg.target_addr);
		return false;
	}

	*throttle_status_reg = msg.data[0];
	return true;
}

void cmd_throttle_status_get(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t throttle_status_reg = 0;
	if (!get_throttle_status(shell, &throttle_status_reg)) {
		shell_print(shell, "Can't get throttle status");
		return;
	}

	bool sense0_enabled = (throttle_status_reg & BIT(7)) != 0;
	bool sense1_enabled = (throttle_status_reg & BIT(6)) != 0;

	shell_print(shell, "Throttle sense0: %s", sense0_enabled ? "enabled" : "disabled");
	shell_print(shell, "Throttle sense1: %s", sense1_enabled ? "enabled" : "disabled");
}

int cmd_throttle_status_en(const struct shell *shell, size_t argc, char **argv)
{
	bool is_perm = false;

	if (argc == 2) {
		if (!strcmp(argv[1], "perm")) {
			is_perm = true;
		} else {
			shell_error(shell, "The last argument must be <perm>");
			return -1;
		}
	}

	uint8_t throttle_status_reg = 0;
	if (!get_throttle_status(shell, &throttle_status_reg)) {
		shell_print(shell, "Can't get throttle status");
		return -1;
	}

	const char *name = argv[-1];

	if (!strcmp(name, "sense0")) {
		throttle_status_reg |= BIT(7);
	} else if (!strcmp(name, "sense1")) {
		throttle_status_reg |= BIT(6);
	} else if (!strcmp(name, "all")) {
		throttle_status_reg |= (BIT(7) | BIT(6));
	} else {
		shell_error(shell, "Unknown name: %s.", name);
		return -1;
	}

	if (!set_throttle_user_settings(&throttle_status_reg, is_perm)) {
		shell_error(shell, "Failed to set throttle %s %s %s", argv[-1], argv[0],
			    (argc == 2) ? argv[1] : "");
		return -1;
	}

	shell_info(shell, "throttle set %s %s %s finish", argv[-1], argv[0],
		   (argc == 2) ? argv[1] : "");
	return 0;
}

int cmd_throttle_status_dis(const struct shell *shell, size_t argc, char **argv)
{
	bool is_perm = false;

	if (argc == 2) {
		if (!strcmp(argv[1], "perm")) {
			is_perm = true;
		} else {
			shell_error(shell, "The last argument must be <perm>");
			return -1;
		}
	}

	uint8_t throttle_status_reg = 0;
	if (!get_throttle_status(shell, &throttle_status_reg)) {
		shell_print(shell, "Can't get throttle status");
		return -1;
	}

	const char *name = argv[-1];

	if (!strcmp(name, "sense0")) {
		throttle_status_reg &= ~BIT(7);
	} else if (!strcmp(name, "sense1")) {
		throttle_status_reg &= ~BIT(6);
	} else if (!strcmp(name, "all")) {
		throttle_status_reg &= ~(BIT(7) | BIT(6));
	} else {
		shell_error(shell, "Unknown name: %s.", name);
		return -1;
	}

	if (!set_throttle_user_settings(&throttle_status_reg, is_perm)) {
		shell_error(shell, "Failed to set throttle %s %s %s", argv[-1], argv[0],
			    (argc == 2) ? argv[1] : "");
		return -1;
	}

	shell_info(shell, "throttle set %s %s %s finsih", argv[-1], argv[0],
		   (argc == 2) ? argv[1] : "");

	return 0;
}

static const struct shell_static_entry throttle_status_entries[] = {
	{
		.syntax = "enable",
		.handler = cmd_throttle_status_en,
		.subcmd = NULL,
		.help = "Enable throttle",
	},
	{
		.syntax = "disable",
		.handler = cmd_throttle_status_dis,
		.subcmd = NULL,
		.help = "Disable throttle",
	},
};

static void throttle_status_get(size_t idx, struct shell_static_entry *entry)
{
	if (idx < ARRAY_SIZE(throttle_status_entries)) {
		*entry = throttle_status_entries[idx];
	} else {
		entry->syntax = NULL;
	}
}

SHELL_DYNAMIC_CMD_CREATE(throttle_status, throttle_status_get);

static const struct shell_static_entry throttle_name_entries[] = {
	{ .syntax = "sense0", .handler = NULL, .subcmd = &throttle_status, .help = "name: sense0" },
	{ .syntax = "sense1", .handler = NULL, .subcmd = &throttle_status, .help = "name: sense1" },
	{ .syntax = "all",
	  .handler = NULL,
	  .subcmd = &throttle_status,
	  .help = "name: all channels" },
};

static void throttle_name_get(size_t idx, struct shell_static_entry *entry)
{
	if (idx < ARRAY_SIZE(throttle_name_entries)) {
		*entry = throttle_name_entries[idx];
	} else {
		entry->syntax = NULL;
	}
}

SHELL_DYNAMIC_CMD_CREATE(throttle_name, throttle_name_get);

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_throttle_switch_cmds,
	SHELL_CMD(get, NULL, "get throttle switch status", cmd_throttle_status_get),
	SHELL_CMD_ARG(set, &throttle_name,
		      "set throttle <sense0|sense1|all> <enable|disable> [perm]", NULL, 3, 1),
	SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(throttle_switch, &sub_throttle_switch_cmds, "thermal trip switch command", NULL);
