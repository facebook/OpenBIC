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

LOG_MODULE_REGISTER(plat_ath_gpio_switch_shell, LOG_LEVEL_DBG);

#define AEGIS_CPLD_ADDR (0x4C >> 1)
#define I2C_BUS_CPLD I2C_BUS5
#define CPLD_ATH_GPIO_SWITCH_ADDR 0x3D

static bool get_ath_gpio_status(const struct shell *shell, uint8_t *ath_gpio_status_reg)
{
	CHECK_NULL_ARG_WITH_RETURN(ath_gpio_status_reg, false);

	I2C_MSG msg = { 0 };
	msg.bus = I2C_BUS_CPLD;
	msg.target_addr = AEGIS_CPLD_ADDR;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = CPLD_ATH_GPIO_SWITCH_ADDR;

	if (i2c_master_read(&msg, 3)) {
		shell_error(shell, "Failed to read from bus %d device: %x", msg.bus,
			    msg.target_addr);
		return false;
	}

	*ath_gpio_status_reg = msg.data[0];
	return true;
}

void cmd_ath_gpio_status_get(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t ath_gpio_status_reg = 0;
	if (!get_ath_gpio_status(shell, &ath_gpio_status_reg)) {
		shell_print(shell, "Can't get ath_gpio status");
		return;
	}

	bool ath_gpio_3_enabled = (ath_gpio_status_reg & ATH_GPIO_3_BIT) != 0;
	bool ath_gpio_4_enabled = (ath_gpio_status_reg & ATH_GPIO_4_BIT) != 0;

	shell_print(shell, "ATH_GPIO_3 switch %s", ath_gpio_3_enabled ? "enabled" : "disabled");
	shell_print(shell, "ATH_GPIO_4 switch %s", ath_gpio_4_enabled ? "enabled" : "disabled");
}

static int cmd_ath_gpio_status_set(const struct shell *shell, size_t argc, char **argv)
{
	if (argc < 3) {
		shell_error(
			shell,
			"Usage: ath_gpio_switch set <ATH_GPIO_3|ATH_GPIO_4|all> <enable|disable> [perm]");
		return -1;
	}

	const char *gpio_name = argv[1];
	const char *action = argv[2];
	bool is_perm = false;

	if (argc == 4) {
		if (!strcmp(argv[3], "perm")) {
			is_perm = true;
		} else {
			shell_error(shell, "The last argument must be <perm>");
			return -1;
		}
	}

	uint8_t ath_gpio_status_reg = 0;
	if (!get_ath_gpio_status(shell, &ath_gpio_status_reg)) {
		shell_error(shell, "Can't get ath_gpio status");
		return -1;
	}

	bool enable = !strcmp(action, "enable");
	bool disable = !strcmp(action, "disable");
	if (!enable && !disable) {
		shell_error(shell, "Invalid action: %s (expected enable|disable)", action);
		return -1;
	}

	if (!strcmp(gpio_name, "ATH_GPIO_3")) {
		ath_gpio_status_reg =
			(ath_gpio_status_reg & ~ATH_GPIO_3_BIT) | (enable ? ATH_GPIO_3_BIT : 0);
	} else if (!strcmp(gpio_name, "ATH_GPIO_4")) {
		ath_gpio_status_reg =
			(ath_gpio_status_reg & ~ATH_GPIO_4_BIT) | (enable ? ATH_GPIO_4_BIT : 0);
	} else if (!strcmp(gpio_name, "all")) {
		if (enable)
			ath_gpio_status_reg |= (ATH_GPIO_3_BIT | ATH_GPIO_4_BIT);
		else
			ath_gpio_status_reg &= ~(ATH_GPIO_3_BIT | ATH_GPIO_4_BIT);
	} else {
		shell_error(shell, "Invalid GPIO name: %s", gpio_name);
		return -1;
	}

	if (!set_ath_gpio_user_settings(&ath_gpio_status_reg, is_perm)) {
		shell_error(shell, "Failed to set ath_gpio %s %s %s", argv[1], argv[2],
			    (argc == 4) ? argv[3] : "");
		return -1;
	}

	shell_info(shell, "ath_gpio set %s %s %s finish", argv[1], argv[2],
		   (argc == 4) ? argv[3] : "");
	return 0;
}

static const struct shell_static_entry ath_gpio_name_entries[] = {
	{ .syntax = "ATH_GPIO_3", .handler = NULL, .subcmd = NULL, .help = "name: ATH_GPIO_3" },
	{ .syntax = "ATH_GPIO_4", .handler = NULL, .subcmd = NULL, .help = "name: ATH_GPIO_4" },
	{ .syntax = "all", .handler = NULL, .subcmd = NULL, .help = "name: all channels" },
};

static void ath_gpio_name_get(size_t idx, struct shell_static_entry *entry)
{
	if (idx < ARRAY_SIZE(ath_gpio_name_entries)) {
		*entry = ath_gpio_name_entries[idx];
	} else {
		entry->syntax = NULL;
	}
}

SHELL_DYNAMIC_CMD_CREATE(ath_gpio_name, ath_gpio_name_get);

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_ath_gpio_switch_cmds,
	SHELL_CMD(get, NULL, "ath_gpio_switch get status", cmd_ath_gpio_status_get),
	SHELL_CMD_ARG(set, &ath_gpio_name,
		      "ath_gpio_switch set <ATH_GPIO_3|ATH_GPIO_4|all> <enable|disable> [perm]",
		      cmd_ath_gpio_status_set, 3, 1),
	SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(ath_gpio_switch, &sub_ath_gpio_switch_cmds, "ath_gpio switch command", NULL);
