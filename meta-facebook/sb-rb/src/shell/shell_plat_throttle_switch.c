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
#include "plat_user_setting.h"
#include "plat_fru.h"
#include "shell_plat_throttle_switch.h"

LOG_MODULE_REGISTER(plat_throttle_switch_shell, LOG_LEVEL_DBG);

#define CPLD_THROTTLE_SWITCH_ADDR 0x25
#define EEPROM_MAX_WRITE_TIME 5

typedef struct throttle_user_settings_struct {
	uint8_t throttle_user_setting_value;
} throttle_user_settings_struct;

throttle_user_settings_struct throttle_user_settings = { 0xFF };

bool set_user_settings_throttle_to_eeprom(void *throttle_user_settings, uint8_t data_length)
{
	CHECK_NULL_ARG_WITH_RETURN(throttle_user_settings, false);

	/* write the throttle_user_settings to eeprom */

	if (!plat_eeprom_write(THROTTLE_USER_SETTINGS_OFFSET, (uint8_t *)throttle_user_settings,
			       data_length)) {
		LOG_ERR("throttle user settings failed to write into eeprom");
		return false;
	}

	k_msleep(EEPROM_MAX_WRITE_TIME);

	return true;
}

bool set_throttle_user_settings(uint8_t *throttle_status_reg, bool is_perm)
{
	CHECK_NULL_ARG_WITH_RETURN(throttle_status_reg, false);

	if (!plat_write_cpld(CPLD_THROTTLE_SWITCH_ADDR, throttle_status_reg)) {
		LOG_ERR("Failed to write throttle to cpld error");
		return false;
	}

	if (is_perm) {
		throttle_user_settings.throttle_user_setting_value = *throttle_status_reg;

		if (!set_user_settings_throttle_to_eeprom(&throttle_user_settings,
							  sizeof(throttle_user_settings))) {
			LOG_ERR("Failed to write throttle to eeprom error");
			return false;
		}
	}
	return true;
}

static bool get_throttle_status(const struct shell *shell, uint8_t *throttle_status_reg)
{
	CHECK_NULL_ARG_WITH_RETURN(throttle_status_reg, false);

	if (!plat_read_cpld(CPLD_THROTTLE_SWITCH_ADDR, throttle_status_reg, 1)) {
		LOG_ERR("Failed to read throttle from cpld error");
		return false;
	}

	return true;
}

void cmd_throttle_status_get(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t throttle_status_reg = 0;
	if (!get_throttle_status(shell, &throttle_status_reg)) {
		shell_print(shell, "Can't get throttle status");
		return;
	}

	bool mmc_lv1_switch_enabled = (throttle_status_reg & BIT(3)) != 0;
	bool mmc_lv2_switch_enabled = (throttle_status_reg & BIT(2)) != 0;
	bool mmc_lv3_switch_enabled = (throttle_status_reg & BIT(1)) != 0;

	shell_print(shell, "Throttle MMC Level 1 switch: %s",
		    mmc_lv1_switch_enabled ? "enabled" : "disabled");
	shell_print(shell, "Throttle MMC Level 2 switch: %s",
		    mmc_lv2_switch_enabled ? "enabled" : "disabled");
	shell_print(shell, "Throttle MMC Level 3 switch: %s",
		    mmc_lv3_switch_enabled ? "enabled" : "disabled");
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

	if (!strcmp(name, "mmc_lv1_switch")) {
		throttle_status_reg |= BIT(3);
	} else if (!strcmp(name, "mmc_lv2_switch")) {
		throttle_status_reg |= BIT(2);
	} else if (!strcmp(name, "mmc_lv3_switch")) {
		throttle_status_reg |= BIT(1);
	} else if (!strcmp(name, "all")) {
		throttle_status_reg |= (BIT(3) | BIT(2) | BIT(1));
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

	if (!strcmp(name, "mmc_lv1_switch")) {
		throttle_status_reg &= ~BIT(3);
	} else if (!strcmp(name, "mmc_lv2_switch")) {
		throttle_status_reg &= ~BIT(2);
	} else if (!strcmp(name, "mmc_lv3_switch")) {
		throttle_status_reg &= ~BIT(1);
	} else if (!strcmp(name, "all")) {
		throttle_status_reg &= ~(BIT(3) | BIT(2) | BIT(1));
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
	{ .syntax = "mmc_lv1_switch",
	  .handler = NULL,
	  .subcmd = &throttle_status,
	  .help = "name: mmc_lv1_switch" },
	{ .syntax = "mmc_lv2_switch",
	  .handler = NULL,
	  .subcmd = &throttle_status,
	  .help = "name: mmc_lv2_switch" },
	{ .syntax = "mmc_lv3_switch",
	  .handler = NULL,
	  .subcmd = &throttle_status,
	  .help = "name: mmc_lv3_switch" },
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
		      "throttle_switch set <lv1 |lv2 |lv3 |all> <enable>|<disable> [perm]", NULL, 3,
		      1),
	SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(throttle_switch, &sub_throttle_switch_cmds, "thermal trip switch command", NULL);
