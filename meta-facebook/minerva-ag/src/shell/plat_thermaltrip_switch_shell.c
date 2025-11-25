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

LOG_MODULE_REGISTER(plat_thermaltrip_switch_shell, LOG_LEVEL_DBG);

#define AEGIS_CPLD_ADDR (0x4C >> 1)
#define I2C_BUS_CPLD I2C_BUS5
#define CPLD_THERMALTRIP_SWITCH_ADDR 0x3D

static bool get_thermaltrip_status(const struct shell *shell, uint8_t *thermaltrip_status_reg)
{
	CHECK_NULL_ARG_WITH_RETURN(thermaltrip_status_reg, false);

	I2C_MSG msg = { 0 };
	msg.bus = I2C_BUS_CPLD;
	msg.target_addr = AEGIS_CPLD_ADDR;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = CPLD_THERMALTRIP_SWITCH_ADDR;

	if (i2c_master_read(&msg, 3)) {
		shell_error(shell, "Failed to read from bus %d device: %x", msg.bus,
			    msg.target_addr);
		return false;
	}

	*thermaltrip_status_reg = msg.data[0];
	return true;
}

void cmd_thermaltrip_status_get(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t thermaltrip_status_reg = 0;
	if (!get_thermaltrip_status(shell, &thermaltrip_status_reg)) {
		shell_print(shell, "Can't get thermaltrip status");
		return;
	}

	bool thermaltrip_enabled = (thermaltrip_status_reg & THERMALTRIP_BIT) != 0;

	shell_print(shell, "thermaltrip switch %s", (thermaltrip_enabled) ? "enable" : "disable");
}

static int cmd_thermaltrip_status_set(const struct shell *shell, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_error(shell, "Usage: thermaltrip_switch set <enable|disable> [perm]");
		return -1;
	}

	const char *action = argv[1];
	bool is_perm = false;

	if (argc == 3) {
		if (!strcmp(argv[2], "perm")) {
			is_perm = true;
		} else {
			shell_error(shell, "The last argument must be <perm>");
			return -1;
		}
	}

	uint8_t thermaltrip_status_reg = 0;
	if (!get_thermaltrip_status(shell, &thermaltrip_status_reg)) {
		shell_print(shell, "Can't get thermaltrip status");
		return -1;
	}

	bool enable = !strcmp(action, "enable");
	bool disable = !strcmp(action, "disable");
	if (!enable && !disable) {
		shell_error(shell, "Invalid action: %s (expected enable|disable)", action);
		return -1;
	}

	if (enable)
		thermaltrip_status_reg |= THERMALTRIP_BIT;
	else
		thermaltrip_status_reg &= ~THERMALTRIP_BIT;

	if (!set_thermaltrip_user_settings(&thermaltrip_status_reg, is_perm)) {
		shell_error(shell, "Failed to set thermaltrip %s %s", argv[1],
			    (argc == 3) ? argv[2] : "");
		return -1;
	}

	shell_info(shell, "thermaltrip set %s %s finish", argv[1], (argc == 3) ? argv[2] : "");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_thermaltrip_switch_cmds,
			       SHELL_CMD(get, NULL, "get thermaltrip switch status",
					 cmd_thermaltrip_status_get),
			       SHELL_CMD_ARG(set, NULL,
					     "thermaltrip_switch set <enable|disable> [perm]",
					     cmd_thermaltrip_status_set, 2, 1),
			       SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(thermaltrip_switch, &sub_thermaltrip_switch_cmds, "thermal trip switch command",
		   NULL);
