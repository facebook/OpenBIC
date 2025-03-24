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

void cmd_thermaltrip_status_get(const struct shell *shell, size_t argc, char **argv)
{
	I2C_MSG msg = { 0 };
	msg.bus = I2C_BUS_CPLD;
	msg.target_addr = AEGIS_CPLD_ADDR;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = CPLD_THERMALTRIP_SWITCH_ADDR;

	if (i2c_master_read(&msg, 3)) {
		shell_error(shell, "Failed to read from bus %d device: %x", msg.bus,
			    msg.target_addr);
		return;
	}

	shell_print(shell, "thermaltrip switch %s", (msg.data[0]) ? "enable" : "disable");
}

static uint8_t switch_set(const struct shell *shell, bool is_enable)
{
	I2C_MSG msg = { 0 };

	msg.bus = I2C_BUS_CPLD;
	msg.target_addr = AEGIS_CPLD_ADDR;
	msg.tx_len = 2;
	msg.data[0] = CPLD_THERMALTRIP_SWITCH_ADDR;
	msg.data[1] = (is_enable) ? 1 : 0;

	if (i2c_master_write(&msg, 3)) {
		shell_error(shell, "Failed to write to bus %d device: %x", msg.bus,
			    msg.target_addr);
		return -1;
	}

	return 0;
}

void cmd_thermaltrip_status_en(const struct shell *shell, size_t argc, char **argv)
{
	switch_set(shell, true);
}

void cmd_thermaltrip_status_dis(const struct shell *shell, size_t argc, char **argv)
{
	switch_set(shell, false);
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_thermaltrip_set_cmd,
	SHELL_CMD(enable, NULL, "enable", cmd_thermaltrip_status_en),
	SHELL_CMD(disable, NULL, "disable", cmd_thermaltrip_status_dis),
	SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_thermaltrip_switch_cmds,
	SHELL_CMD(get, NULL, "get thermaltrip switch status", cmd_thermaltrip_status_get),
	SHELL_CMD(set, &sub_thermaltrip_set_cmd, "set thermaltrip switch status", NULL),
	SHELL_SUBCMD_SET_END);

	/* Root of command test */
SHELL_CMD_REGISTER(thermaltrip_switch, &sub_thermaltrip_switch_cmds,
	"thermal trip switch command", NULL);
