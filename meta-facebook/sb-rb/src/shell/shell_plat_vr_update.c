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
#include "plat_cpld.h"

LOG_MODULE_REGISTER(plat_vr_update_shell, LOG_LEVEL_DBG);

#define CPLD_ADDR (0x4C >> 1)
#define I2C_BUS_CPLD I2C_BUS11

void cmd_vr_update_status_get(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t read_back_data = 0;
	if (!plat_read_cpld(VR_UPDATE_REG, &read_back_data, 1)) {
		shell_error(shell, "Failed to read vr_update switch status");
		return;
	}
	// bit 0 is for vr update switch, 1: enable, 0: disable
	read_back_data = read_back_data & 0x01;
	shell_print(shell, "vr_update switch %s", (read_back_data) ? "enable" : "disable");
}
void set_vr_update_switch_status(bool enable)
{
	if (!set_cpld_bit(VR_UPDATE_REG, 0, enable)) {
		LOG_ERR("Failed to set vr_update switch status");
		return;
	}
}
void cmd_vr_update_status_en(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "vr_update switch set to enable");
	set_vr_update_switch_status(true);
}

void cmd_vr_update_status_dis(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "vr_update switch set to disable");
	set_vr_update_switch_status(false);
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_plat_vr_update_set_cmd,
			       SHELL_CMD(enable, NULL, "enable", cmd_vr_update_status_en),
			       SHELL_CMD(disable, NULL, "disable", cmd_vr_update_status_dis),
			       SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_vr_update_switch_cmds,
			       SHELL_CMD(get, NULL, "get vr_update switch status",
					 cmd_vr_update_status_get),
			       SHELL_CMD(set, &sub_plat_vr_update_set_cmd,
					 "set vr_update switch status", NULL),
			       SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(vr_update_switch, &sub_vr_update_switch_cmds, "vr_update_switch command", NULL);
