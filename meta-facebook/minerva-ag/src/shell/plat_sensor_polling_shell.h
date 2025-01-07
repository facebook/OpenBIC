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

#ifndef PLAT_SENSOR_POLLING_SHELL_H
#define PLAT_SENSOR_POLLING_SHELL_H

#include <shell/shell.h>

void cmd_set_plat_sensor_polling_all(const struct shell *shell, size_t argc, char **argv);
void cmd_set_plat_sensor_polling_ubc(const struct shell *shell, size_t argc, char **argv);
void cmd_set_plat_sensor_polling_vr(const struct shell *shell, size_t argc, char **argv);
void cmd_set_plat_sensor_polling_temp(const struct shell *shell, size_t argc, char **argv);
void cmd_get_plat_sensor_polling_all(const struct shell *shell, size_t argc, char **argv);
void cmd_get_plat_sensor_polling_ubc(const struct shell *shell, size_t argc, char **argv);
void cmd_get_plat_sensor_polling_vr(const struct shell *shell, size_t argc, char **argv);
void cmd_get_plat_sensor_polling_temp(const struct shell *shell, size_t argc, char **argv);

/* Sub-command Level 3 of command test */
SHELL_STATIC_SUBCMD_SET_CREATE(
	cmd_set_plat_sensor_polling,
	SHELL_CMD(all, NULL, "set platform sensor polling all", cmd_set_plat_sensor_polling_all),
	SHELL_CMD(ubc, NULL, "set platform sensor polling ubc", cmd_set_plat_sensor_polling_ubc),
	SHELL_CMD(vr, NULL, "set platform sensor polling vr", cmd_set_plat_sensor_polling_vr),
	SHELL_CMD(temp, NULL, "set platform sensor polling temp", cmd_set_plat_sensor_polling_temp),
	SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(cmd_get_plat_sensor_polling,
			       SHELL_CMD(all, NULL, "get platform sensor polling all",
					 cmd_get_plat_sensor_polling_all),
			       SHELL_SUBCMD_SET_END);

/* Sub-command Level 2 of command test */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_plat_sensor_polling_cmd,
			       SHELL_CMD(set, &cmd_set_plat_sensor_polling,
					 "set platform sensor polling", NULL),
			       SHELL_CMD(get, &cmd_get_plat_sensor_polling,
					 "get platform sensor polling", NULL),
			       SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(set_sensor_polling, &sub_plat_sensor_polling_cmd,
		   "Disable/Enable sensor polling for group of sensors", NULL);

#endif
