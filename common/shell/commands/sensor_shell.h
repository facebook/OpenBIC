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

#ifndef SENSOR_SHELL_H
#define SENSOR_SHELL_H

#include <shell/shell.h>

/* According to IPMI specification Table 43, length of sensor name maximum is 16 bytes. */
#define MAX_SENSOR_NAME_LENGTH 32 // 31 bytes sensor name and 1 byte null character

enum SENSOR_ACCESS { SENSOR_READ, SENSOR_WRITE };

void cmd_sensor_cfg_list_all(const struct shell *shell, size_t argc, char **argv);
void cmd_sensor_cfg_get(const struct shell *shell, size_t argc, char **argv);
void cmd_control_sensor_polling(const struct shell *shell, size_t argc, char **argv);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_sensor_cmds,
			       SHELL_CMD(list_all, NULL, "List all SENSOR config.",
					 cmd_sensor_cfg_list_all),
			       SHELL_CMD(get, NULL, "Get SENSOR config", cmd_sensor_cfg_get),
			       SHELL_CMD(control_sensor_polling, NULL,
					 "Enable/Disable sensor polling",
					 cmd_control_sensor_polling),
			       SHELL_SUBCMD_SET_END);

#endif
