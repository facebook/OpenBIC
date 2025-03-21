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

#ifndef CPLD_SHELL_H
#define CPLD_SHELL_H

#include <shell/shell.h>

void cmd_cpld_dump(const struct shell *shell, size_t argc, char **argv);
void set_cpld_polling(const struct shell *shell, size_t argc, char **argv);
void get_cpld_polling(const struct shell *shell, size_t argc, char **argv);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_cpld_control_polling_cmds,
			       SHELL_CMD(set, NULL, "cpld polling set", set_cpld_polling),
			       SHELL_CMD(get, NULL, "cpld polling get", get_cpld_polling),
			       SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_cpld_cmd, SHELL_CMD(dump, NULL, "cpld dump", cmd_cpld_dump),
			       SHELL_CMD(control_polling, &sub_cpld_control_polling_cmds,
					 "cpld control polling", NULL),
			       SHELL_SUBCMD_SET_END);

#endif
