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

#ifndef INFO_SHELL_H
#define INFO_SHELL_H

#include <shell/shell.h>

int cmd_info_print(const struct shell *shell, size_t argc, char **argv);

/* Sensor sub command */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_info_cmds,
			       SHELL_CMD(all, NULL, "List all platform info.", cmd_info_print),
			       SHELL_SUBCMD_SET_END);

#endif
