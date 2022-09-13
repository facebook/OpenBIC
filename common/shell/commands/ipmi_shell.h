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

#ifndef IPMI_SHELL_H
#define IPMI_SHELL_H

#include <stdlib.h>
#include <shell/shell.h>

void cmd_ipmi_list(const struct shell *shell, size_t argc, char **argv);
void cmd_ipmi_raw(const struct shell *shell, size_t argc, char **argv);

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_ipmi_cmds, SHELL_CMD(scan, NULL, "Scanning all supported commands", cmd_ipmi_list),
	SHELL_CMD(raw, NULL, "Send raw command", cmd_ipmi_raw), SHELL_SUBCMD_SET_END);

#endif
