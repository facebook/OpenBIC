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

#ifndef POWER_SHELL_H
#define POWER_SHELL_H

#include <stdlib.h>
#include <shell/shell.h>

struct device_arr {
	uint8_t enable;
	char *name;
};

enum device_id {
	DEVICE_HOST,
	DEVICE_BMC,
	MAX_DEVICE_COUNT,
};

void cmd_power_status(const struct shell *shell, size_t argc, char **argv);
void cmd_power_control(const struct shell *shell, size_t argc, char **argv);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_power_cmds,
			       SHELL_CMD(status, NULL, "List power status", cmd_power_status),
			       SHELL_CMD(control, NULL, "Power control", cmd_power_control),
			       SHELL_SUBCMD_SET_END);

#endif
