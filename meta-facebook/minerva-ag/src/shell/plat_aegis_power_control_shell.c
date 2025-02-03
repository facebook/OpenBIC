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

#include <zephyr.h>
#include <shell/shell.h>
#include <stdlib.h>
#include <logging/log.h>
#include "plat_isr.h"

LOG_MODULE_REGISTER(plat_aegis_power_control_shell, LOG_LEVEL_DBG);

int cmd_aegis_power_control(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 1) {
		shell_warn(shell, "Help: test aegis_power <on/off/cycle>");
		return -1;
	}
	if (!strcmp(argv[0], "on")) {
		if (!plat_power_control(1)) {
			shell_error(shell, "plat power control failed");
			return -1;
		}
	} else if (!strcmp(argv[0], "off")) {
		if (!plat_power_control(0)) {
			shell_error(shell, "plat power control failed");
			return -1;
		}
	} else if (!strcmp(argv[0], "cycle")) {
		if (!plat_power_control(0)) {
			shell_error(shell, "plat power control failed");
			return -1;
		}
		if (!plat_power_control(1)) {
			shell_error(shell, "plat power control failed");
			return -1;
		}
	} else {
		shell_error(shell, "plat power control not supported");
		return -1;
	}
	shell_print(shell, "plat power control finish");
	return 0;
}
