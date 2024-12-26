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
#include <string.h>
#include <stdio.h>
#include <zephyr.h>
#include "plat_hook.h"
#include "plat_uart_pwrevent_shell.h"

void cmd_enable_or_disable_uart_pwrevent(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 1) {
		shell_warn(shell, "Help: uart_pwrevent enable|disable");
		return;
	}

	char is_enable_string[16] = { 0 };
	snprintf(is_enable_string, sizeof(is_enable_string), "%s", argv[0]);

	if (strcmp(is_enable_string, "enable") == 0) {
		set_uart_power_event_is_enable(true);
		shell_print(shell, "uart_pwrevent enable success!");

	} else if (strcmp(is_enable_string, "disable") == 0) {
		set_uart_power_event_is_enable(false);
		shell_print(shell, "uart_pwrevent disable success!");
	} else {
		shell_error(shell, "Type wrong sub command, Help: uart_pwrevent enable|disable");
	}

	return;
}

/* Sub-command Level 1 of command uart-pwrevent */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_uart_pwrevent_cmds,
			       SHELL_CMD(enable, NULL, "enable_uart_pwrevent",
					 cmd_enable_or_disable_uart_pwrevent),
			       SHELL_CMD(disable, NULL, "disable_uart_pwrevent",
					 cmd_enable_or_disable_uart_pwrevent),
			       SHELL_SUBCMD_SET_END);

/* Root of command uart-pwrevent */
SHELL_CMD_REGISTER(uart_pwrevent, &sub_uart_pwrevent_cmds, "uart-pwrevent commands for AG", NULL);
