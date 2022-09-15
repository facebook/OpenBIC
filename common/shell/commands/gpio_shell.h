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

#ifndef GPIO_SHELL_H
#define GPIO_SHELL_H

#include <stdlib.h>
#include <shell/shell.h>
#include <drivers/gpio.h>

void cmd_gpio_cfg_list_group(const struct shell *shell, size_t argc, char **argv);
void cmd_gpio_cfg_list_all(const struct shell *shell, size_t argc, char **argv);
void cmd_gpio_cfg_get(const struct shell *shell, size_t argc, char **argv);
void cmd_gpio_cfg_set_val(const struct shell *shell, size_t argc, char **argv);
void cmd_gpio_cfg_set_int_type(const struct shell *shell, size_t argc, char **argv);
void cmd_gpio_muti_fn_ctl_list(const struct shell *shell, size_t argc, char **argv);
void device_gpio_name_get(size_t idx, struct shell_static_entry *entry);

SHELL_DYNAMIC_CMD_CREATE(gpio_device_name, device_gpio_name_get);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_gpio_set_cmds,
			       SHELL_CMD(val, NULL, "Set pin value.", cmd_gpio_cfg_set_val),
			       SHELL_CMD(int_type, NULL, "Set interrupt pin type.",
					 cmd_gpio_cfg_set_int_type),
			       SHELL_SUBCMD_SET_END);

/* GPIO sub commands */
SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_gpio_cmds,
	SHELL_CMD(list_group, &gpio_device_name, "List all GPIO config from certain group.",
		  cmd_gpio_cfg_list_group),
	SHELL_CMD(list_all, NULL, "List all GPIO config.", cmd_gpio_cfg_list_all),
	SHELL_CMD(get, NULL, "Get GPIO config", cmd_gpio_cfg_get),
	SHELL_CMD(set, &sub_gpio_set_cmds, "Set GPIO config", NULL),
	SHELL_CMD(multifnctl, NULL, "List all GPIO multi-function control regs.",
		  cmd_gpio_muti_fn_ctl_list),
	SHELL_SUBCMD_SET_END);

#endif
