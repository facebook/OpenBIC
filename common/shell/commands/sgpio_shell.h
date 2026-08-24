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

#ifndef SGPIO_SHELL_H
#define SGPIO_SHELL_H

#include <stdlib.h>
#include <shell/shell.h>
#include "hal_gpio.h"

#if defined(CONFIG_SGPIO_NPCM4XX)

void cmd_sgpio_cfg_list_all(const struct shell *shell, size_t argc, char **argv);
void cmd_sgpio_cfg_get(const struct shell *shell, size_t argc, char **argv);
void cmd_sgpio_cfg_set_val(const struct shell *shell, size_t argc, char **argv);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_sgpio_set_cmds,
			       SHELL_CMD(val, NULL, "Set pin value.", cmd_sgpio_cfg_set_val),
			       SHELL_SUBCMD_SET_END);

/* SGPIO sub commands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_sgpio_cmds,
			       SHELL_CMD(list_all, NULL, "List all SGPIO config.",
					 cmd_sgpio_cfg_list_all),
			       SHELL_CMD(get, NULL, "Get SGPIO config", cmd_sgpio_cfg_get),
			       SHELL_CMD(set, &sub_sgpio_set_cmds, "Set SGPIO config", NULL),
			       SHELL_SUBCMD_SET_END);

#endif /* CONFIG_SGPIO_NPCM4XX */

#endif
