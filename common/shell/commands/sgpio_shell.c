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

#include "sgpio_shell.h"

#if defined(CONFIG_SGPIO_NPCM4XX)

#include "hal_gpio.h"
#include <drivers/gpio.h>
#include <stdio.h>
#include <string.h>

enum SGPIO_ACCESS { SGPIO_READ, SGPIO_WRITE };

static int sgpio_access_cfg(const struct shell *shell, int idx, enum SGPIO_ACCESS mode, int *data)
{
	if (!shell) {
		return 1;
	}

	if (idx >= SGPIO_CFG_SIZE || idx < 0) {
		shell_error(shell, "sgpio_access_cfg - sgpio index out of bound!");
		return 1;
	}

	switch (mode) {
	case SGPIO_READ:
		if (sgpio_cfg[idx].is_init == DISABLE) {
			return 1;
		}

		char *pin_prop = (sgpio_cfg[idx].property == OPEN_DRAIN) ? "OD" : "PP";
		char *pin_dir = (sgpio_cfg[idx].direction == GPIO_INPUT) ? "input" : "output";

		int val = sgpio_get(idx);
		if (val == 0 || val == 1) {
			shell_print(shell, "[%-3d] %-35s: %-3s | %-6s | %d", idx, sgpio_name[idx],
				    pin_prop, pin_dir, val);
		} else {
			shell_error(shell, "[%-3d] %-35s: %-3s | %-6s | err[%d]", idx,
				    sgpio_name[idx], pin_prop, pin_dir, val);
			return 1;
		}

		break;

	case SGPIO_WRITE:
		if (!data) {
			shell_error(shell, "sgpio_access_cfg - SGPIO_WRITE value empty!");
			return 1;
		}

		if (*data != 0 && *data != 1) {
			shell_error(
				shell,
				"sgpio_access_cfg - SGPIO_WRITE value should only accept 0 or 1!");
			return 1;
		}

		if (sgpio_set(idx, *data)) {
			shell_error(shell, "sgpio_access_cfg - SGPIO_WRITE failed!");
			return 1;
		}

		break;

	default:
		shell_error(shell, "sgpio_access_cfg - No such mode %d!", mode);
		return 1;
	}

	return 0;
}

void cmd_sgpio_cfg_list_all(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 1 && argc != 2) {
		shell_warn(shell, "Help: platform sgpio list_all <key_word(optional)>");
		return;
	}

	const char *key_word = NULL;
	if (argc == 2)
		key_word = argv[1];

	for (int idx = 0; idx < SGPIO_CFG_SIZE; idx++) {
		if (key_word && !strstr(sgpio_name[idx], key_word))
			continue;
		sgpio_access_cfg(shell, idx, SGPIO_READ, NULL);
	}

	return;
}

void cmd_sgpio_cfg_get(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_warn(shell, "Help: platform sgpio get <sgpio_idx>");
		return;
	}

	int sgpio_index = strtol(argv[1], NULL, 10);
	if (sgpio_access_cfg(shell, sgpio_index, SGPIO_READ, NULL))
		shell_error(shell, "sgpio[%d] get failed!", sgpio_index);

	return;
}

void cmd_sgpio_cfg_set_val(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_warn(shell, "Help: platform sgpio set val <sgpio_idx> <data>");
		return;
	}

	int sgpio_index = strtol(argv[1], NULL, 10);
	int data = strtol(argv[2], NULL, 10);

	if (sgpio_access_cfg(shell, sgpio_index, SGPIO_WRITE, &data))
		shell_error(shell, "sgpio[%d] --> %d ,failed!", sgpio_index, data);
	else
		shell_print(shell, "sgpio[%d] --> %d ,success!", sgpio_index, data);

	return;
}

#endif /* CONFIG_SGPIO_NPCM4XX */
