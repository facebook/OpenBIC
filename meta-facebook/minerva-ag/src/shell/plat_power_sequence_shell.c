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
#include "plat_hook.h"
#include "plat_i2c.h"

#define AEGIS_CPLD_ADDR (0x4C >> 1)

LOG_MODULE_REGISTER(plat_power_sequence_shell, LOG_LEVEL_DBG);

int cmd_power_sequence(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 1) {
		shell_warn(shell, "Help: power_sequence <power_up|power_down>");
		return -1;
	}

	power_sequence *power_sequence_table;
	size_t size;
	if (!strcmp(argv[0], "power_up")) {
		shell_print(shell, "UBC_ENABLE ->");
		power_sequence_table = power_sequence_on_table;
		size = power_sequence_on_table_size;
	} else if (!strcmp(argv[0], "power_down")) {
		shell_print(shell, "RST_ATH ->");
		power_sequence_table = power_sequence_off_table;
		size = power_sequence_off_table_size;
	} else {
		shell_print(shell, "Unsupport power_sequence: %s", argv[0]);
		return -1;
	}
	for (size_t i = 0; i < size; i++) {
		uint8_t data;
		uint8_t cpld_offset = power_sequence_table[i].cpld_offsets;

		if (!plat_i2c_read(I2C_BUS5, AEGIS_CPLD_ADDR, cpld_offset, &data, 1)) {
			LOG_ERR("Failed to read cpld register from cpld");
			continue;
			;
		}
		uint16_t during_time = data * 2;
		shell_print(shell, "            [%2d]%-50s %d ms", power_sequence_table[i].index,
			    power_sequence_table[i].power_rail_name, during_time);
	}
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_power_sequence_cmds,
			       SHELL_CMD(power_up, NULL, "power_sequence power_up command",
					 cmd_power_sequence),
			       SHELL_CMD(power_down, NULL, "power_sequence power_down command",
					 cmd_power_sequence),
			       SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(power_sequence, &sub_power_sequence_cmds, "power_sequence <power_up|power_down>",
		   NULL);
