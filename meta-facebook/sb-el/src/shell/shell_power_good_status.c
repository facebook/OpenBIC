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

#include <shell/shell.h>
#include <stdlib.h>
#include <stdio.h>
#include "plat_cpld.h"
#include "plat_class.h"
#include "shell_arke_power.h"

LOG_MODULE_REGISTER(shell_plat_power_good_status, LOG_LEVEL_INF);

#define POWER_GOOD_STATUS_COUNT power_good_status_table_for_steps_on_count
void show_power_good_status(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t last_offset = 0xFF;
	uint8_t reg_data = 0;

	for (int index = 0; index < PWRGD_MAX; index++) {
		for (int i = 0; i < POWER_GOOD_STATUS_COUNT; i++) {
			if (index != power_good_status_table_for_steps_on[i].index)
				continue;
			uint8_t offset = power_good_status_table_for_steps_on[i].cpld_offsets;
			uint8_t bit = power_good_status_table_for_steps_on[i].bit_loc;

			// if offset is different, read from CPLD
			if (offset != last_offset) {
				if (!plat_read_cpld(offset, &reg_data, 1)) {
					shell_error(shell, "Read CPLD offset 0x%x failed", offset);
					continue;
				}
				last_offset = offset;
			}

			uint8_t value = (reg_data >> bit) & 0x01;

			shell_print(shell, "%-20s %d",
				    power_good_status_table_for_steps_on[i].power_rail_name, value);
			if (get_asic_board_id() == ASIC_BOARD_ID_EVB &&
			    power_good_status_table_for_steps_on[i].index == PWRGD_P1V8_R) {
				//check pwrgd PWRGD_P1V8_AUX status is on
				if (!value) {
					shell_print(
						shell,
						"P1V8 is off, skip p3v3_osfp power good status check.");
				} else
					pwer_gd_get_status(shell);
			}
		}
	}
}

SHELL_CMD_REGISTER(power_good_status, NULL, "Show all the power good status",
		   show_power_good_status);
