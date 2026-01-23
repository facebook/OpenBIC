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
#include <shell/shell.h>

#include "plat_pldm_sensor.h"
#include "plat_cpld.h"
#include "plat_class.h"
#include "plat_adc.h"

// test command
void cmd_test(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "Hello world!");
}

void cmd_read_raw(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t sensor_id = strtoul(argv[1], NULL, 16);
	uint8_t offset = strtoul(argv[2], NULL, 16);
	uint8_t len = strtoul(argv[3], NULL, 10);

	if (!len)
		len = 1;
	uint8_t data[len];
	memset(data, 0, len);

	if ((sensor_id == 0) || (sensor_id >= SENSOR_NUM_NUMBERS)) {
		if (!plat_read_cpld(offset, data, 1)) {
			shell_warn(shell, "cpld read 0x%02x fail", offset);
			return;
		}
	} else {
		if (!get_raw_data_from_sensor_id(sensor_id, offset, data, len)) {
			shell_warn(shell, "sensor_id 0x%02x read 0x%02x fail", sensor_id, offset);
			return;
		}
	}

	shell_hexdump(shell, data, len);
	shell_print(shell, "");
}

void cmd_cpld_dump(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_warn(shell, "Help: test cpld dump <offset> <length>");
		return;
	}

	uint8_t offset = strtoul(argv[1], NULL, 16);
	uint8_t len = strtoul(argv[2], NULL, 10);

	if (!len)
		len = 1;
	uint8_t data[len];
	memset(data, 0, len);

	if (!plat_read_cpld(offset, data, len)) {
		shell_warn(shell, "cpld read 0x%02x fail", offset);
		return;
	}

	shell_hexdump(shell, data, len);
	shell_print(shell, "");
}
void cmd_cpld_write(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_warn(shell, "Help: test cpld write <offset> <data>");
		return;
	}

	uint8_t offset = strtoul(argv[1], NULL, 16);
	uint8_t data = strtoul(argv[2], NULL, 16);

	if (!plat_write_cpld(offset, &data)) {
		shell_warn(shell, "cpld write 0x%02x fail", offset);
		return;
	}

	shell_warn(shell, "cpld write %02x to offset %02x", data, offset);
}

void cmd_info(const struct shell *shell, size_t argc, char **argv)
{
	static const char *const vr_module_str[] = {
		[VR_MODULE_MPS] = "MPS",
		[VR_MODULE_RNS] = "RNS",
	};

	static const char *const ubc_module_str[] = {
		[UBC_MODULE_DELTA] = "DELTA",
		[UBC_MODULE_MPS] = "MPS",
		[UBC_MODULE_FLEX] = "FLEX",
		[UBC_MODULE_LUXSHARE] = "LUXSHARE",
	};

	static const char *const asic_board_id_str[] = {
		[ASIC_BOARD_ID_RSVD1] = "RSVD1",
		[ASIC_BOARD_ID_RSVD2] = "RSVD2",
		[ASIC_BOARD_ID_RAINBOW] = "RAINBOW",
		[ASIC_BOARD_ID_EVB] = "EVB",
	};

	uint8_t vr = get_vr_module();
	uint8_t ubc = get_ubc_module();
	uint8_t board_id = get_asic_board_id();
	uint8_t board_rev = get_board_rev_id();
	uint8_t adc_idx = get_adc_type();

	shell_warn(shell, "vr module: %s",
		   (vr < VR_MODULE_UNKNOWN) ? vr_module_str[vr] : "UNKNOWN");
	shell_warn(shell, "ubc module: %s",
		   (ubc < UBC_MODULE_UNKNOWN) ? ubc_module_str[ubc] : "UNKNOWN");
	shell_warn(shell, "mmc slot: %d", get_mmc_slot() + 1);
	shell_warn(shell, "asic board id: %s",
		   (board_id < ASIC_BOARD_ID_UNKNOWN) ? asic_board_id_str[board_id] : "UNKNOWN");
	shell_warn(shell, "asic board rev id: %d", board_rev);
	shell_warn(shell, "adc idx: %d", adc_idx);
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_cpld_cmds, SHELL_CMD(dump, NULL, "cpld dump", cmd_cpld_dump),
			       SHELL_CMD(write, NULL, "write cpld register", cmd_cpld_write),
			       SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_test_cmds, SHELL_CMD(test, NULL, "test command", cmd_test),
			       SHELL_CMD(read_raw, NULL, "read raw data test command",
					 cmd_read_raw),
			       SHELL_CMD(cpld, &sub_cpld_cmds, "cpld commands", NULL),
			       SHELL_CMD(info, NULL, "info commands", cmd_info),
			       SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(test, &sub_test_cmds, "Test commands", NULL);
