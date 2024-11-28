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
#include "plat_log.h"
#include "plat_fru.h"

void cmd_set_event(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_warn(shell, "Help: test log set_event <error_code_1> <error_code_2> ");
		shell_warn(shell, "error_code_1: high byte, error_code_2: low byte");
		return;
	}

	uint16_t error_code = ((strtol(argv[1], NULL, 16)) << 8) | (strtol(argv[2], NULL, 16));
	shell_print(shell, "Generate error code: 0x%x", error_code);

	error_log_event(error_code, LOG_ASSERT);

	return;
}

void cmd_log_dump(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_warn(shell, "Help: test log log_dump <order>, order start from 1");
		return;
	}

	int order = strtol(argv[1], NULL, 10);

	if (order < 1) {
		shell_warn(shell, "Help: test log log_dump <order>, order start from 1");
		return;
	}

	uint8_t log_data[128] = { 0 };
	plat_log_read(log_data, AEGIS_FRU_LOG_SIZE, order);

	shell_hexdump(shell, log_data, sizeof(uint8_t) * AEGIS_FRU_LOG_SIZE);

	return;
}

void cmd_test_read(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 4) {
		shell_warn(shell, "Help: test log test_read <offset_1> <offset_2> <length>");
		return;
	}

	uint16_t offset = ((strtol(argv[1], NULL, 16)) << 8) | (strtol(argv[2], NULL, 16));
	printf("offset = 0x%04X\n", offset);

	int length = strtol(argv[3], NULL, 16);

	uint8_t log_data[128] = { 0 };
	plat_eeprom_read(offset, log_data, length);
	printf("AEGIS_FRU_LOG_SIZE = %d\n", AEGIS_FRU_LOG_SIZE);

	shell_hexdump(shell, log_data, sizeof(uint8_t) * AEGIS_FRU_LOG_SIZE);

	return;
}

void cmd_log_clear(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 1) {
		shell_warn(shell, "Help: test log log_clear");
		return;
	}

	k_msleep(1000);

	plat_clear_log();
	shell_print(shell, "plat_clear_log finished!");

	return;
}
