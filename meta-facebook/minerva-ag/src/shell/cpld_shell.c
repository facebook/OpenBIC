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
#include "hal_i2c.h"
#include "plat_i2c.h"
#include "plat_event.h"
#include <stdlib.h>

#define AEGIS_CPLD_ADDR (0x4C >> 1)
#define I2C_BUS_CPLD I2C_BUS5

void cmd_cpld_dump(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_warn(shell, "Help: test cpld dump <offset> <length>");
		return;
	}

	int cpld_offset = strtol(argv[1], NULL, 16);
	int cpld_length = strtol(argv[2], NULL, 16);

	if (cpld_offset < 0x00 || cpld_offset > 0xFF) {
		shell_error(shell, "<offset> value is out of range!");
		return;
	}

	if (cpld_length <= 0x00 || cpld_length > 0xFF) {
		shell_error(shell, "<length> value is out of range!");
		return;
	}

	uint8_t retry = 3;
	I2C_MSG i2c_msg = { 0 };

	i2c_msg.bus = I2C_BUS_CPLD;
	i2c_msg.target_addr = AEGIS_CPLD_ADDR;
	i2c_msg.rx_len = cpld_length;
	i2c_msg.tx_len = 1;
	i2c_msg.data[0] = cpld_offset;

	if (i2c_master_read(&i2c_msg, retry)) {
		shell_error(shell, "Failed to test cpld dump");
	} else {
		shell_print(shell, "bus %d, addr 0x%x, offset 0x%x, len 0x%x.", i2c_msg.bus,
			    i2c_msg.target_addr, cpld_offset, i2c_msg.rx_len);

		shell_hexdump(shell, i2c_msg.data, i2c_msg.rx_len);
		shell_print(shell, "");
	}
	return;
}

void set_cpld_polling(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_warn(shell, "Help: test cpld control_polling set <enable>");
		return;
	}

	uint8_t polling_enable = strtol(argv[1], NULL, 10);

	if (polling_enable < 0 || polling_enable > 1) {
		shell_error(shell, "polling_enable value is out of range!");
		return;
	}

	set_cpld_polling_enable_flag(polling_enable);
}

void get_cpld_polling(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 1) {
		shell_warn(shell, "Help: test cpld control_polling get");
		return;
	}

	shell_print(shell, "cpld polling = %d", get_cpld_polling_enable_flag());
}
