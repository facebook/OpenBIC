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
#include <logging/log.h>
#include "sensor.h"
#include "plat_hook.h"

LOG_MODULE_REGISTER(plat_smbus_shell);

#define I2C_DEVICE_PREFIX "I2C_"
#define MAX_I2C_BYTES 31

static int8_t name2idx(const char *name)
{
	if (name == NULL)
		return -1;

	if (strncmp(name, I2C_DEVICE_PREFIX, strlen(I2C_DEVICE_PREFIX)) != 0)
		return -1;

	return strtol(name + strlen(I2C_DEVICE_PREFIX), NULL, 10);
}

static int cmd_repeat_read(const struct shell *shell, size_t argc, char **argv)
{
	// read first byte from device
	I2C_MSG msg = { 0 };
	msg.bus = name2idx(argv[1]);
	if (msg.bus == 0xff) {
		shell_error(shell, "Invalid bus name: %s", argv[1]);
		return -1;
	}

	msg.target_addr = strtol(argv[2], NULL, 16);
	int repeat_times = strtol(argv[5], NULL, 16);

	for (int i = 0; i < repeat_times; i++) {
		msg.tx_len = 1;
		msg.rx_len = strtol(argv[4], NULL, 16); //read_bytes
		msg.data[0] = strtol(argv[3], NULL, 16); //cmd
		if (i2c_master_read(&msg, 5)) {
			shell_error(shell, "Failed to read from bus %d device: %x", msg.bus,
				    msg.target_addr);
			return -1;
		}

		for (int j = 0; j < msg.rx_len; j++)
			shell_fprintf(shell, SHELL_NORMAL, "%02x ", msg.data[j]);
		shell_fprintf(shell, SHELL_NORMAL, "\n");
	}

	return 0;
}

static int cmd_block_read(const struct shell *shell, size_t argc, char **argv)
{
	// read first byte from device
	I2C_MSG msg = { 0 };
	msg.bus = name2idx(argv[1]);
	if (msg.bus == 0xff) {
		shell_error(shell, "Invalid bus name: %s", argv[1]);
		return -1;
	}

	msg.target_addr = strtol(argv[2], NULL, 16);
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = strtol(argv[3], NULL, 16); //cmd

	if (i2c_master_read(&msg, 5)) {
		shell_error(shell, "Failed to read from bus %d device: %x", msg.bus,
			    msg.target_addr);
		return -1;
	}

	// read again with the length from the first byte
	msg.tx_len = 1;
	msg.rx_len = msg.data[0] + 1;
	msg.data[0] = strtol(argv[3], NULL, 16); //cmd
	if (i2c_master_read(&msg, 5)) {
		shell_error(shell, "Failed to read from bus %d device: %x", msg.bus,
			    msg.target_addr);
		return -1;
	}

	shell_hexdump(shell, msg.data, msg.rx_len);
	return 0;
}

static int cmd_block_write(const struct shell *shell, size_t argc, char **argv)
{
	I2C_MSG msg = { 0 };
	msg.bus = name2idx(argv[1]);
	if (msg.bus == 0xff) {
		shell_error(shell, "Invalid bus name: %s", argv[1]);
		return -1;
	}

	int num_bytes = argc - 4; //byte count

	if (num_bytes > MAX_I2C_BYTES) {
		num_bytes = MAX_I2C_BYTES;
	}

	msg.target_addr = strtol(argv[2], NULL, 16);
	msg.tx_len = argc - 2;
	msg.data[0] = strtol(argv[3], NULL, 16); //cmd
	msg.data[1] = num_bytes;
	for (int i = 0; i < num_bytes; i++)
		msg.data[i + 2] = strtol(argv[4 + i], NULL, 16);

	if (i2c_master_write(&msg, 5)) {
		shell_error(shell, "Failed to write to bus %d device: %x", msg.bus,
			    msg.target_addr);
		return -1;
	}

	return 0;
}

static void device_name_get(size_t idx, struct shell_static_entry *entry)
{
	const struct device *dev = shell_device_lookup(idx, I2C_DEVICE_PREFIX);

	entry->syntax = (dev != NULL) ? dev->name : NULL;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;
}

SHELL_DYNAMIC_CMD_CREATE(dsub_device_name, device_name_get);

/* level 1 */
SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_smbus_cmds,
	SHELL_CMD_ARG(block_write, &dsub_device_name,
		      "smbus block_write     <bus> <devaddr> <cmd> [<byte1>, ...]", cmd_block_write,
		      5, MAX_I2C_BYTES),
	SHELL_CMD_ARG(block_read, &dsub_device_name, "smbus block_read      <bus> <devaddr> <cmd> ",
		      cmd_block_read, 4, 0),
	SHELL_CMD_ARG(repeat_read, &dsub_device_name,
		      "smbus repeat_read      <bus> <devaddr> <cmd> <read_bytes> <repeat_times>",
		      cmd_repeat_read, 6, 0),
	SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(smbus, &sub_smbus_cmds, "smbus command", NULL);
