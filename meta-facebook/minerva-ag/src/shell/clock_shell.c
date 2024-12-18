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
#include <stdio.h>
#include <string.h>
#include <zephyr.h>
#include <logging/log.h>
#include "libutil.h"
#include "plat_i2c.h"
#include "clock_shell.h"

LOG_MODULE_REGISTER(clock_shell);

#define CLK_GEN_312M_ADDR (0x12 >> 1)
#define CLK_GEN_100M_ADDR (0xD0 >> 1)
#define CLK_BUF_U471_ADDR (0xCE >> 1)
#define CLK_BUF_U519_ADDR (0xD8 >> 1)

clock_compnt_mapping clock_compnt_mapping_table[] = {
	{ CLKGEN_312M, CLK_GEN_312M_ADDR, I2C_BUS1 },
	{ CLKBUF_100M_U471, CLK_BUF_U471_ADDR, I2C_BUS1 },
	{ CLKBUF_100M_U519, CLK_BUF_U519_ADDR, I2C_BUS1 },
	{ CLKGEN_100M, CLK_GEN_100M_ADDR, I2C_BUS1 },
};

int find_clock_address_and_bus_by_clock_name_index(uint8_t clock_index, uint8_t *addr, uint8_t *bus)
{
	CHECK_NULL_ARG_WITH_RETURN(addr, -1);
	CHECK_NULL_ARG_WITH_RETURN(bus, -1);

	for (uint8_t i = 0; i < ARRAY_SIZE(clock_compnt_mapping_table); i++) {
		if (clock_compnt_mapping_table[i].clock_name_index == clock_index) {
			*addr = clock_compnt_mapping_table[i].addr;
			*bus = clock_compnt_mapping_table[i].bus;
			return 0;
		}
	}

	return -1;
}

void cmd_set_clock(const struct shell *shell, size_t argc, char **argv)
{
	if (argc < 4) {
		shell_warn(shell, "Help: set <clock> <reg-offset> <value>");
		return;
	}

	int clock_index = 0;
	char clock_string[20] = { 0 };

	snprintf(clock_string, sizeof(clock_string), "%s", argv[1]);

	if (strcmp(clock_string, "CLKGEN_312M") == 0) {
		clock_index = CLKGEN_312M;
	} else if (strcmp(clock_string, "CLKBUF_100M_U471") == 0) {
		clock_index = CLKBUF_100M_U471;
	} else if (strcmp(clock_string, "CLKBUF_100M_U519") == 0) {
		clock_index = CLKBUF_100M_U519;
	} else if (strcmp(clock_string, "CLKGEN_100M") == 0) {
		clock_index = CLKGEN_100M;
	} else {
		shell_error(shell, "Type wrong clock name");
		return;
	}

	int result = 0;
	uint8_t addr = 0;
	uint8_t bus = 0;
	uint8_t offset = 0;

	offset = strtol(argv[2], NULL, 16);

	result = find_clock_address_and_bus_by_clock_name_index(clock_index, &addr, &bus);
	if (result != 0) {
		shell_error(shell, "Can't find clock address and bus by clock name");
		return;
	}

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.tx_len = argc - 2;
	i2c_msg.data[0] = offset;

	if (i2c_msg.tx_len > 1) {
		for (int i = 0; i < i2c_msg.tx_len - 1; i++) {
			i2c_msg.data[i + 1] = strtol(argv[i + 3], NULL, 16);
			LOG_INF("i2c_msg.data[%d + 1] = 0x%x", i, i2c_msg.data[i + 1]);
		}
	}

	if (i2c_master_write(&i2c_msg, retry)) {
		shell_error(shell, "Failed to write reg, bus: %d, addr: 0x%x, reg: 0x%x", bus, addr,
			    offset);
		return;
	}

	shell_print(shell, "clock set <clock> <reg-offset> <value> success!");

	return;
}

void cmd_get_clock(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 4) {
		shell_warn(shell, "Help: get <clock> <reg-offset> <read_length>");
		return;
	}

	int clock_index = 0;
	char clock_string[20] = { 0 };
	snprintf(clock_string, sizeof(clock_string), "%s", argv[1]);

	if (strcmp(clock_string, "CLKGEN_312M") == 0) {
		clock_index = CLKGEN_312M;
	} else if (strcmp(clock_string, "CLKBUF_100M_U471") == 0) {
		clock_index = CLKBUF_100M_U471;
	} else if (strcmp(clock_string, "CLKBUF_100M_U519") == 0) {
		clock_index = CLKBUF_100M_U519;
	} else if (strcmp(clock_string, "CLKGEN_100M") == 0) {
		clock_index = CLKGEN_100M;
	} else {
		shell_error(shell, "Type wrong clock name");
		return;
	}

	int result = 0;
	uint8_t addr = 0;
	uint8_t bus = 0;
	uint8_t offset = 0;
	uint8_t read_len = 0;

	offset = strtol(argv[2], NULL, 16);
	read_len = strtol(argv[3], NULL, 10);

	result = find_clock_address_and_bus_by_clock_name_index(clock_index, &addr, &bus);
	if (result != 0) {
		shell_error(shell, "Can't find clock address and bus by clock name");
		return;
	}

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = read_len;
	i2c_msg.data[0] = offset;

	if (i2c_master_read(&i2c_msg, retry)) {
		shell_error(shell, "Failed to read reg, bus: %d, addr: 0x%x, reg: 0x%x", bus, addr,
			    offset);
		return;
	}

	for (int i = 0; i < read_len; i++) {
		shell_print(shell, "Byte%d = 0x%x", i + 1, i2c_msg.data[i]);
	}

	return;
}

/* Sub-command Level 1 of command clock */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_clock_cmds, SHELL_CMD(set, NULL, "set_clock", cmd_set_clock),
			       SHELL_CMD(get, NULL, "get_clock", cmd_get_clock),
			       SHELL_SUBCMD_SET_END);

/* Root of command clock */
SHELL_CMD_REGISTER(clock, &sub_clock_cmds, "Clock commands for AG", NULL);
