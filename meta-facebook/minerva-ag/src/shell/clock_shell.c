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
#include <sys/slist.h>
#include "libutil.h"
#include "plat_i2c.h"
#include "clock_shell.h"

LOG_MODULE_REGISTER(clock_shell);

#define CLK_GEN_312M_ADDR (0x12 >> 1)
#define CLK_GEN_100M_ADDR (0xD0 >> 1)
#define CLK_BUF_U471_ADDR (0xCE >> 1)
#define CLK_BUF_U519_ADDR (0xD8 >> 1)

#define CLK_GEN_312M_SW_RESET_OFFSET 0x0C
#define CLK_GEN_312M_LOSMON_0_STS_OFFSET 0x30
#define CLK_GEN_312M_LOSMON_1_STS_OFFSET 0x40
#define CLK_GEN_312M_LOSMON_2_STS_OFFSET 0x50
#define CLK_GEN_312M_ACTMON_0_STS_OFFSET 0x60
#define CLK_GEN_312M_ACTMON_1_STS_OFFSET 0x80
#define CLK_BUF_100M_WRITE_LOCK_CLEAR_LOS_EVENT_OFFSET 0x27
#define CLK_GEN_312M_PAGE_REGISTER 0xFC
#define CLK_BUF_100M_BYTE_COUNT 0x7

#define REGISTER_BYTE_MAX 4

typedef struct _clock_default_info {
	sys_snode_t node;
	bool is_default;
	uint8_t clock_index;
	uint8_t value[REGISTER_BYTE_MAX];
	uint8_t write_length;
	uint16_t offset;
} clock_default_info;

clock_compnt_mapping clock_compnt_mapping_table[] = {
	{ CLKGEN_312M, CLK_GEN_312M_ADDR, I2C_BUS1, "CLKGEN_312M" },
	{ AEGIS_CLKBUF_100M_U471, CLK_BUF_U471_ADDR, I2C_BUS1, "AEGIS_CLKBUF_100M_U471" },
	{ AEGIS_CLKBUF_100M_U519, CLK_BUF_U519_ADDR, I2C_BUS1, "AEGIS_CLKBUF_100M_U519" },
	{ CLKGEN_100M, CLK_GEN_100M_ADDR, I2C_BUS1, "CLKGEN_100M" },
};

static sys_slist_t clock_register_default_list =
	SYS_SLIST_STATIC_INIT(&clock_register_default_list);

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

int check_clock_offset_msb(uint8_t offset_msb, uint8_t addr, uint8_t bus)
{
	uint8_t data[4] = { 0 };

	if (offset_msb == 0) {
		memcpy(data, (uint8_t[]){ 0x00, 0x00, 0x00, 0x00 }, 4);
	} else if (offset_msb == 1) {
		memcpy(data, (uint8_t[]){ 0x00, 0x01, 0x00, 0x00 }, 4);
	} else {
		return -1;
	}

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.tx_len = 5;
	i2c_msg.data[0] = CLK_GEN_312M_PAGE_REGISTER;

	memcpy(&i2c_msg.data[1], data, sizeof(data));

	if (i2c_master_write(&i2c_msg, retry)) {
		LOG_ERR("Failed to write reg, bus: %d, addr: 0x%x, reg: 0x%x", bus, addr,
			CLK_GEN_312M_PAGE_REGISTER);
		return -1;
	}

	return 0;
}

bool clock_enum_get(uint8_t *name, uint8_t *num)
{
	CHECK_NULL_ARG_WITH_RETURN(name, false);
	CHECK_NULL_ARG_WITH_RETURN(num, false);

	for (int i = 0; i < CLK_COMPONENT_MAX; i++) {
		if (strcmp(name, clock_compnt_mapping_table[i].clock_name) == 0) {
			*num = i;
			return true;
		}
	}

	LOG_ERR("invalid clock name %s", name);
	return false;
}

void cmd_set_clock(const struct shell *shell, size_t argc, char **argv)
{
	if (argc < 4) {
		shell_warn(shell, "Help: set <clock> <reg-offset> <value>|default");
		return;
	}

	enum CLOCK_COMPONENT clock_index;

	if (clock_enum_get(argv[1], &clock_index) == false) {
		shell_error(shell, "Invalid clock name: %s", argv[1]);
		return;
	}

	int result = 0;
	uint8_t addr = 0, bus = 0, write_length = 0;
	uint8_t offset_lsb = 0, offset_msb = 0;
	uint16_t offset = 0;

	offset = strtol(argv[2], NULL, 16);

	offset_lsb = offset & 0xff;
	offset_msb = (offset >> 8) & 0xff;

	result = find_clock_address_and_bus_by_clock_name_index(clock_index, &addr, &bus);
	if (result != 0) {
		shell_error(shell, "Can't find clock address and bus by clock name");
		return;
	}

	if (clock_index == CLKGEN_312M) {
		result = check_clock_offset_msb(offset_msb, addr, bus);
		if (result != 0) {
			shell_error(shell, "Failed to write reg, bus: %d, addr: 0x%x, reg: 0x%x",
				    bus, addr, CLK_GEN_312M_PAGE_REGISTER);
			return;
		}
	}

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.data[0] = offset_lsb;

	if (strcmp(argv[3], "default") == 0) {
		sys_snode_t *node;
		sys_snode_t *s_node;

		SYS_SLIST_FOR_EACH_NODE_SAFE (&clock_register_default_list, node, s_node) {
			clock_default_info *p = (clock_default_info *)node;

			if (p->clock_index == clock_index && p->offset == offset) {
				if (p->is_default == true) {
					i2c_msg.tx_len = p->write_length + 1;
					memcpy(&i2c_msg.data[1], p->value, p->write_length);

					if (i2c_master_write(&i2c_msg, retry)) {
						shell_error(
							shell,
							"Failed to write reg, bus: %d, addr: 0x%x, reg: 0x%x",
							bus, addr, offset);
						return;
					}

					shell_print(shell, "clock set %s 0x%x default success!",
						    argv[1], offset);
					return;
				} else {
					shell_error(shell, "clock set %s 0x%x default failed!",
						    argv[1], offset);
					return;
				}
			}
		}
		shell_error(shell, "clock set %s 0x%x default failed!", argv[1], offset);
		return;
	} else {
		//Before setting, you must first get register value and set default table.

		write_length = argc - 3; // bytes for writing to register, excluding offset

		i2c_msg.tx_len = 1;
		i2c_msg.rx_len = write_length;

		if (i2c_master_read(&i2c_msg, retry)) {
			shell_error(shell, "Failed to read reg, bus: %d, addr: 0x%x, reg: 0x%x",
				    bus, addr, offset);
			return;
		}

		if (sys_slist_is_empty(&clock_register_default_list)) {
			clock_default_info *p = NULL;
			p = (clock_default_info *)malloc(sizeof(*p));
			if (!p) {
				shell_error(shell, "clock_default_info alloc failed!");
				return;
			}
			memset(p, 0, sizeof(*p));

			p->clock_index = clock_index;
			p->is_default = true;
			p->offset = offset;
			p->write_length = write_length;
			memcpy(p->value, i2c_msg.data, write_length);
			sys_slist_append(&clock_register_default_list, &p->node);
		} else {
			sys_snode_t *node;
			sys_snode_t *s_node;
			bool find_default = false;

			SYS_SLIST_FOR_EACH_NODE_SAFE (&clock_register_default_list, node, s_node) {
				clock_default_info *p = (clock_default_info *)node;

				if (p->clock_index == clock_index && p->offset == offset) {
					if (p->is_default == true) {
						find_default = true;
						break; // default register value have been stored
					}
				}
			}

			if (find_default == false) {
				clock_default_info *p = NULL;
				p = (clock_default_info *)malloc(sizeof(*p));
				if (!p) {
					shell_error(shell, "clock_default_info alloc failed!");
					return;
				}
				memset(p, 0, sizeof(*p));

				p->clock_index = clock_index;
				p->is_default = true;
				p->offset = offset;
				p->write_length = write_length;
				memcpy(p->value, i2c_msg.data, write_length);
				sys_slist_append(&clock_register_default_list, &p->node);
			}
		}

		//Set value to register
		i2c_msg.tx_len = argc - 2;
		i2c_msg.rx_len = 0;
		i2c_msg.data[0] = offset_lsb;

		for (int i = 0; i < i2c_msg.tx_len - 1; i++) {
			i2c_msg.data[i + 1] = strtol(argv[i + 3], NULL, 16);
		}

		if (i2c_master_write(&i2c_msg, retry)) {
			shell_error(shell, "Failed to write reg, bus: %d, addr: 0x%x, reg: 0x%x",
				    bus, addr, offset);
			return;
		}

		shell_print(shell, "clock set %s 0x%x <value> success!", argv[1], offset);
	}

	return;
}

void cmd_get_clock(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 4) {
		shell_warn(shell, "Help: get <clock> <reg-offset> <read_length>");
		return;
	}

	enum CLOCK_COMPONENT clock_index;

	if (clock_enum_get(argv[1], &clock_index) == false) {
		shell_error(shell, "Invalid clock name: %s", argv[1]);
		return;
	}

	int result = 0;
	uint8_t addr = 0;
	uint8_t bus = 0;
	uint8_t read_len = 0;
	uint8_t offset_lsb = 0, offset_msb = 0;
	uint16_t offset = 0;

	offset = strtol(argv[2], NULL, 16);
	read_len = strtol(argv[3], NULL, 10);

	offset_lsb = offset & 0xff;
	offset_msb = (offset >> 8) & 0xff;

	result = find_clock_address_and_bus_by_clock_name_index(clock_index, &addr, &bus);
	if (result != 0) {
		shell_error(shell, "Can't find clock address and bus by clock name");
		return;
	}

	if (clock_index == CLKGEN_312M) {
		result = check_clock_offset_msb(offset_msb, addr, bus);
		if (result != 0) {
			shell_error(shell, "Failed to write reg, bus: %d, addr: 0x%x, reg: 0x%x",
				    bus, addr, CLK_GEN_312M_PAGE_REGISTER);
			return;
		}
	}

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = read_len;
	i2c_msg.data[0] = offset_lsb;

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

void cmd_get_clock_status(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_warn(shell, "Help: get <clock>");
		return;
	}

	enum CLOCK_COMPONENT clock_index;

	if (clock_enum_get(argv[1], &clock_index) == false) {
		shell_error(shell, "Invalid clock name: %s", argv[1]);
		return;
	}

	int result = 0;
	uint8_t addr = 0;
	uint8_t bus = 0;

	result = find_clock_address_and_bus_by_clock_name_index(clock_index, &addr, &bus);
	if (result != 0) {
		shell_error(shell, "Can't find clock address and bus by clock name");
		return;
	}

	if (clock_index == CLKGEN_312M) {
		result = check_clock_offset_msb(0, addr, bus);
		if (result != 0) {
			shell_error(shell, "Failed to write reg, bus: %d, addr: 0x%x, reg: 0x%x",
				    bus, addr, CLK_GEN_312M_PAGE_REGISTER);
			return;
		}
	}

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;

	switch (clock_index) {
	case CLKGEN_312M:
		i2c_msg.bus = bus;
		i2c_msg.target_addr = addr;
		i2c_msg.tx_len = 1;
		i2c_msg.rx_len = 1;
		i2c_msg.data[0] = CLK_GEN_312M_LOSMON_0_STS_OFFSET;
		if (i2c_master_read(&i2c_msg, retry)) {
			shell_error(shell, "Failed to read reg, bus: %d, addr: 0x%x, reg: 0x%x",
				    bus, addr, CLK_GEN_312M_LOSMON_0_STS_OFFSET);
			return;
		}
		shell_print(shell, "LOSMON_0_STS Register = 0x%x", i2c_msg.data[0]);

		memset(i2c_msg.data, 0, I2C_BUFF_SIZE);

		i2c_msg.data[0] = CLK_GEN_312M_LOSMON_1_STS_OFFSET;
		if (i2c_master_read(&i2c_msg, retry)) {
			shell_error(shell, "Failed to read reg, bus: %d, addr: 0x%x, reg: 0x%x",
				    bus, addr, CLK_GEN_312M_LOSMON_1_STS_OFFSET);
			return;
		}
		shell_print(shell, "LOSMON_1_STS Register = 0x%x", i2c_msg.data[0]);

		memset(i2c_msg.data, 0, I2C_BUFF_SIZE);

		i2c_msg.data[0] = CLK_GEN_312M_LOSMON_2_STS_OFFSET;
		if (i2c_master_read(&i2c_msg, retry)) {
			shell_error(shell, "Failed to read reg, bus: %d, addr: 0x%x, reg: 0x%x",
				    bus, addr, CLK_GEN_312M_LOSMON_2_STS_OFFSET);
			return;
		}
		shell_print(shell, "LOSMON_2_STS Register = 0x%x", i2c_msg.data[0]);

		memset(i2c_msg.data, 0, I2C_BUFF_SIZE);

		i2c_msg.data[0] = CLK_GEN_312M_ACTMON_0_STS_OFFSET;
		if (i2c_master_read(&i2c_msg, retry)) {
			shell_error(shell, "Failed to read reg, bus: %d, addr: 0x%x, reg: 0x%x",
				    bus, addr, CLK_GEN_312M_ACTMON_0_STS_OFFSET);
			return;
		}
		shell_print(shell, "ACTMON_0_STS Register = 0x%x", i2c_msg.data[0]);

		memset(i2c_msg.data, 0, I2C_BUFF_SIZE);

		i2c_msg.data[0] = CLK_GEN_312M_ACTMON_1_STS_OFFSET;
		if (i2c_master_read(&i2c_msg, retry)) {
			shell_error(shell, "Failed to read reg, bus: %d, addr: 0x%x, reg: 0x%x",
				    bus, addr, CLK_GEN_312M_ACTMON_1_STS_OFFSET);
			return;
		}
		shell_print(shell, "ACTMON_1_STS Register = 0x%x", i2c_msg.data[0]);
		break;
	case AEGIS_CLKBUF_100M_U471:
	case AEGIS_CLKBUF_100M_U519:
		i2c_msg.bus = bus;
		i2c_msg.target_addr = addr;
		i2c_msg.tx_len = 1;
		i2c_msg.rx_len = 2; //first byte is data length because it is block read
		i2c_msg.data[0] = CLK_BUF_100M_WRITE_LOCK_CLEAR_LOS_EVENT_OFFSET;
		if (i2c_master_read(&i2c_msg, retry)) {
			shell_error(shell, "Failed to read reg, bus: %d, addr: 0x%x, reg: 0x%x",
				    bus, addr, CLK_BUF_100M_WRITE_LOCK_CLEAR_LOS_EVENT_OFFSET);
			return;
		}
		shell_print(shell, "WRITE_LOCK_CLEAR_LOS_EVENT Register = 0x%x", i2c_msg.data[1]);
		break;
	case CLKGEN_100M:
		shell_warn(shell, "CLKGEN_100M has no clock status need to get");
		break;
	default:
		shell_error(shell, "Type wrong clock name");
	}

	return;
}

void cmd_clear_clock_status(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_warn(shell, "Help: clear <clock>");
		return;
	}

	enum CLOCK_COMPONENT clock_index;

	if (clock_enum_get(argv[1], &clock_index) == false) {
		shell_error(shell, "Invalid clock name: %s", argv[1]);
		return;
	}

	int result = 0;
	uint8_t addr = 0;
	uint8_t bus = 0;

	result = find_clock_address_and_bus_by_clock_name_index(clock_index, &addr, &bus);
	if (result != 0) {
		shell_error(shell, "Can't find clock address and bus by clock name");
		return;
	}

	if (clock_index == CLKGEN_312M) {
		result = check_clock_offset_msb(0, addr, bus);
		if (result != 0) {
			shell_error(shell, "Failed to write reg, bus: %d, addr: 0x%x, reg: 0x%x",
				    bus, addr, CLK_GEN_312M_PAGE_REGISTER);
			return;
		}
	}

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;
	int byte_count;

	switch (clock_index) {
	case CLKGEN_312M:
		i2c_msg.bus = bus;
		i2c_msg.target_addr = addr;
		i2c_msg.tx_len = 3;
		i2c_msg.data[0] = CLK_GEN_312M_SW_RESET_OFFSET;
		i2c_msg.data[1] = 0x3E;
		i2c_msg.data[2] = 0x00;
		if (i2c_master_write(&i2c_msg, retry)) {
			shell_error(shell, "Failed to write reg, bus: %d, addr: 0x%x, reg: 0x%x",
				    bus, addr, CLK_GEN_312M_SW_RESET_OFFSET);
			return;
		}

		k_sleep(K_MSEC(10));

		i2c_msg.data[0] = CLK_GEN_312M_SW_RESET_OFFSET;
		i2c_msg.data[1] = 0x00;
		i2c_msg.data[2] = 0x00;

		if (i2c_master_write(&i2c_msg, retry)) {
			shell_error(shell, "Failed to write reg, bus: %d, addr: 0x%x, reg: 0x%x",
				    bus, addr, CLK_GEN_312M_SW_RESET_OFFSET);
			return;
		}

		shell_print(shell, "clock clear %s success!", argv[1]);
		break;
	case AEGIS_CLKBUF_100M_U471:
	case AEGIS_CLKBUF_100M_U519:
		i2c_msg.bus = bus;
		i2c_msg.target_addr = addr;
		i2c_msg.tx_len = 1;
		i2c_msg.rx_len = 2;
		i2c_msg.data[0] = CLK_BUF_100M_BYTE_COUNT;

		if (i2c_master_read(&i2c_msg, retry)) {
			shell_error(shell, "Failed to read reg, bus: %d, addr: 0x%x, reg: 0x%x",
				    bus, addr, CLK_BUF_100M_BYTE_COUNT);
			return;
		}

		byte_count = i2c_msg.data[1];

		i2c_msg.tx_len = 3;
		i2c_msg.rx_len = 0;
		i2c_msg.data[0] = CLK_BUF_100M_WRITE_LOCK_CLEAR_LOS_EVENT_OFFSET;
		i2c_msg.data[1] = byte_count;
		i2c_msg.data[2] = 0x2;
		if (i2c_master_write(&i2c_msg, retry)) {
			shell_error(shell, "Failed to write reg, bus: %d, addr: 0x%x, reg: 0x%x",
				    bus, addr, CLK_BUF_100M_WRITE_LOCK_CLEAR_LOS_EVENT_OFFSET);
			return;
		}
		shell_print(shell, "clock clear %s success!", argv[1]);
		break;
	case CLKGEN_100M:
		shell_warn(shell, "CLKGEN_100M has no clock status need to clear");
		break;
	default:
		shell_error(shell, "Type wrong clock name");
	}

	return;
}

bool clock_name_get(uint8_t index, uint8_t **name)
{
	CHECK_NULL_ARG_WITH_RETURN(name, false);

	if (index >= CLK_COMPONENT_MAX) {
		*name = NULL;
		return false;
	}

	*name = (uint8_t *)clock_compnt_mapping_table[index].clock_name;
	return true;
}

static void all_clock_name_get(size_t idx, struct shell_static_entry *entry)
{
	uint8_t *name = NULL;
	clock_name_get((uint8_t)idx, &name);

	entry->syntax = (name) ? (const char *)name : NULL;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;
}

SHELL_DYNAMIC_CMD_CREATE(clock_name, all_clock_name_get);

/* Sub-command Level 1 of command clock */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_clock_cmds,
			       SHELL_CMD_ARG(set, &clock_name,
					     "set <clock> <reg-offset> <value>|default",
					     cmd_set_clock, 4, 3),
			       SHELL_CMD_ARG(get, &clock_name,
					     "get <clock> <reg-offset> <read_length>",
					     cmd_get_clock, 4, 0),
			       SHELL_SUBCMD_SET_END);

/* Root of command clock */
SHELL_CMD_REGISTER(clock, &sub_clock_cmds, "Clock commands for AG", NULL);

/* Sub-command Level 1 of command clock_status */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_clock_status_cmds,
			       SHELL_CMD_ARG(get, &clock_name, "get <clock>", cmd_get_clock_status,
					     2, 0),
			       SHELL_CMD_ARG(clear, &clock_name, "clear <clock>",
					     cmd_clear_clock_status, 2, 0),
			       SHELL_SUBCMD_SET_END);

/* Root of command clock status*/
SHELL_CMD_REGISTER(clock_status, &sub_clock_status_cmds, "Clock status commands for AG", NULL);
