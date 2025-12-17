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
#include "plat_isr.h"
#include "plat_i2c.h"
#include "plat_hook.h"
#include "plat_cpld.h"
#include "plat_ioexp.h"
#include "plat_class.h"

LOG_MODULE_REGISTER(plat_asic_mfio_debug_shell, LOG_LEVEL_DBG);

#define NC_HAMSA_MFIO20_BIT 2
#define NC_HAMSA_MFIO21_BIT 3
#define NC_HAMSA_MFIO22_BIT 4
#define NC_HAMSA_MFIO23_BIT 5
#define NC_HAMSA_MFIO24_BIT 6
#define NC_HAMSA_MFIO25_BIT 7
#define NC_HAMSA_MFIO26_BIT 0
#define NC_HAMSA_MFIO27_BIT 1
#define NC_HAMSA_MFIO28_BIT 2
#define NC_HAMSA_MFIO29_BIT 3
#define NC_HAMSA_MFIO30_BIT 4
#define NC_HAMSA_MFIO31_BIT 5

#define PORT_0 0
#define PORT_1 1
#define PORT_2 2

typedef struct {
	const char *name;
	uint8_t bit;
	uint8_t port;
} unassigned_mifo_pin_map_t;

typedef struct {
	uint8_t config_reg;
	uint8_t output_reg;
	uint8_t input_reg;
} port_map_t;

static const port_map_t port_map[] = {
	{TCA6424A_CONFIG_0, TCA6424A_OUTPUT_PORT_0, TCA6424A_INPUT_PORT_0},
	{TCA6424A_CONFIG_1, TCA6424A_OUTPUT_PORT_1, TCA6424A_INPUT_PORT_1},
	{TCA6424A_CONFIG_2, TCA6424A_OUTPUT_PORT_2, TCA6424A_INPUT_PORT_2},
};
static const unassigned_mifo_pin_map_t mfio_list[] = {
	{ "NC_HAMSA_MFIO20", NC_HAMSA_MFIO20_BIT, PORT_0 },
	{ "NC_HAMSA_MFIO21", NC_HAMSA_MFIO21_BIT, PORT_0 },
	{ "NC_HAMSA_MFIO22", NC_HAMSA_MFIO22_BIT, PORT_0 },
	{ "NC_HAMSA_MFIO23", NC_HAMSA_MFIO23_BIT, PORT_0 },
	{ "NC_HAMSA_MFIO24", NC_HAMSA_MFIO24_BIT, PORT_0 },
	{ "NC_HAMSA_MFIO25", NC_HAMSA_MFIO25_BIT, PORT_0 },
	{ "NC_HAMSA_MFIO26", NC_HAMSA_MFIO26_BIT, PORT_1 },
	{ "NC_HAMSA_MFIO27", NC_HAMSA_MFIO27_BIT, PORT_1 },
	{ "NC_HAMSA_MFIO28", NC_HAMSA_MFIO28_BIT, PORT_1 },
	{ "NC_HAMSA_MFIO29", NC_HAMSA_MFIO29_BIT, PORT_1 },
	{ "NC_HAMSA_MFIO30", NC_HAMSA_MFIO30_BIT, PORT_1 },
	{ "NC_HAMSA_MFIO31", NC_HAMSA_MFIO31_BIT, PORT_1 },
};
static int cmd_mfio_get_all(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "%-20s|%-15s|%-25s", "MFIO name", "Input/Output", "value");
	uint8_t config = 0;
	uint8_t config_value = 0;
	uint8_t input_value = 0;
	uint8_t output_value = 0;
	
	for(int i = 0; i < ARRAY_SIZE(mfio_list); i++){
		if (config != port_map[mfio_list[i].port].config_reg){
			config = port_map[mfio_list[i].port].config_reg;
			tca6424a_i2c_read(port_map[mfio_list[i].port].config_reg, &config_value, 1);
			tca6424a_i2c_read(port_map[mfio_list[i].port].input_reg, &input_value, 1);
			tca6424a_i2c_read(port_map[mfio_list[i].port].output_reg, &output_value, 1);
		}
		if((config_value>>mfio_list[i].bit)&1)
			shell_print(shell, "%-20s|%-15s|%-10d", mfio_list[i].name, "Input", (input_value>>mfio_list[i].bit)&1);
		else
			shell_print(shell, "%-20s|%-15s|%-10d", mfio_list[i].name, "Output", (output_value>>mfio_list[i].bit)&1);

	}

	return 0;
}

static int cmd_set_mfio_io(const struct shell *shell, size_t argc, char **argv)
{
	// shell_print(shell, "set ");
	uint8_t set_value = strtol(argv[2], NULL, 10);
	for(int i = 0; i < ARRAY_SIZE(mfio_list); i++){
		if(!strcmp(mfio_list[i].name, argv[1])){
			if(!tca6424a_i2c_write_bit(port_map[mfio_list[i].port].config_reg,mfio_list[i].bit, set_value))
				return -1;
			shell_print(shell, "set %s as : %s", mfio_list[i].name, (set_value == 0)? "Output": "Input");
			break;
		}
	}
	return 0;
}
static int cmd_set_mfio_value(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t set_value = strtol(argv[2], NULL, 10);
	uint8_t config_value = 0;
	for(int i = 0; i < ARRAY_SIZE(mfio_list); i++){
		if(!strcmp(mfio_list[i].name, argv[1])){
			tca6424a_i2c_read(port_map[mfio_list[i].port].config_reg, &config_value, 1);
			if ((config_value>>mfio_list[i].bit)&1){
				shell_error(shell, "Can't set value to Input port");
				return -1;
			}
			if(!tca6424a_i2c_write_bit(port_map[mfio_list[i].port].output_reg,mfio_list[i].bit, set_value))
				return -1;
			shell_print(shell, "set %s value to: %s", mfio_list[i].name, set_value);
			break;
		}
	}
	return 0;
}
static void mfio_dynamic_get_name(size_t idx, struct shell_static_entry *entry)
{
	entry->syntax = NULL;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;

	if (idx < ARRAY_SIZE(mfio_list)) {
		entry->syntax = mfio_list[idx].name;
	}
}
SHELL_DYNAMIC_CMD_CREATE(mifo_name, mfio_dynamic_get_name);

/* level 2 */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_mfio_get_cmds,
			       SHELL_CMD(all, NULL, "bootstrap get all", cmd_mfio_get_all),
			       SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_set_cmds,
			       SHELL_CMD(io, &mifo_name, "set io <mfio_name> 1/0", cmd_set_mfio_io),
				   SHELL_CMD(value, &mifo_name, "set value <mfio_name> 1/0", cmd_set_mfio_value),
			       SHELL_SUBCMD_SET_END);

/* level 1 */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_asic_mfio_debug_cmds,
			       SHELL_CMD(get, &sub_mfio_get_cmds, "get all", NULL),
			       SHELL_CMD(set, &sub_set_cmds,
					     "set io/value <mfio_name> 1/0",
					     NULL),
			       SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(asic_mfio_debug, &sub_asic_mfio_debug_cmds, "asic_mfio_debug set/get commands", NULL);
