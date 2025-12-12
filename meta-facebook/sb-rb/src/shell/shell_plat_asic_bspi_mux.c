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
#include "plat_gpio.h"

#define AEGIS_CPLD_ADDR (0x4C >> 1)

LOG_MODULE_REGISTER(plat_asic_spi_mux_shell, LOG_LEVEL_DBG);

enum asic_bspi_name_index
{
	NC_SPI_MEDHA0_CRM_MUX,
	NC_SPI_MEDHA1_CRM_MUX,
	NC_SPI_HAMSA_CRM_MUX,
	MAX_BSPI_IDX,
};

typedef struct _asic_bspi_info_ {
	uint8_t mux_idx;
	uint8_t gpio_idx;
	uint8_t *mux_name;
} asic_bspi_info;

asic_bspi_info asic_bspi_mux_table[] =
{
	{ NC_SPI_MEDHA0_CRM_MUX, NC_SPI_MEDHA0_CRM_MUX_IN1, "NC_SPI_MEDHA0_CRM_MUX_IN1" },
	{ NC_SPI_MEDHA1_CRM_MUX, NC_SPI_MEDHA1_CRM_MUX_IN1, "NC_SPI_MEDHA1_CRM_MUX_IN1" },
	{ NC_SPI_HAMSA_CRM_MUX, NC_SPI_HAMSA_CRM_MUX_IN1, "NC_SPI_HAMSA_CRM_MUX_IN1"},
};

bool bspi_name_get(uint8_t idx, uint8_t **name)
{
	CHECK_NULL_ARG_WITH_RETURN(name, false);

	if (idx >= MAX_BSPI_IDX) {
		*name = NULL;
		return false;
	}

	*name = (uint8_t *)asic_bspi_mux_table[idx].mux_name;
	return true;
}

static int cmd_bspi_get_all(const struct shell *shell, size_t argc, char **argv)
{
	if (get_asic_board_id() != ASIC_BOARD_ID_EVB)
	{
		shell_warn(shell, "EVB support only!");
		return -1;
	}

	shell_print(shell, "%-4s|%-25s", "BSPI name", "1/0 (on/off)");
	for (int i = 0; i < MAX_BSPI_IDX; i++) {
		uint8_t *bspi_mux_name = (uint8_t *)asic_bspi_mux_table[i].mux_name;
		int on_off_value = gpio_get(asic_bspi_mux_table[i].gpio_idx);
		shell_print(shell, "%-4d|%-40s|0x%-40d", i, bspi_mux_name, on_off_value);
	}

	return 0;
}


static int cmd_bpsi_set(const struct shell *shell, size_t argc, char **argv)
{

	if (get_asic_board_id() != ASIC_BOARD_ID_EVB)
	{
		shell_warn(shell, "EVB support only!");
		return -1;
	}

	if (argc != 3) {
		shell_print(shell, "Usage: bspi mux set %s <on|off>", argv[0]);
		return -1;
	}

	// if dc is not on , it won't work
	if (!gpio_get(RST_IRIS_PWR_ON_PLD_R1_N))
	{
		shell_warn(shell, "Need to do DC on first or it won't works!");
		return -1;
	}

	const char *name = argv[1];
	long set_val = strtol(argv[2], NULL, 10);
	if (!strcmp(name, "NC_SPI_MEDHA0_CRM_MUX_IN1"))
	{
		if (gpio_set(NC_SPI_MEDHA0_CRM_MUX_IN1, set_val) == -1)
		{
			shell_print(shell, "set NC_SPI_MEDHA0_CRM_MUX_IN1 fail!");	
			return 0;
		}
			
	}
	else if (!strcmp(name, "NC_SPI_MEDHA1_CRM_MUX_IN1"))
	{
		if (gpio_set(NC_SPI_MEDHA1_CRM_MUX_IN1, set_val) == -1)
		{
			shell_print(shell, "set NC_SPI_MEDHA1_CRM_MUX_IN1 fail!");
			return 0;
		}
			
	}
	else if (!strcmp(name, "NC_SPI_HAMSA_CRM_MUX_IN1"))
	{
		if (gpio_set(NC_SPI_HAMSA_CRM_MUX_IN1, set_val) == -1)
		{
			shell_print(shell, "set NC_SPI_HAMSA_CRM_MUX_IN1 fail!");
			return 0;
		}
			
	}

	shell_print(shell, "Set %s %s success\n", argv[1], argv[2]);
	return 0;
}

static void asic_bspi_mux_name_get_(size_t idx, struct shell_static_entry *entry)
{
	uint8_t *name = NULL;
	bspi_name_get((uint8_t)idx, &name);

	entry->syntax = (name) ? (const char *)name : NULL;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;
}

SHELL_DYNAMIC_CMD_CREATE(asic_bspi_mux_name, asic_bspi_mux_name_get_);

/* level 2 */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_bspi_get_cmds,
			       SHELL_CMD(all, NULL, "asic BSPI mux get all", cmd_bspi_get_all),
			       SHELL_SUBCMD_SET_END);

/* level 1 */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_bspi_cmds,
			       SHELL_CMD(get, &sub_bspi_get_cmds, "get all", NULL),
			       SHELL_CMD_ARG(set, &asic_bspi_mux_name,
					     "set <bspi mux-name>|all",
					     cmd_bpsi_set, 3, 1),
			       SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(asic_bspi, &sub_bspi_cmds, "asic BSPI set/get commands", NULL);
