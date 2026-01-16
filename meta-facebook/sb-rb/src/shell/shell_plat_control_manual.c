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
#include "plat_cpld.h"
#include "plat_gpio.h"

#define MANUAL_CONTROL_STRAP_CPLD_OFFSET 0xb2

#define MANUAL_CONTROL_STRAP_BIT 0
#define MANUAL_CONTROL_STRAP_BIT_MASK 0x01
#define ENABLE_MANUAL_CONTROL_STRAP 0
#define DISABLE_MANUAL_CONTROL_STRAP 1

static int cmd_enable_manual_control_strap(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t reg_status = 0;

	if (!set_cpld_bit(MANUAL_CONTROL_STRAP_CPLD_OFFSET, MANUAL_CONTROL_STRAP_BIT, ENABLE_MANUAL_CONTROL_STRAP)){
		shell_error(shell, "write manual control strap to CPLD failed");
		return -1;
	}

	if (!plat_read_cpld(MANUAL_CONTROL_STRAP_CPLD_OFFSET, &reg_status, 1)) {
		shell_error(shell, "read manual control strap from CPLD failed");
		return -1;
	}
	
	if ((reg_status &MANUAL_CONTROL_STRAP_BIT_MASK) != ENABLE_MANUAL_CONTROL_STRAP){
		shell_error(shell, "enable manual control strap failed");
		return -1;
	}
	
	shell_print(shell, "STRAP_HIZ_EN : %d (low active)", reg_status & MANUAL_CONTROL_STRAP_BIT_MASK);
	return 0;	
}

static int cmd_disable_manual_control_strap(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t reg_status = 0;

	if (!set_cpld_bit(MANUAL_CONTROL_STRAP_CPLD_OFFSET, MANUAL_CONTROL_STRAP_BIT, DISABLE_MANUAL_CONTROL_STRAP)){
		shell_error(shell, "write manual control strap to CPLD failed");
		return -1;
	}

	if (!plat_read_cpld(MANUAL_CONTROL_STRAP_CPLD_OFFSET, &reg_status, 1)) {
		shell_error(shell, "read manual control strap from CPLD failed");
		return -1;
	}
	
	if ((reg_status & MANUAL_CONTROL_STRAP_BIT_MASK) != DISABLE_MANUAL_CONTROL_STRAP){
		shell_error(shell, "disable manual control strap failed");
		return -1;
	}
	
	shell_print(shell, "STRAP_HIZ_EN : %d (low active)", reg_status & MANUAL_CONTROL_STRAP_BIT_MASK);
	return 0;	
}
static int cmd_get_manual_control_strap_status(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t reg_status = 0;
		if (!plat_read_cpld(MANUAL_CONTROL_STRAP_CPLD_OFFSET, &reg_status, 1)) {
		shell_error(shell, "read manual control strap from CPLD failed");
		return -1;
	}
	shell_print(shell, "STRAP_HIZ_EN : %d (low active)", reg_status & MANUAL_CONTROL_STRAP_BIT_MASK);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_set_strap_control_cmds,
			       SHELL_CMD_ARG(enable, NULL, "enable manual control strap",
					 cmd_enable_manual_control_strap, 1, 0),
				   SHELL_CMD_ARG(disable, NULL, "disable manual control strap",
					 cmd_disable_manual_control_strap, 1, 0),
					SHELL_CMD_ARG(get, NULL, "get manual control strap status",
					 cmd_get_manual_control_strap_status, 1, 0),
			       SHELL_SUBCMD_SET_END);

/* Root of command spi test */
SHELL_CMD_REGISTER(strap_control_manual, &sub_set_strap_control_cmds, "strap_control_manual <enable | disable | get>", NULL);
