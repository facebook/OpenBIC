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

#include "info_shell.h"
#include "plat_version.h"
#include "util_sys.h"

#ifndef CONFIG_BOARD
#define CONFIG_BOARD "unknown"
#endif

#define RTOS_TYPE "Zephyr"

__weak void pal_show_extra_info(const struct shell *shell)
{
	return;
}

int cmd_info_print(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(
		shell,
		"========================{SHELL COMMAND INFO}========================================");
	shell_print(shell, "* NAME:          Platform command");
	shell_print(shell, "* DESCRIPTION:   Commands that could be used to debug or validate.");
	shell_print(shell, "* DATE/VERSION:  none");
	shell_print(shell, "* CHIP/OS:       %s - %s", CONFIG_BOARD, RTOS_TYPE);
	shell_print(shell, "* Note:          none");
	shell_print(shell, "------------------------------------------------------------------");
	shell_print(shell, "* PLATFORM:      %s-%s", PLATFORM_NAME, PROJECT_NAME);
	shell_print(shell, "* BOARD ID:      %d", BOARD_ID);
	shell_print(shell, "* STAGE:         %d", PROJECT_STAGE);
	shell_print(shell, "* SYSTEM:        %d", get_system_class());
	shell_print(shell, "* FW VERSION:    %d.%d", FIRMWARE_REVISION_1, FIRMWARE_REVISION_2);
	shell_print(shell, "* FW DATE:       %x%x.%x.%x", BIC_FW_YEAR_MSB, BIC_FW_YEAR_LSB,
		    BIC_FW_WEEK, BIC_FW_VER);
	shell_print(shell, "* FW IMAGE:      %s.bin", CONFIG_KERNEL_BIN_NAME);
	pal_show_extra_info(shell);
	shell_print(
		shell,
		"========================{SHELL COMMAND INFO}========================================");
	return 0;
}
