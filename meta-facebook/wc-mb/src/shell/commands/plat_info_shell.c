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

#include "plat_info_shell.h"
#include "plat_class.h"

int cmd_wcmb_info_print(const struct shell *shell, size_t argc, char **argv)
{
	;
	shell_print(shell, "========================{PLATFORM INFO}========================");
	shell_print(shell, "* NAME:          Waimea Canyon platform command");
	shell_print(shell, "* DESCRIPTION:   Commands that could be used to debug or validate.");
	shell_print(shell, "* Note:          none");
	shell_print(shell, "------------------------------------------------------------------");
	shell_print(shell, "* SYSTEM CLASS:  %s",
		    get_system_class() == SYS_DUAL ? "Dual" : "Single");
	shell_print(shell, "* SYSTEM SOURCE: %s",
		    get_source_class() == SRC_MAIN ? "Main" : "Sencond");
	shell_print(shell, "========================{PLATFORM INFO}========================");
	return 0;
}
