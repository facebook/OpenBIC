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

void cmd_enable_spi(const struct shell *shell, size_t argc, char **argv);
void cmd_disable_spi(const struct shell *shell, size_t argc, char **argv);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_spi_cmd,
			       SHELL_CMD(enable, NULL, "switch mux to flash", cmd_enable_spi),
			       SHELL_CMD(disable, NULL, "switch mux to pcie", cmd_disable_spi),
			       SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(spi_control, &sub_spi_cmd, "flash control", NULL);