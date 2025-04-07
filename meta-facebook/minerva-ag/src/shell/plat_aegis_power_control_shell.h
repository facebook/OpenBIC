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

#ifndef PLAT_PLDM_AEGIS_POWER_CONTROL_SHELL_H
#define PLAT_PLDM_AEGIS_POWER_CONTROL_SHELL_H

#include <shell/shell.h>

int cmd_aegis_power_control(const struct shell *shell, size_t argc, char **argv);

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_aegis_power_cmds,
	SHELL_CMD(on, NULL, "aegis power control command", cmd_aegis_power_control),
	SHELL_CMD(off, NULL, "aegis power control command", cmd_aegis_power_control),
	SHELL_CMD(cycle, NULL, "aegis power control command", cmd_aegis_power_control),
	SHELL_SUBCMD_SET_END);

#endif
