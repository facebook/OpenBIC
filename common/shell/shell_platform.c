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

#include "commands/gpio_shell.h"
#include "commands/info_shell.h"
#include "commands/sensor_shell.h"
#include "commands/flash_shell.h"
#include "commands/ipmi_shell.h"
#include "commands/power_shell.h"
#include "commands/pldm_shell.h"

/* MAIN command */
SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_platform_cmds, SHELL_CMD(info, NULL, "Platform info.", cmd_info_print),
	SHELL_CMD(gpio, &sub_gpio_cmds, "GPIO relative command.", NULL),
	SHELL_CMD(sensor, &sub_sensor_cmds, "SENSOR relative command.", NULL),
	SHELL_CMD(flash, &sub_flash_cmds, "FLASH(spi) relative command.", NULL),
	SHELL_CMD(ipmi, &sub_ipmi_cmds, "IPMI relative command.", NULL),
	SHELL_CMD(power, &sub_power_cmds, "POWER relative command.", NULL),
	SHELL_CMD(pldm, &sub_pldm_cmds, "PLDM over MCTP relative command.", NULL),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(platform, &sub_platform_cmds, "Platform commands", NULL);
