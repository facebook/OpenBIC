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
 *
 * Minimal "plat" shell command group, standing in for the equivalent
 * commands under common/shell/commands (not buildable here yet - see
 * top-level README). Just enough to query live platform GPIO state.
 */

#include <zephyr/shell/shell.h>

#include "plat_gpio.h"

static int cmd_gpio_mon0(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "mon0 (SW2): %s", plat_gpio_mon0_get() ? "asserted" : "deasserted");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_plat_gpio, SHELL_CMD(mon0, NULL, "Read mon0 (SW2) live state",
							 cmd_gpio_mon0),
				SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_plat, SHELL_CMD(gpio, &sub_plat_gpio, "GPIO status commands",
						    NULL),
				SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(plat, &sub_plat, "OpenBIC platform commands", NULL);
