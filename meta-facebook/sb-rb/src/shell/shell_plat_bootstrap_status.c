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
#include <logging/log.h>

LOG_MODULE_REGISTER(shell_bootstrap_status, LOG_LEVEL_DBG);
#define MFIO16_BOOTSTRAP_STATUS_CPLD_OFFSET 0xb1

#define HAMSA_MFIO16_BOOTSTRAP_BIT 2
#define MEDHA0_MFIO16_BOOTSTRAP_BIT 1
#define MEDHA1_MFIO16_BOOTSTRAP_BIT 0

static int cmd_get_bootstrap_status(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "get MFIO16 status from CPLD");
	uint8_t reg_status = 0;

	if (!plat_read_cpld(MFIO16_BOOTSTRAP_STATUS_CPLD_OFFSET, &reg_status, 1)) {
			LOG_DBG("plat_read_cpld failed: offset=0x%02x", MFIO16_BOOTSTRAP_STATUS_CPLD_OFFSET);
			shell_error(shell, "read MFIO16 status from CPLD failed");
			return -1;
		}
	shell_print(shell, "HAMSA_MFIO16 : %d", (reg_status >> HAMSA_MFIO16_BOOTSTRAP_BIT)&1);
	shell_print(shell, "MEDHA0_MFIO16 : %d", (reg_status >> MEDHA0_MFIO16_BOOTSTRAP_BIT)&1);
	shell_print(shell, "MEDHA1_MFIO16 : %d", (reg_status >> MEDHA1_MFIO16_BOOTSTRAP_BIT)&1);

	return reg_status;	
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_set_spimux_oob_cmds,
			       SHELL_CMD_ARG(status, NULL, "bootstrap status command",
					 cmd_get_bootstrap_status, 1, 0),
			       SHELL_SUBCMD_SET_END);

/* Root of command spi test */
SHELL_CMD_REGISTER(bootstrap_MFIO16, &sub_set_spimux_oob_cmds, "bootstrap status commands", NULL);
