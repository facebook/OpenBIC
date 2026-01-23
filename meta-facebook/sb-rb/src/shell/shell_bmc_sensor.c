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

LOG_MODULE_REGISTER(plat_bmc_sensor_shell, LOG_LEVEL_DBG);

static int cmd_get_bmc_sensor(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 1) {
		shell_error(shell, "Usage: bmc_sensor get");
		return -1;
	}

	uint8_t pwr_value_lsb = 0;
	uint8_t pwr_value_msb = 0;
	if (!plat_read_cpld(CPLD_POWER_INFO_0_REG, &pwr_value_lsb, 1)){
		shell_error(shell, "LSB read from CPLD fail");
		return -1;
	}
	if (!plat_read_cpld(CPLD_POWER_INFO_1_REG, &pwr_value_msb, 1)){
		shell_error(shell, "MSB read from CPLD fail");
		return -1;
	}

	shell_info(shell, "P52V_ASIC_SENSE_PWR:%d", (pwr_value_msb<<8)|pwr_value_lsb);

	return 0;
}
SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_bmc_sensor, SHELL_CMD(get, NULL, "get P52V_ASIC_SENSE_PWR from CPLD", cmd_get_bmc_sensor),
	SHELL_SUBCMD_SET_END);

/* Root of command bmc_sensor */
SHELL_CMD_REGISTER(bmc_sensor, &sub_bmc_sensor, "bmc_sensor get",
		       NULL);
