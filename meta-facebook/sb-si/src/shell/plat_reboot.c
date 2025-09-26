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
#include <zephyr.h>
#include <stdio.h>
#include <zephyr.h>
#include "util_sys.h"
#include "plat_pldm_sensor.h"

#define PLAT_WAIT_SENSOR_POLLING_END_DELAY_MS 1000

void cmd_plat_reboot(struct shell *shell, size_t argc, char **argv)
{
	set_plat_sensor_polling_enable_flag(false);
	k_msleep(PLAT_WAIT_SENSOR_POLLING_END_DELAY_MS);
	submit_bic_warm_reset();
}

SHELL_CMD_REGISTER(reboot, NULL, "reboot command", cmd_plat_reboot);