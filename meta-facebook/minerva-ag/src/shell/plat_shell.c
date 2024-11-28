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

#include "plat_sensor_polling_shell.h"
#include "log_shell.h"
#include "cpld_shell.h"
#include "plat_pldm_fw_version_shell.h"

/* Sub-command Level 1 of command test */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_test_cmds,
			       SHELL_CMD(sensor, &sub_plat_sensor_polling_cmd,
					 "set/get platform sensor polling command", NULL),
			       SHELL_CMD(log, &sub_plat_log_cmd, "platform log command", NULL),
			       SHELL_CMD(cpld, &sub_cpld_cmd, "cpld command", NULL),
			       SHELL_CMD(get_fw_version, &sub_get_fw_version_cmd,
					 "get fw version command", NULL),
			       SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(test, &sub_test_cmds, "Test commands for AG", NULL);
