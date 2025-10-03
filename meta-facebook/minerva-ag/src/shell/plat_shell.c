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
#include "plat_sensor_polling_shell.h"
#include "log_shell.h"
#include "cpld_shell.h"
#include "plat_pldm_fw_version_shell.h"
#include "mctp.h"
#include "pldm.h"
#include "plat_aegis_power_control_shell.h"
#include <time.h>
#include "plat_datetime.h"

void pldm_cmd(const struct shell *shell, size_t argc, char **argv)
{
	if (argc < 4) {
		shell_warn(shell, "Help: pldm <eid> <pldm_type> <pldm_cmd> <pldm_data>");
		return;
	}

	const uint8_t eid = strtol(argv[1], NULL, 16);

	uint8_t resp_buf[PLDM_MAX_DATA_SIZE] = { 0 };
	pldm_msg pmsg = { 0 };
	pmsg.hdr.msg_type = MCTP_MSG_TYPE_PLDM;
	pmsg.hdr.pldm_type = strtol(argv[2], NULL, 16);
	pmsg.hdr.cmd = strtol(argv[3], NULL, 16);
	pmsg.hdr.rq = PLDM_REQUEST;
	pmsg.len = argc - 4;
	uint8_t req_buf[pmsg.len];
	pmsg.buf = req_buf;

	for (int i = 0; i < pmsg.len; i++)
		pmsg.buf[i] = strtol(argv[i + 4], NULL, 16);

	mctp *mctp_inst = NULL;
	if (get_mctp_info_by_eid(eid, &mctp_inst, &pmsg.ext_params) == false) {
		shell_print(shell, "Failed to get mctp info by eid 0x%x", eid);
		return;
	}

	uint16_t resp_len = mctp_pldm_read(mctp_inst, &pmsg, resp_buf, sizeof(resp_buf));
	if (resp_len == 0) {
		shell_print(shell, "Failed to get mctp-pldm response");
		return;
	}

	shell_print(shell, "RESP");
	shell_hexdump(shell, resp_buf, resp_len);

	return;
}

void cmd_plat_get_datetime(const struct shell *shell, size_t argc, char **argv)
{
	struct tm tm_now;
	rtc_get_tm(&tm_now);

	shell_print(shell, "datetime: %04d-%02d-%02d %02d:%02d:%02d, epoch: %ld",
		    tm_now.tm_year + 1900, tm_now.tm_mon + 1, tm_now.tm_mday, tm_now.tm_hour,
		    tm_now.tm_min, tm_now.tm_sec);
}

/* Sub-command Level 1 of command test */
SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_test_cmds,
	SHELL_CMD(sensor, &sub_plat_sensor_polling_cmd, "set/get platform sensor polling command",
		  NULL),
	SHELL_CMD(log, &sub_plat_log_cmd, "platform log command", NULL),
	SHELL_CMD(cpld, &sub_cpld_cmd, "cpld command", NULL),
	SHELL_CMD(get_fw_version, &sub_get_fw_version_cmd, "get fw version command", NULL),
	SHELL_CMD(pldm, NULL, "send pldm to bmc", pldm_cmd),
	SHELL_CMD(aegis_power, &sub_aegis_power_cmds, "aegis power commands", NULL),
	SHELL_CMD(get_datetime, NULL, "get datetime", cmd_plat_get_datetime), SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(test, &sub_test_cmds, "Test commands for AG", NULL);
