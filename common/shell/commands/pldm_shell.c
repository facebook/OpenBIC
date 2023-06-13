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

#include "pldm.h"
#include "pldm_shell.h"
#include <stdlib.h>
#include <string.h>

/*
 * Command Functions
 */

void cmd_pldm_send_req(const struct shell *shell, size_t argc, char **argv)
{
	if (argc < 4) {
		shell_warn(
			shell,
			"Help: platform pldm sendreq <mctp_dest_eid> <pldm_type> <cmd> <data...>");
		return;
	}

	uint8_t mctp_dest_eid = strtol(argv[1], NULL, 16);
	uint8_t pldm_type = strtol(argv[2], NULL, 16);
	uint8_t pldm_cmd = strtol(argv[3], NULL, 16);
	uint16_t pldm_data_len = argc - 4;

	uint8_t resp_buf[PLDM_MAX_DATA_SIZE] = { 0 };
	pldm_msg pmsg = { 0 };
	pmsg.hdr.msg_type = MCTP_MSG_TYPE_PLDM;
	pmsg.hdr.pldm_type = pldm_type;
	pmsg.hdr.cmd = pldm_cmd;
	pmsg.hdr.rq = PLDM_REQUEST;
	pmsg.len = pldm_data_len;
	for (int i = 0; i < pmsg.len; i++)
		pmsg.buf[i] = strtol(argv[4 + i], NULL, 16);

	mctp *mctp_inst = NULL;
	if (get_mctp_info_by_eid(mctp_dest_eid, &mctp_inst, &pmsg.ext_params) == false) {
		shell_error(shell, "Failed to get mctp info by eid 0x%x", mctp_dest_eid);
		return;
	}

	uint16_t resp_len = mctp_pldm_read(mctp_inst, &pmsg, resp_buf, sizeof(resp_buf));
	if (resp_len == 0) {
		shell_error(shell, "Failed to get mctp response");
		return;
	}

	shell_print(shell, "* mctp: 0x%x addr: 0x%x eid: 0x%x msg_type: 0x%x", mctp_inst, pmsg.ext_params.smbus_ext_params.addr, mctp_dest_eid, MCTP_MSG_TYPE_PLDM);
	shell_print(shell, "  pldm_type: 0x%x pldm cmd: 0x%x", pldm_type, pldm_cmd);
	shell_hexdump(shell, pmsg.buf, pmsg.len);

	if (resp_buf[0] != PLDM_SUCCESS)
		shell_error(shell, "Response with bad cc 0x%x", resp_buf[0]);
	else {
		shell_hexdump(shell, resp_buf, resp_len);
		shell_print(shell, "");
	}
}
