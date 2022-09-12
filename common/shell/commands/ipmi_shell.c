#include "ipmi_shell.h"
#include <stdio.h>
#include <zephyr.h>
#include <string.h>
#include "ipmi.h"
#include "ipmb.h"

void cmd_ipmi_raw(const struct shell *shell, size_t argc, char **argv)
{
	if (argc < 3) {
		shell_warn(shell, "Help: platform ipmi raw <netfn> <cmd> <datas>");
		return;
	}

	int netfn = strtol(argv[1], NULL, 16);
	int cmd = strtol(argv[2], NULL, 16);
	int data_len = argc - 3;

	ipmi_msg_cfg msg;
	msg.buffer.InF_source = SELF;
	msg.buffer.InF_target = SELF;
	msg.buffer.netfn = netfn;
	msg.buffer.cmd = cmd;
	msg.buffer.data_len = data_len;
	for (int i = 0; i < data_len; i++) {
		msg.buffer.data[i] = strtol(argv[3 + i], NULL, 16);
	}

	int retry = 0;
	while (k_msgq_put(&ipmi_msgq, &msg, K_NO_WAIT) != 0 && retry < 3) {
		k_msgq_purge(&ipmi_msgq);
		shell_error(shell, "Retry to put msg into ipmi msgq...");
		retry++;
	}

	if (k_msgq_get(&self_ipmi_msgq, &msg, K_MSEC(1000))) {
		shell_error(shell, "Failed to get ipmi msgq in time");
		return;
	}

	shell_print(shell, "BIC self command netfn:0x%x cmd:0x%x response with cc 0x%x",
		    msg.buffer.netfn, msg.buffer.cmd, msg.buffer.completion_code);
	if (msg.buffer.completion_code == CC_SUCCESS) {
		shell_hexdump(shell, msg.buffer.data, msg.buffer.data_len);
		shell_print(shell, "");
	}
}

void cmd_ipmi_list(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 1) {
		shell_warn(shell, "Help: platform ipmi scan");
		return;
	}

	uint8_t dummy_msg[10] = { 0 };
	dummy_msg[0] = (IANA_ID & 0xFF);
	dummy_msg[1] = (IANA_ID >> 8) & 0xFF;
	dummy_msg[2] = (IANA_ID >> 16) & 0xFF;

	uint8_t tmp_cmd_saver[255];
	uint8_t tmp_cmd_num = 0;

	ipmi_msg_cfg msg;
	msg.buffer.InF_source = SELF;
	msg.buffer.InF_target = SELF;
	msg.buffer.data_len = ARRAY_SIZE(dummy_msg);
	memcpy(msg.buffer.data, dummy_msg, ARRAY_SIZE(dummy_msg));

	shell_print(
		shell,
		"------------------------------------------------------------------------------");
	for (int netfn_idx = 0; netfn_idx < 0xFF; netfn_idx += 2) {
		tmp_cmd_num = 0;
		for (int cmd_idx = 0; cmd_idx < 0xFF; cmd_idx++) {
			msg.buffer.netfn = netfn_idx;
			msg.buffer.cmd = cmd_idx;
			msg.buffer.completion_code = CC_INVALID_CMD;
			msg.buffer.data_len = ARRAY_SIZE(dummy_msg);
			memcpy(msg.buffer.data, dummy_msg, ARRAY_SIZE(dummy_msg));

			if (k_msgq_put(&ipmi_msgq, &msg, K_NO_WAIT)) {
				shell_error(shell, "Failed to send req netfn:0x%x cmd:0x%x...",
					    netfn_idx, cmd_idx);
				continue;
			}
			if (k_msgq_get(&self_ipmi_msgq, &msg, K_MSEC(1000))) {
				/* msg won't come back if using bridge command */
				if (cmd_idx == CMD_OEM_1S_MSG_IN || cmd_idx == CMD_OEM_1S_MSG_OUT) {
					tmp_cmd_saver[tmp_cmd_num] = cmd_idx;
					tmp_cmd_num++;
					continue;
				}
				shell_error(shell, "Failed to get resp netfn:0x%x cmd:0x%x...",
					    netfn_idx, cmd_idx);
				continue;
			}
			if (msg.buffer.completion_code != CC_INVALID_CMD) {
				tmp_cmd_saver[tmp_cmd_num] = cmd_idx;
				tmp_cmd_num++;
			}
		}
		if (tmp_cmd_num) {
			shell_print(shell, "* netfn %xh:", netfn_idx);
			shell_hexdump(shell, tmp_cmd_saver, tmp_cmd_num);
		}
	}
	shell_print(
		shell,
		"------------------------------------------------------------------------------");
}
