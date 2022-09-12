#ifndef IPMI_SHELL_H
#define IPMI_SHELL_H

#include <stdlib.h>
#include <shell/shell.h>

void cmd_ipmi_list(const struct shell *shell, size_t argc, char **argv);
void cmd_ipmi_raw(const struct shell *shell, size_t argc, char **argv);

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_ipmi_cmds, SHELL_CMD(scan, NULL, "Scanning all supported commands", cmd_ipmi_list),
	SHELL_CMD(raw, NULL, "Send raw command", cmd_ipmi_raw), SHELL_SUBCMD_SET_END);

#endif
