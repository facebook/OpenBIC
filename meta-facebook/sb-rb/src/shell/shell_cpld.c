#include <stdlib.h>
#include <shell/shell.h>

#include "plat_cpld.h"

void cmd_cpld_read(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t offset = strtoul(argv[1], NULL, 16);
	uint8_t data = 0;

	if (!plat_read_cpld(offset, &data)) {
		shell_warn(shell, "cpld read 0x%02x fail", offset);
		return;
	}

	shell_warn(shell, "0x%02x", data);
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_cpld_test_cmds,
			       SHELL_CMD(read, NULL, "cpld read register command", cmd_cpld_read),
			       SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(cpld_test, &sub_cpld_test_cmds, "cpld test commands", NULL);