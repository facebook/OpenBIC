#ifndef INFO_SHELL_H
#define INFO_SHELL_H

#include <shell/shell.h>

int cmd_info_print(const struct shell *shell, size_t argc, char **argv);

/* Sensor sub command */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_info_cmds,
			       SHELL_CMD(all, NULL, "List all platform info.", cmd_info_print),
			       SHELL_SUBCMD_SET_END);

#endif
