#ifndef DEBUG_SHELL_H
#define DEBUG_SHELL_H

#include <stdlib.h>
#include <shell/shell.h>
#include <drivers/gpio.h>

void cmd_debug_list(const struct shell *shell, size_t argc, char **argv);
void cmd_debug_set(const struct shell *shell, size_t argc, char **argv);
void cmd_debug_set_all(const struct shell *shell, size_t argc, char **argv);
void cmd_debug_disable(const struct shell *shell, size_t argc, char **argv);

/* DEBUG sub commands */
SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_debug_cmds,
	SHELL_CMD(list, NULL, "List current log levels for all log types.", &cmd_debug_list),
	SHELL_CMD(set, NULL, "Set log level for system", &cmd_debug_set),
	SHELL_CMD(set_all, NULL, "Set log level for all systems", &cmd_debug_set_all),
	SHELL_CMD(disable, NULL, "Disable logging for all systems", &cmd_debug_disable),
	SHELL_SUBCMD_SET_END);

#endif
