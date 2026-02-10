#include <stdlib.h>
#include <shell/shell.h>

#include "plat_pldm_sensor.h"
#include "plat_cpld.h"

// echo command
void cmd_echo(const struct shell *shell, size_t argc, char **argv)
{
	for (size_t i = 1; i < argc; i++) {
		printf("%s ", argv[i]);
	}
	printf("\n");
}

/* Root of command echo */
SHELL_CMD_REGISTER(echo, NULL, "echo commands", cmd_echo);