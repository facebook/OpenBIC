#include "commands/gpio_shell.h"
#include "commands/info_shell.h"
#include "commands/sensor_shell.h"

/* MAIN command */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_platform_cmds,
			       SHELL_CMD(info, NULL, "Platform info.", cmd_info_print),
			       SHELL_CMD(gpio, &sub_gpio_cmds, "GPIO relative command.", NULL),
			       SHELL_CMD(sensor, &sub_sensor_cmds, "SENSOR relative command.",
					 NULL),
			       SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(platform, &sub_platform_cmds, "Platform commands", NULL);
