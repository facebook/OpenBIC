#include <zephyr.h>
#include <shell/shell.h>
#include "plat_shell_e1s.h"

/* Sub-command Level 2 of command test */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_e1s_cmds,
			       SHELL_CMD(power, NULL, "Stress E1S power consumption",
					 cmd_stress_e1s_pwr),
			       SHELL_SUBCMD_SET_END);

/* Sub-command Level 1 of command test */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_test_cmds,
			       SHELL_CMD(e1s, &sub_e1s_cmds, "E1S related command", NULL),
			       SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(test, &sub_test_cmds, "Test commands for GT", NULL);
