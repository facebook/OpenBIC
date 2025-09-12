#include <stdlib.h>
#include <shell/shell.h>

#include "plat_cpld.h"

// iris power command
static bool iris_power_control(uint8_t onoff)
{
	uint8_t tmp = onoff ? 0x80 : 0x00;
	return plat_write_cpld(CPLD_OFFSET_MMC_PWR_EN, &tmp);
}
void cmd_iris_power_on(const struct shell *shell, size_t argc, char **argv)
{
	if (!iris_power_control(1))
		shell_warn(shell, "iris power on fail!");
}
void cmd_iris_power_off(const struct shell *shell, size_t argc, char **argv)
{
	if (!iris_power_control(0))
		shell_warn(shell, "iris power off fail!");
}

void cmd_iris_power_cycle(const struct shell *shell, size_t argc, char **argv)
{
	if (!iris_power_control(0))
		shell_warn(shell, "iris power cycle(off) fail!");
	k_msleep(5000);
	if (!iris_power_control(1))
		shell_warn(shell, "iris power cycle(on) fail!");
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_iris_power_cmd,
			       SHELL_CMD(on, NULL, "iris power on", cmd_iris_power_on),
			       SHELL_CMD(off, NULL, "iris power off", cmd_iris_power_off),
			       SHELL_CMD(cycle, NULL, "iris power cycle", cmd_iris_power_cycle),
			       SHELL_SUBCMD_SET_END);

/* Root of command echo */
SHELL_CMD_REGISTER(iris_power, &sub_iris_power_cmd, "iris power commands", NULL);