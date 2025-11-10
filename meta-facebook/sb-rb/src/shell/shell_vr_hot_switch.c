#include <stdlib.h>
#include <shell/shell.h>

#include "plat_pldm_sensor.h"
#include "plat_cpld.h"
#include "plat_class.h"

#define ENABLE 1
#define DISABLE 0
#define VR_HOT_SWITCH_BIT 0

LOG_MODULE_REGISTER(plat_vr_hot_switch_shell);

void set_cmd_vr_hot_switch_enable(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 1) {
		shell_warn(shell, "Usage: vr_hot_switch_enable");
		return;
	}

	if (set_cpld_bit(ASIC_VR_HOT_SWITCH, VR_HOT_SWITCH_BIT, ENABLE))
		shell_info(shell, "vr_hot_switch set enable");
	else
		shell_error(shell, "vr_hot_switch set enable failed");
}

void set_cmd_vr_hot_switch_disable(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 1) {
		shell_warn(shell, "Usage: vr_hot_switch_disable");
		return;
	}

	if (set_cpld_bit(ASIC_VR_HOT_SWITCH, VR_HOT_SWITCH_BIT, DISABLE))
		shell_info(shell, "vr_hot_switch set disable");
	else
		shell_error(shell, "vr_hot_switch set disable failed");
}

void vr_hot_switch_get_cmds(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t vr_hot_switch_status = 0;
	plat_read_cpld(ASIC_VR_HOT_SWITCH, &vr_hot_switch_status, 1);
	// vr host value is 0-bit, check if enable
	if (vr_hot_switch_status & BIT(0))
		shell_print(shell, "vr_hot_switch enable");
	else
		shell_print(shell, "vr_hot_switch disable");
}

SHELL_STATIC_SUBCMD_SET_CREATE(vr_hot_switch_set_cmds,
			       SHELL_CMD(enable, NULL, "enable", set_cmd_vr_hot_switch_enable),
			       SHELL_CMD(disable, NULL, "disable", set_cmd_vr_hot_switch_disable),
			       SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(vr_hot_switch_sub_cmds,
			       SHELL_CMD(get, NULL, "get vr_hot_switch status",
					 vr_hot_switch_get_cmds),
			       SHELL_CMD(set, &vr_hot_switch_set_cmds, "set vr_hot_switch status",
					 NULL),
			       SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(vr_hot_switch, &vr_hot_switch_sub_cmds, "vr_hot_switch commands", NULL);