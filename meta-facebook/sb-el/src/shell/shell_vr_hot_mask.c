#include <stdlib.h>
#include <shell/shell.h>

#include "plat_pldm_sensor.h"
#include "plat_cpld.h"
#include "plat_class.h"

LOG_MODULE_REGISTER(plat_vr_hot_mask_shell);

void vr_hot_mask_set_cmds(const struct shell *shell, size_t argc, char **argv)
{
	if (get_asic_board_id() != ASIC_BOARD_ID_ELECTRA) {
		shell_warn(shell, "Rainbow support only!");
		return;
	}

	if (argc != 2) {
		shell_warn(shell, "Usage: vr_hot_mask set <enable/disable>");
		return;
	}

	if (strcmp(argv[1], "enable") == 0) {
		set_plat_vr_hot_mask_flag(true);
		shell_info(shell, "vr_hot_mask set enable");
		return;
	}

	if (strcmp(argv[1], "disable") == 0) {
		set_plat_vr_hot_mask_flag(false);
		shell_info(shell, "vr_hot_mask set disable");
		return;
	}
}

void vr_hot_mask_get_cmds(const struct shell *shell, size_t argc, char **argv)
{
	if (get_asic_board_id() != ASIC_BOARD_ID_ELECTRA) {
		shell_warn(shell, "Rainbow support only!");
		return;
	}

	shell_print(shell, "vr_hot_mask %s", get_plat_vr_hot_mask_flag() ? "enable" : "disable");
}

SHELL_STATIC_SUBCMD_SET_CREATE(vr_hot_mask_sub_cmds,
			       SHELL_CMD(get, NULL, "vr_hot_mask get", vr_hot_mask_get_cmds),
			       SHELL_CMD(set, NULL, "vr_hot_mask set <enable/disable>",
					 vr_hot_mask_set_cmds),
			       SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(vr_hot_mask, &vr_hot_mask_sub_cmds, "vr_hot_mask commands", NULL);