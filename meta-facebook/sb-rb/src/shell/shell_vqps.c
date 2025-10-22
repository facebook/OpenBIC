#include <stdlib.h>
#include <shell/shell.h>

#include "plat_pldm_sensor.h"
#include "plat_cpld.h"
#include "plat_class.h"

LOG_MODULE_REGISTER(plat_vqps_shell);

void set_cmd_vqps(const struct shell *shell, size_t argc, char **argv)
{
	// if type in "vqps set MEDHA0_VQPS_U_EN 1" than will show "set MEDHA0_VQPS_U_EN 1"
	shell_info(shell, "set %s %s", argv[0], argv[1]);
	// set "pin" in 1 or 0
	if (strcmp(argv[0], "MEDHA0_VQPS_TOP_EN") == 0)
		set_cpld_bit(ASIC_VQPS, 4, atoi(argv[1]));
	else if (strcmp(argv[0], "MEDHA1_VQPS_TOP_EN") == 0)
		set_cpld_bit(ASIC_VQPS, 3, atoi(argv[1]));
	else if (strcmp(argv[0], "MEDHA0_VQPS_U_EN") == 0)
		set_cpld_bit(ASIC_VQPS, 2, atoi(argv[1]));
	else if (strcmp(argv[0], "MEDHA1_VQPS_U_EN") == 0)
		set_cpld_bit(ASIC_VQPS, 1, atoi(argv[1]));
	else if (strcmp(argv[0], "HAMSA_VQPS_EFUSE_USER") == 0)
		set_cpld_bit(ASIC_VQPS, 0, atoi(argv[1]));
	else
		shell_error(shell, "Unknown name: %s.", argv[0]);
}

void vqps_get_cmds(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t vqps_status = 0;
	plat_read_cpld(ASIC_VQPS, &vqps_status, 1);
	/*
	"vqps get" will show below value
	MEDHA0_VQPS_TOP_EN : 0
	MEDHA1_VQPS_TOP_EN : 0
	MEDHA0_VQPS_U_EN : 0
	MEDHA1_VQPS_U_EN : 0
	HAMSA_VQPS_EFUSE_USER : 0
	*/
	shell_info(shell, "MEDHA0_VQPS_TOP_EN : %d", (vqps_status >> 4) & 0x1);
	shell_info(shell, "MEDHA1_VQPS_TOP_EN : %d", (vqps_status >> 3) & 0x1);
	shell_info(shell, "MEDHA0_VQPS_U_EN : %d", (vqps_status >> 2) & 0x1);
	shell_info(shell, "MEDHA1_VQPS_U_EN : %d", (vqps_status >> 1) & 0x1);
	shell_info(shell, "HAMSA_VQPS_EFUSE_USER : %d", (vqps_status >> 0) & 0x1);
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	vqps_set_cmds, SHELL_CMD(MEDHA0_VQPS_TOP_EN, NULL, "MEDHA0_VQPS_TOP_EN", set_cmd_vqps),
	SHELL_CMD(MEDHA1_VQPS_TOP_EN, NULL, "MEDHA1_VQPS_TOP_EN", set_cmd_vqps),
	SHELL_CMD(MEDHA0_VQPS_U_EN, NULL, "MEDHA0_VQPS_U_EN", set_cmd_vqps),
	SHELL_CMD(MEDHA1_VQPS_U_EN, NULL, "MEDHA1_VQPS_U_EN", set_cmd_vqps),
	SHELL_CMD(HAMSA_VQPS_EFUSE_USER, NULL, "HAMSA_VQPS_EFUSE_USER", set_cmd_vqps),
	SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(vqps_sub_cmds,
			       SHELL_CMD(get, NULL, "get vqps status", vqps_get_cmds),
			       SHELL_CMD(set, &vqps_set_cmds, "set vqps status", NULL),
			       SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(vqps, &vqps_sub_cmds, "vqps commands", NULL);