#include <stdlib.h>
#include <shell/shell.h>

#include "plat_pldm_sensor.h"
#include "plat_cpld.h"
#include "plat_class.h"

/*
000:MEDHA0
001:MEDHA1
010:OWL_E
011:OWL_W
100:HAMSA
*/
enum jtag_mux { MUX_HAMSA = 0, MUX_MEDHA_0, MUX_MEDHA_1, MUX_OWL_E, MUX_OWL_W, MUX_JTAG_MUX_MAX };

LOG_MODULE_REGISTER(plat_jtag_mux_shell);

void set_cmd_jtag_mux(const struct shell *shell, size_t argc, char **argv)
{
	//jtag_mux set <MEDHA_0 | MEDHA_1 | OWL_E | OWL_W | HAMSA>
	// if argc not match
	if (argc != 1) {
		shell_warn(shell, "Help: jtag_mux set <MEDHA_0 | MEDHA_1 | OWL_E | OWL_W | HAMSA>");
		return;
	}
	uint8_t write_in_data = 0;
	if (strcmp(argv[0], "MEDHA_0") == 0) {
		write_in_data = MUX_MEDHA_0;
		plat_write_cpld(ASIC_JTAG_MUX_SEL, &write_in_data);
		shell_info(shell, "jtag_mux set to MEDHA_0");
	} else if (strcmp(argv[0], "MEDHA_1") == 0) {
		write_in_data = MUX_MEDHA_1;
		plat_write_cpld(ASIC_JTAG_MUX_SEL, &write_in_data);
		shell_info(shell, "jtag_mux set to MEDHA_1");
	} else if (strcmp(argv[0], "OWL_E") == 0) {
		write_in_data = MUX_OWL_E;
		plat_write_cpld(ASIC_JTAG_MUX_SEL, &write_in_data);
		shell_info(shell, "jtag_mux set to OWL_E");
	} else if (strcmp(argv[0], "OWL_W") == 0) {
		write_in_data = MUX_OWL_W;
		plat_write_cpld(ASIC_JTAG_MUX_SEL, &write_in_data);
		shell_info(shell, "jtag_mux set to OWL_W");
	} else if (strcmp(argv[0], "HAMSA") == 0) {
		write_in_data = MUX_HAMSA;
		plat_write_cpld(ASIC_JTAG_MUX_SEL, &write_in_data);
		shell_info(shell, "jtag_mux set to HAMSA");
	}

	else {
		shell_error(
			shell,
			"Type wrong sub command, Help: jtag_mux set <MEDHA_0 | MEDHA_1 | OWL_E | OWL_W | HAMSA>");
	}
}

static const char *const jtag_mux_table[MUX_JTAG_MUX_MAX] = {
	[MUX_MEDHA_0] = "MEDHA_0", [MUX_MEDHA_1] = "MEDHA_1", [MUX_OWL_E] = "OWL_E",
	[MUX_OWL_W] = "OWL_W",	   [MUX_HAMSA] = "HAMSA",
};

void jtag_mux_get_cmds(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t jtag_mux_status = 0;
	plat_read_cpld(ASIC_JTAG_MUX_SEL, &jtag_mux_status, 1);
	/*
	if user type "jtag_mux get" than show above value
	JTAG MUX : <MEDHA_0 | MEDHA_1 | OWL_E | OWL_W | HAMSA>
	*/
	for (int i = 0; i < MUX_JTAG_MUX_MAX; i++) {
		if (jtag_mux_status == i) {
			shell_info(shell, "JTAG MUX : %s", jtag_mux_table[i]);
			break;
		}
	}
}

SHELL_STATIC_SUBCMD_SET_CREATE(jtag_mux_set_cmds,
			       SHELL_CMD(MEDHA_0, NULL, "MEDHA_0", set_cmd_jtag_mux),
			       SHELL_CMD(MEDHA_1, NULL, "MEDHA_1", set_cmd_jtag_mux),
			       SHELL_CMD(OWL_E, NULL, "OWL_E", set_cmd_jtag_mux),
			       SHELL_CMD(OWL_W, NULL, "OWL_W", set_cmd_jtag_mux),
			       SHELL_CMD(HAMSA, NULL, "HAMSA", set_cmd_jtag_mux),
			       SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(jtag_mux_sub_cmds,
			       SHELL_CMD(get, NULL, "get jtag_mux status", jtag_mux_get_cmds),
			       SHELL_CMD(set, &jtag_mux_set_cmds, "set jtag_mux status", NULL),
			       SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(jtag_mux, &jtag_mux_sub_cmds, "jtag_mux commands", NULL);