#include <stdlib.h>
#include <shell/shell.h>

#include "plat_pldm_sensor.h"
#include "plat_cpld.h"
#include "plat_class.h"

LOG_MODULE_REGISTER(plat_clock_ctrl_shell);
enum clock_ctrl_list {
	CLK_GEN_48M,
	CLK_BUF_100M,
	CLK_GEN_100M,
	CLK_GEN_312M,
};

typedef struct power_good_status {
	uint8_t index;
	uint8_t bit_loc;
	uint8_t cpld_offsets;
	uint8_t *power_rail_name;
	uint8_t enable_value;
	uint8_t disable_value;

} power_good_status;

power_good_status clcok_control_table[] = {
	// VR Power Good pin reading(0x07)
	{ CLK_GEN_48M, 7, VR_AND_CLK_EN, "CLK_GEN_48M", 1, 0 },
	{ CLK_BUF_100M, 6, VR_AND_CLK_EN, "CLK_BUF_100M", 0, 1 },
	{ CLK_GEN_100M, 2, VR_AND_CLK_EN_PIN_CTRL, "CLK_GEN_100M", 1,
	  0 }, //TODO:To be implemented​
	{ CLK_GEN_312M, 0, VR_AND_CLK_EN_PIN_CTRL, "CLK_GEN_312M", 0, 1 },
};

bool clk_ctrl_set_cmds(uint8_t cpld_offset, uint8_t bit, uint8_t value)
{
	if (!set_cpld_bit(cpld_offset, bit, value))
		return false;
	return true;
}

void set_clk_ctrl_en(const struct shell *shell, size_t argc, char **argv)
{
	// if type in "clock_control enable CLK_BUF_100M" than will show "enable CLK_BUF_100M successfully/failed !"

	if (strcmp(argv[1], "CLK_GEN_48M") == 0) {
		if (!clk_ctrl_set_cmds(clcok_control_table[CLK_GEN_48M].cpld_offsets,
				       clcok_control_table[CLK_GEN_48M].bit_loc,
				       clcok_control_table[CLK_GEN_48M].enable_value)) {
			shell_error(shell, "enable %s failed !", argv[1]);
		} else {
			shell_print(shell, "enable %s successfully !", argv[1]);
		}
	}

	else if (strcmp(argv[1], "CLK_BUF_100M") == 0) {
		if (!clk_ctrl_set_cmds(clcok_control_table[CLK_BUF_100M].cpld_offsets,
				       clcok_control_table[CLK_BUF_100M].bit_loc,
				       clcok_control_table[CLK_BUF_100M].enable_value)) {
			shell_error(shell, "enable %s failed !", argv[1]);
		} else {
			shell_print(shell, "enable %s successfully !", argv[1]);
		}
	} else if (strcmp(argv[1], "CLK_GEN_100M") == 0) {
		if (!clk_ctrl_set_cmds(clcok_control_table[CLK_GEN_100M].cpld_offsets,
				       clcok_control_table[CLK_GEN_100M].bit_loc,
				       clcok_control_table[CLK_GEN_100M].enable_value)) {
			shell_error(shell, "enable %s failed !", argv[1]);
		} else {
			shell_print(shell, "enable %s successfully !", argv[1]);
		}
	} else if (strcmp(argv[1], "CLK_GEN_312M") == 0) {
		if (!clk_ctrl_set_cmds(clcok_control_table[CLK_GEN_312M].cpld_offsets,
				       clcok_control_table[CLK_GEN_312M].bit_loc,
				       clcok_control_table[CLK_GEN_312M].enable_value)) {
			shell_error(shell, "enable %s failed !", argv[1]);
		} else {
			shell_print(shell, "enable %s successfully !", argv[1]);
		}
	} else
		shell_error(shell, "set enable unknown name: %s.", argv[1]);
}

void set_clk_ctrl_dis(const struct shell *shell, size_t argc, char **argv)
{
	if (strcmp(argv[1], "CLK_GEN_48M") == 0) {
		if (!clk_ctrl_set_cmds(clcok_control_table[CLK_GEN_48M].cpld_offsets,
				       clcok_control_table[CLK_GEN_48M].bit_loc,
				       clcok_control_table[CLK_GEN_48M].disable_value)) {
			shell_error(shell, "disable %s failed !", argv[1]);
		} else {
			shell_print(shell, "disable %s successfully !", argv[1]);
		}
	}

	else if (strcmp(argv[1], "CLK_BUF_100M") == 0) {
		if (!clk_ctrl_set_cmds(clcok_control_table[CLK_BUF_100M].cpld_offsets,
				       clcok_control_table[CLK_BUF_100M].bit_loc,
				       clcok_control_table[CLK_BUF_100M].disable_value)) {
			shell_error(shell, "disable %s failed !", argv[1]);
		} else {
			shell_print(shell, "disable %s successfully !", argv[1]);
		}
	} else if (strcmp(argv[1], "CLK_GEN_100M") == 0) {
		if (!clk_ctrl_set_cmds(clcok_control_table[CLK_GEN_100M].cpld_offsets,
				       clcok_control_table[CLK_GEN_100M].bit_loc,
				       clcok_control_table[CLK_GEN_100M].disable_value)) {
			shell_error(shell, "disable %s failed !", argv[1]);
		} else {
			shell_print(shell, "disable %s successfully !", argv[1]);
		}
	} else if (strcmp(argv[1], "CLK_GEN_312M") == 0) {
		if (!clk_ctrl_set_cmds(clcok_control_table[CLK_GEN_312M].cpld_offsets,
				       clcok_control_table[CLK_GEN_312M].bit_loc,
				       clcok_control_table[CLK_GEN_312M].disable_value)) {
			shell_error(shell, "disable %s failed !", argv[1]);
		} else {
			shell_print(shell, "disable %s successfully !", argv[1]);
		}
	} else
		shell_error(shell, "set disable unknown name: %s.", argv[1]);
}
void clk_ctrl_get_cmds(const struct shell *shell, size_t argc, char **argv)
{
	/*
	"clock_control get" will show below value
	CLK_GEN_48M      1
	CLK_BUF_100M     1
	CLK_GEN_100M     1
	CLK_GEN_312M     1
	*/
	for (int i = 0; i < sizeof(clcok_control_table) / sizeof(power_good_status); i++) {
		uint8_t read_value = 0;
		plat_read_cpld(clcok_control_table[i].cpld_offsets, &read_value, 1);
		shell_info(shell, "%-20s %d", clcok_control_table[i].power_rail_name,
			   (read_value >> clcok_control_table[i].bit_loc) & 0x01);
	}
}

SHELL_STATIC_SUBCMD_SET_CREATE(clk_ctrl_en_cmds, SHELL_CMD(CLK_GEN_48M, NULL, "CLK_GEN_48M", NULL),
			       SHELL_CMD(CLK_BUF_100M, NULL, "CLK_BUF_100M", NULL),
			       SHELL_CMD(CLK_GEN_100M, NULL, "CLK_GEN_100M", NULL),
			       SHELL_CMD(CLK_GEN_312M, NULL, "CLK_GEN_312M", NULL),
			       SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(clk_ctrl_dis_cmds, SHELL_CMD(CLK_GEN_48M, NULL, "CLK_GEN_48M", NULL),
			       SHELL_CMD(CLK_BUF_100M, NULL, "CLK_BUF_100M", NULL),
			       SHELL_CMD(CLK_GEN_100M, NULL, "CLK_GEN_100M", NULL),
			       SHELL_CMD(CLK_GEN_312M, NULL, "CLK_GEN_312M", NULL),
			       SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(
	clock_control_cmds, SHELL_CMD(get, NULL, "get clock control", clk_ctrl_get_cmds),
	SHELL_CMD(enable, &clk_ctrl_en_cmds, "enable clock control", set_clk_ctrl_en),
	SHELL_CMD(disable, &clk_ctrl_dis_cmds, "disable clock control", set_clk_ctrl_dis),
	SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(clock_control, &clock_control_cmds, "clock_control commands", NULL);