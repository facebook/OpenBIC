#include <stdlib.h>
#include <shell/shell.h>
#include "plat_cpld.h"
#include <logging/log.h>
#include <shell_plat_power_sequence.h>
// iris power command

#define enable 0x01
#define disable 0x00

LOG_MODULE_REGISTER(shell_iris_power);

typedef struct steps_on_struct {
	uint8_t cpld_offset;
	uint8_t bit;
	uint8_t *name;
} steps_on_struct;

static steps_on_struct steps_on[] = {
	// follow pwr sequence order
	{ VR_AND_CLK_EN_PIN_CTRL, 1, "FM_PLD_UBC_EN" }, //p12V
	{ VR_4_EN, 6, "FM_P3V3_EN" }, //FM_P3V3_EN
	{ VR_4_EN, 7, "FM_P5V_EN" }, //FM_P5V_EN
	{ VR_4_EN, 5, "LDO_IN_1V2_EN_R" }, //LDO_IN_1V2_EN_R
	{ VR_4_EN, 4, "FM_P1V80_EN" }, //FM_P1V80_EN
	{ VR_3_EN, 0, "FM_AVDD_HCSL_EN" }, //FM_AVDD_HCSL_EN
	{ VR_1_EN, 3, "FM_HAMSA_VDD_EN" }, //FM_HAMSA_VDD_EN
	{ VR_1_EN, 7, "MEDHA1_VDD_EN" }, //MEDHA1_VDD_EN
	{ VR_1_EN, 6, "FM_MEDHA0_VDD_EN" }, //FM_MEDHA0_VDD_EN
	{ VR_1_EN, 5, "FM_OWL_E_VDD_EN" }, //FM_OWL_E_VDD_EN
	{ VR_1_EN, 4, "FM_OWL_W_VDD_EN" }, //FM_OWL_W_VDD_EN
	{ VR_1_EN, 1, "FM_MAX_M_VDD_EN" }, //FM_MAX_M_VDD_EN
	{ VR_1_EN, 0, "FM_MAX_N_VDD_EN" }, //FM_MAX_N_VDD_EN
	{ VR_1_EN, 2, "FM_MAX_S_VDD_EN" }, //FM_MAX_S_VDD_EN
	{ VR_AND_CLK_EN, 3, "FM_OWL_E_TRVDD0P75_EN" }, //FM_OWL_E_TRVDD0P75_EN
	{ VR_AND_CLK_EN, 2, "FM_OWL_W_TRVDD0P75_EN" }, //FM_OWL_W_TRVDD0P75_EN
	{ VR_2_EN, 4, "FM_VDDPHY_HBM0_HBM2_HBM4_HBM6_EN" }, //FM_VDDPHY_HBM0_HBM2_HBM4_HBM6_EN
	{ VR_2_EN, 0, "FM_VDDPHY_HBM1_HBM3_HBM5_HBM7_EN" }, //FM_VDDPHY_HBM1_HBM3_HBM5_HBM7_EN
	{ VR_4_EN, 1, "FM_P1V5_PLL_VDDA_OWL_EN" }, //FM_P1V5_PLL_VDDA_OWL_EN
	{ VR_4_EN, 0, "FM_P1V5_PLL_VDDA_SOC_EN" }, //FM_P1V5_PLL_VDDA_SOC_EN
	{ VR_4_EN, 2, "FM_PLL_VDDA15_HBM0_HBM2_EN" }, //FM_PLL_VDDA15_HBM0_HBM2_EN
	{ VR_4_EN, 5, "FM_PLL_VDDA15_HBM1_HBM3_EN" }, //FM_PLL_VDDA15_HBM1_HBM3_EN
	{ VR_4_EN, 6, "FM_PLL_VDDA15_HBM4_HBM6_EN" }, //FM_PLL_VDDA15_HBM4_HBM6_EN
	{ VR_4_EN, 4, "FM_PLL_VDDA15_HBM5_HBM7_EN" }, //FM_PLL_VDDA15_HBM5_HBM7_EN
	{ VR_AND_CLK_EN, 6, "FM_AEGIS_CLK_100MHZ_EN_N" }, //FM_AEGIS_CLK_100MHZ_EN_N
	{ VR_AND_CLK_EN, 7, "FM_AEGIS_CLK_48MHZ_EN" }, //FM_AEGIS_CLK_48MHZ_EN
	{ VR_AND_CLK_EN_PIN_CTRL, 0, "FM_IRIS_CLK_312MHZ_EN_N" }, //FM_IRIS_CLK_312MHZ_EN_N
	{ VR_2_EN, 5, "FM_VPP_HBM0_HBM2_HBM4_HBM6_EN" }, //FM_VPP_HBM0_HBM2_HBM4_HBM6_EN
	{ VR_2_EN, 1, "FM_VPP_HBM1_HBM3_HBM5_HBM7_EN" }, //FM_VPP_HBM1_HBM3_HBM5_HBM7_EN
	{ VR_2_EN, 6, "FM_VDDQC_HBM0_HBM2_HBM4_HBM6_EN" }, //FM_VDDQC_HBM0_HBM2_HBM4_HBM6_EN
	{ VR_2_EN, 2, "FM_VDDQC_HBM1_HBM3_HBM5_HBM7_EN" }, //FM_VDDQC_HBM1_HBM3_HBM5_HBM7_EN
	{ VR_2_EN, 7, "FM_VDDQL_HBM0_HBM2_HBM4_HBM6_EN" }, //FM_VDDQL_HBM0_HBM2_HBM4_HBM6_EN
	{ VR_2_EN, 3, "FM_VDDQL_HBM1_HBM3_HBM5_HBM7_EN" }, //FM_VDDQL_HBM1_HBM3_HBM5_HBM7_EN
	{ VR_AND_CLK_EN, 1, "FM_HAMSA_AVDD_PCIE_EN" }, //FM_HAMSA_AVDD_PCIE_EN
	{ VR_AND_CLK_EN, 5, "FM_OWL_E_TRVDD0P9_EN" }, //FM_OWL_E_TRVDD0P9_EN
	{ VR_AND_CLK_EN, 4, "FM_OWL_W_TRVDD0P9_EN" }, //FM_OWL_W_TRVDD0P9_EN
	{ VR_3_EN, 3, "FM_PVDD0P9_EN" }, //FM_PVDD0P9_EN (OWL_E_PVDD0P9, OWL_W_PVDD0P9)
	{ VR_3_EN, 2, "FM_P1V5_RVDD_EN" }, //FM_P1V5_RVDD_EN (OWL_E_RVDD1P5, OWL_W_RVDD1P5)
	{ VR_AND_CLK_EN, 0, "FM_HAMSA_VDDHRXTX_PCIE_EN" }, //FM_HAMSA_VDDHRXTX_PCIE_EN
	{ VR_3_EN, 2, "FM_PVDD1P5_EN" }, //FM_PVDD1P5_EN
	{ RESET, 5, "HAMSA_POWER_ON_RESET_PLD_L" }, //HAMSA_POWER_ON_RESET_PLD_L
	{ RESET, 4, "MEDHA0_POWER_ON_RESET_PLD_L" }, //MEDHA0_POWER_ON_RESET_PLD_L
	{ RESET, 3, "MEDHA1_POWER_ON_RESET_PLD_L" }, //MEDHA1_POWER_ON_RESET_PLD_L
	{ RESET, 2, "HAMSA_SYS_RST_PLD_L" }, //HAMSA_SYS_RST_PLD_L
	{ RESET, 1, "MEDHA0_SYS_RST_PLD_L" }, //MEDHA0_SYS_RST_PLD_L
	{ RESET, 0, "MEDHA1_SYS_RST_PLD_L" }, //MEDHA1_SYS_RST_PLD_L
};

#define MAX_STEPS (sizeof(steps_on) / sizeof(steps_on[0]))

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

bool inverse_bit(uint8_t offset, uint8_t bit)
{
	uint8_t tmp_data = 0;
	bool error_flag = false;
	if (!plat_read_cpld(offset, &tmp_data, 1))
		error_flag = true;
	LOG_INF("read offset 0x%x = 0x%x", offset, tmp_data);
	tmp_data ^= BIT(bit);
	LOG_WRN("write offset 0x%x = 0x%x", offset, tmp_data);
	if (!plat_write_cpld(offset, &tmp_data))
		error_flag = true;
	return error_flag;
}

int power_steps = 0;
void cmd_iris_steps_on(const struct shell *shell, size_t argc, char **argv)
{
	if (power_steps >= MAX_STEPS) {
		LOG_WRN("no more steps");
		return;
	}
	// first step need to inverse all enable bit
	if (power_steps == 0) {
		for (int i = 0; i < MAX_STEPS; i++) {
			if (inverse_bit(steps_on[i].cpld_offset, steps_on[i].bit)) {
				LOG_ERR("set %-35s fail", steps_on[i].name);
			}
		}

		if (!iris_power_control(1)) {
			LOG_WRN("iris power on fail!");
			return;
		}
	}

	LOG_DBG("set %s on, offset = 0x%x, bit = %d", steps_on[power_steps].name,
		steps_on[power_steps].cpld_offset, steps_on[power_steps].bit);

	// turn on the steps on bit
	if (inverse_bit(steps_on[power_steps].cpld_offset, steps_on[power_steps].bit)) {
		LOG_ERR("set %-35s fail", steps_on[power_steps].name);
		return;
	} else
		LOG_INF("set %-35s success", steps_on[power_steps].name);

	power_steps += 1;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_iris_power_cmd,
			       SHELL_CMD(on, NULL, "iris power on", cmd_iris_power_on),
			       SHELL_CMD(off, NULL, "iris power off", cmd_iris_power_off),
			       SHELL_CMD(cycle, NULL, "iris power cycle", cmd_iris_power_cycle),
			       SHELL_CMD(steps_on, NULL, "iris power steps_on", cmd_iris_steps_on),
			       SHELL_SUBCMD_SET_END);

/* Root of command echo */
SHELL_CMD_REGISTER(iris_power, &sub_iris_power_cmd, "iris power commands", NULL);