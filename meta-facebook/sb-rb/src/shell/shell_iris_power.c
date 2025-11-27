#include <stdlib.h>
#include <shell/shell.h>
#include "plat_cpld.h"
#include <logging/log.h>
#include <shell_plat_power_sequence.h>
#include "plat_gpio.h"
#include "plat_isr.h"
// iris power command

#define enable 0x01
#define disable 0x00
#define NO_DEFINED 0xFF

LOG_MODULE_REGISTER(shell_iris_power);
typedef struct power_good_status {
	uint8_t index;
	uint8_t bit_loc;
	uint8_t cpld_offsets;
	uint8_t *power_rail_name;

} power_good_status;
enum power_good_status_type_for_steps_on {
	//VR Power Good pin reading(0x07)
	MODULE_PWRGD,
	PWRGD_P1V8_AUX,
	PWRGD_OWL_E_TRVDD0P9_R,
	PWRGD_OWL_W_TRVDD0P9_R,
	PWRGD_OWL_E_TRVDD0P75_R,
	PWRGD_OWL_W_TRVDD0P75_R,
	PWRGD_HAMSA_AVDD_PCIE_R,
	PWRGD_HAMSA_VDDHRXTX_PCIE_R,
	//VR Power Good pin reading(0x08)
	PWRGD_MEDHA1_VDD,
	PWRGD_MEDHA0_VDD,
	PWRGD_OWL_E_VDD_R,
	PWRGD_OWL_W_VDD_R,
	PWRGD_HAMSA_VDD_R,
	PWRGD_MAX_S_VDD_R,
	PWRGD_MAX_M_VDD_R,
	PWRGD_MAX_N_VDD_R,
	//VR Power Good pin reading(0x09)
	PWRGD_VDDQL_HBM0_HBM2_HBM4_HBM6_R,
	PWRGD_VDDQC_HBM0_HBM2_HBM4_HBM6_R,
	PWRGD_VPP_HBM0_HBM2_HBM4_HBM6_R,
	PWRGD_VDDPHY_HBM0_HBM2_HBM4_HBM6_R,
	PWRGD_VDDQL_HBM1_HBM3_HBM5_HBM7_R,
	PWRGD_VDDQC_HBM1_HBM3_HBM5_HBM7_R,
	PWRGD_VPP_HBM1_HBM3_HBM5_HBM7_R,
	PWRGD_VDDPHY_HBM1_HBM3_HBM5_HBM7_R,
	//VR Power Good pin reading(0x0A)
	PWRGD_PLL_VDDA15_HBM0_HBM2,
	PWRGD_PLL_VDDA15_HBM4_HBM6,
	PWRGD_PLL_VDDA15_HBM1_HBM3,
	PWRGD_PLL_VDDA15_HBM5_HBM7,
	PWRGD_P0V9_OWL_E_PVDD,
	PWRGD_P0V9_OWL_W_PVDD,
	PWRGD_P1V5_E_RVDD,
	PWRGD_P1V5_W_RVDD,
	//VR Power Good pin reading(0x0B)
	P12V_UBC1_PWRGD,
	P12V_UBC2_PWRGD,
	PWRGD_P5V_R,
	PWRGD_P3V3_R,
	PWRGD_P1V8_R,
	PWRGD_LDO_IN_1V2_R,
	PWRGD_P1V5_PLL_VDDA_OWL_E,
	PWRGD_P1V5_PLL_VDDA_SOC,
	//VR Power Good pin reading(0x0C)
	PWRGD_PVDD1P5,
	PWRGD_P0V75_AVDD_HCSL,
	PWRGD_P1V5_PLL_VDDA_OWL_W,
	PWRGD_MAX
};
power_good_status power_good_status_table_for_steps_on[] = {
	// VR Power Good pin reading(0x07)
	{ MODULE_PWRGD, 7, VR_PWRGD_PIN_READING_1_REG, "MODULE_PWRGD" },
	{ PWRGD_P1V8_AUX, 6, VR_PWRGD_PIN_READING_1_REG, "PWRGD_P1V8_AUX" },
	{ PWRGD_OWL_E_TRVDD0P9_R, 5, VR_PWRGD_PIN_READING_1_REG, "PWRGD_OWL_E_TRVDD0P9_R" },
	{ PWRGD_OWL_W_TRVDD0P9_R, 4, VR_PWRGD_PIN_READING_1_REG, "PWRGD_OWL_W_TRVDD0P9_R" },
	{ PWRGD_OWL_E_TRVDD0P75_R, 3, VR_PWRGD_PIN_READING_1_REG, "PWRGD_OWL_E_TRVDD0P75_R" },
	{ PWRGD_OWL_W_TRVDD0P75_R, 2, VR_PWRGD_PIN_READING_1_REG, "PWRGD_OWL_W_TRVDD0P75_R" },
	{ PWRGD_HAMSA_AVDD_PCIE_R, 1, VR_PWRGD_PIN_READING_1_REG, "PWRGD_HAMSA_AVDD_PCIE_R" },
	{ PWRGD_HAMSA_VDDHRXTX_PCIE_R, 0, VR_PWRGD_PIN_READING_1_REG,
	  "PWRGD_HAMSA_VDDHRXTX_PCIE_R" },
	// VR Power Good pin reading(0x08)
	{ PWRGD_MEDHA1_VDD, 7, VR_PWRGD_PIN_READING_2_REG, "PWRGD_MEDHA1_VDD" },
	{ PWRGD_MEDHA0_VDD, 6, VR_PWRGD_PIN_READING_2_REG, "PWRGD_MEDHA0_VDD" },
	{ PWRGD_OWL_E_VDD_R, 5, VR_PWRGD_PIN_READING_2_REG, "PWRGD_OWL_E_VDD_R" },
	{ PWRGD_OWL_W_VDD_R, 4, VR_PWRGD_PIN_READING_2_REG, "PWRGD_OWL_W_VDD_R" },
	{ PWRGD_HAMSA_VDD_R, 3, VR_PWRGD_PIN_READING_2_REG, "PWRGD_HAMSA_VDD_R" },
	{ PWRGD_MAX_S_VDD_R, 2, VR_PWRGD_PIN_READING_2_REG, "PWRGD_MAX_S_VDD_R" },
	{ PWRGD_MAX_M_VDD_R, 1, VR_PWRGD_PIN_READING_2_REG, "PWRGD_MAX_M_VDD_R" },
	{ PWRGD_MAX_N_VDD_R, 0, VR_PWRGD_PIN_READING_2_REG, "PWRGD_MAX_N_VDD_R" },
	// VR Power Good pin reading(0x09)
	{ PWRGD_VDDQL_HBM0_HBM2_HBM4_HBM6_R, 7, VR_PWRGD_PIN_READING_3_REG,
	  "PWRGD_VDDQL_HBM0_HBM2_HBM4_HBM6_R" },
	{ PWRGD_VDDQC_HBM0_HBM2_HBM4_HBM6_R, 6, VR_PWRGD_PIN_READING_3_REG,
	  "PWRGD_VDDQC_HBM0_HBM2_HBM4_HBM6_R" },
	{ PWRGD_VPP_HBM0_HBM2_HBM4_HBM6_R, 5, VR_PWRGD_PIN_READING_3_REG,
	  "PWRGD_VPP_HBM0_HBM2_HBM4_HBM6_R" },
	{ PWRGD_VDDPHY_HBM0_HBM2_HBM4_HBM6_R, 4, VR_PWRGD_PIN_READING_3_REG,
	  "PWRGD_VDDPHY_HBM0_HBM2_HBM4_HBM6_R" },
	{ PWRGD_VDDQL_HBM1_HBM3_HBM5_HBM7_R, 3, VR_PWRGD_PIN_READING_3_REG,
	  "PWRGD_VDDQL_HBM1_HBM3_HBM5_HBM7_R" },
	{ PWRGD_VDDQC_HBM1_HBM3_HBM5_HBM7_R, 2, VR_PWRGD_PIN_READING_3_REG,
	  "PWRGD_VDDQC_HBM1_HBM3_HBM5_HBM7_R" },
	{ PWRGD_VPP_HBM1_HBM3_HBM5_HBM7_R, 1, VR_PWRGD_PIN_READING_3_REG,
	  "PWRGD_VPP_HBM1_HBM3_HBM5_HBM7_R" },
	{ PWRGD_VDDPHY_HBM1_HBM3_HBM5_HBM7_R, 0, VR_PWRGD_PIN_READING_3_REG,
	  "PWRGD_VDDPHY_HBM1_HBM3_HBM5_HBM7_R" },
	//VR Power Good pin reading(0x0A)
	{ PWRGD_PLL_VDDA15_HBM0_HBM2, 7, VR_PWRGD_PIN_READING_4_REG, "PWRGD_PLL_VDDA15_HBM0_HBM2" },
	{ PWRGD_PLL_VDDA15_HBM4_HBM6, 6, VR_PWRGD_PIN_READING_4_REG, "PWRGD_PLL_VDDA15_HBM4_HBM6" },
	{ PWRGD_PLL_VDDA15_HBM1_HBM3, 5, VR_PWRGD_PIN_READING_4_REG, "PWRGD_PLL_VDDA15_HBM1_HBM3" },
	{ PWRGD_PLL_VDDA15_HBM5_HBM7, 4, VR_PWRGD_PIN_READING_4_REG, "PWRGD_PLL_VDDA15_HBM5_HBM7" },
	{ PWRGD_P0V9_OWL_E_PVDD, 3, VR_PWRGD_PIN_READING_4_REG, "PWRGD_P0V9_OWL_E_PVDD" },
	{ PWRGD_P0V9_OWL_W_PVDD, 2, VR_PWRGD_PIN_READING_4_REG, "PWRGD_P0V9_OWL_W_PVDD" },
	{ PWRGD_P1V5_E_RVDD, 1, VR_PWRGD_PIN_READING_4_REG, "PWRGD_P1V5_E_RVDD" },
	{ PWRGD_P1V5_W_RVDD, 0, VR_PWRGD_PIN_READING_4_REG, "PWRGD_P1V5_W_RVDD" },
	//VR Power Good pin reading(0x0B)
	{ P12V_UBC1_PWRGD, 7, VR_PWRGD_PIN_READING_5_REG, "P12V_UBC1_PWRGD" },
	{ P12V_UBC2_PWRGD, 6, VR_PWRGD_PIN_READING_5_REG, "P12V_UBC2_PWRGD" },
	{ PWRGD_P5V_R, 5, VR_PWRGD_PIN_READING_5_REG, "PWRGD_P5V_R" },
	{ PWRGD_P3V3_R, 4, VR_PWRGD_PIN_READING_5_REG, "PWRGD_P3V3_R" },
	{ PWRGD_P1V8_R, 3, VR_PWRGD_PIN_READING_5_REG, "PWRGD_P1V8_R" },
	{ PWRGD_LDO_IN_1V2_R, 2, VR_PWRGD_PIN_READING_5_REG, "PWRGD_LDO_IN_1V2_R" },
	{ PWRGD_P1V5_PLL_VDDA_OWL_E, 1, VR_PWRGD_PIN_READING_5_REG, "PWRGD_P1V5_PLL_VDDA_OWL_E" },
	{ PWRGD_P1V5_PLL_VDDA_SOC, 0, VR_PWRGD_PIN_READING_5_REG, "PWRGD_P1V5_PLL_VDDA_SOC" },
	//VR Power Good pin reading(0x0C)
	{ PWRGD_PVDD1P5, 7, VR_PWRGD_PIN_READING_6_REG, "PWRGD_PVDD1P5" },
	{ PWRGD_P0V75_AVDD_HCSL, 6, VR_PWRGD_PIN_READING_6_REG, "PWRGD_P0V75_AVDD_HCSL" },
	{ PWRGD_P1V5_PLL_VDDA_OWL_W, 5, VR_PWRGD_PIN_READING_6_REG, "PWRGD_P1V5_PLL_VDDA_OWL_W" },
};
typedef struct steps_on_struct {
	uint8_t power_on_value;
	uint8_t cpld_offset;
	uint8_t bit;
	uint8_t *name;
	uint8_t pwrgd_idx;
} steps_on_struct;

static steps_on_struct steps_on[] = {
	// follow pwr sequence order
	{ 1, VR_AND_CLK_EN_PIN_CTRL, 1, "FM_PLD_UBC_EN", P12V_UBC1_PWRGD }, //p12V
	{ 1, VR_4_EN, 6, "FM_P3V3_EN", PWRGD_P3V3_R }, //FM_P3V3_EN
	{ 1, VR_AND_CLK_EN_PIN_CTRL, 3, "FM_P4V2_EN_R", NO_DEFINED }, //FM_P4V2_EN_R
	{ 1, VR_4_EN, 7, "FM_P5V_EN", PWRGD_P5V_R }, //FM_P5V_EN
	{ 1, VR_4_EN, 5, "LDO_IN_1V2_EN_R", PWRGD_LDO_IN_1V2_R }, //LDO_IN_1V2_EN_R
	{ 1, VR_4_EN, 4, "FM_P1V80_EN", PWRGD_P1V8_R }, //FM_P1V80_EN
	{ 1, VR_3_EN, 0, "FM_AVDD_HCSL_EN", PWRGD_P0V75_AVDD_HCSL }, //FM_AVDD_HCSL_EN
	{ 1, VR_1_EN, 3, "FM_HAMSA_VDD_EN", PWRGD_HAMSA_VDD_R }, //FM_HAMSA_VDD_EN
	{ 1, VR_1_EN, 7, "MEDHA1_VDD_EN", PWRGD_MEDHA1_VDD }, //MEDHA1_VDD_EN
	{ 1, VR_1_EN, 6, "FM_MEDHA0_VDD_EN", PWRGD_MEDHA0_VDD }, //FM_MEDHA0_VDD_EN
	{ 1, VR_1_EN, 5, "FM_OWL_E_VDD_EN", PWRGD_OWL_E_VDD_R }, //FM_OWL_E_VDD_EN
	{ 1, VR_1_EN, 4, "FM_OWL_W_VDD_EN", PWRGD_OWL_W_VDD_R }, //FM_OWL_W_VDD_EN
	{ 1, VR_1_EN, 1, "FM_MAX_M_VDD_EN", PWRGD_MAX_M_VDD_R }, //FM_MAX_M_VDD_EN
	{ 1, VR_1_EN, 0, "FM_MAX_N_VDD_EN", PWRGD_MAX_N_VDD_R }, //FM_MAX_N_VDD_EN
	{ 1, VR_1_EN, 2, "FM_MAX_S_VDD_EN", PWRGD_MAX_S_VDD_R }, //FM_MAX_S_VDD_EN
	{ 1, VR_AND_CLK_EN, 3, "FM_OWL_E_TRVDD0P75_EN",
	  PWRGD_OWL_E_TRVDD0P75_R }, //FM_OWL_E_TRVDD0P75_EN
	{ 1, VR_AND_CLK_EN, 2, "FM_OWL_W_TRVDD0P75_EN",
	  PWRGD_OWL_W_TRVDD0P75_R }, //FM_OWL_W_TRVDD0P75_EN
	{ 1, VR_2_EN, 4, "FM_VDDPHY_HBM0_HBM2_HBM4_HBM6_EN",
	  PWRGD_VDDPHY_HBM0_HBM2_HBM4_HBM6_R }, //FM_VDDPHY_HBM0_HBM2_HBM4_HBM6_EN
	{ 1, VR_2_EN, 0, "FM_VDDPHY_HBM1_HBM3_HBM5_HBM7_EN",
	  PWRGD_VDDPHY_HBM1_HBM3_HBM5_HBM7_R }, //FM_VDDPHY_HBM1_HBM3_HBM5_HBM7_EN
	{ 1, VR_4_EN, 1, "FM_P1V5_PLL_VDDA_OWL_EN",
	  PWRGD_P1V5_PLL_VDDA_OWL_E }, //FM_P1V5_PLL_VDDA_OWL_EN
	{ 1, VR_4_EN, 0, "FM_P1V5_PLL_VDDA_SOC_EN",
	  PWRGD_P1V5_PLL_VDDA_SOC }, //FM_P1V5_PLL_VDDA_SOC_EN
	{ 1, VR_3_EN, 7, "FM_PLL_VDDA15_HBM0_HBM2_EN",
	  PWRGD_PLL_VDDA15_HBM0_HBM2 }, //FM_PLL_VDDA15_HBM0_HBM2_EN
	{ 1, VR_3_EN, 5, "FM_PLL_VDDA15_HBM1_HBM3_EN",
	  PWRGD_PLL_VDDA15_HBM1_HBM3 }, //FM_PLL_VDDA15_HBM1_HBM3_EN
	{ 1, VR_3_EN, 6, "FM_PLL_VDDA15_HBM4_HBM6_EN",
	  PWRGD_PLL_VDDA15_HBM4_HBM6 }, //FM_PLL_VDDA15_HBM4_HBM6_EN
	{ 1, VR_3_EN, 4, "FM_PLL_VDDA15_HBM5_HBM7_EN",
	  PWRGD_PLL_VDDA15_HBM5_HBM7 }, //FM_PLL_VDDA15_HBM5_HBM7_EN
	{ 1, VR_AND_CLK_EN_PIN_CTRL, 2, "FM_P3V3_CLK_EN", NO_DEFINED }, //FM_P3V3_CLK_EN
	{ 0, VR_AND_CLK_EN, 6, "FM_AEGIS_CLK_100MHZ_EN_N", NO_DEFINED }, //FM_AEGIS_CLK_100MHZ_EN_N
	{ 1, VR_AND_CLK_EN, 7, "FM_AEGIS_CLK_48MHZ_EN", NO_DEFINED }, //FM_AEGIS_CLK_48MHZ_EN
	{ 0, VR_AND_CLK_EN_PIN_CTRL, 0, "FM_IRIS_CLK_312MHZ_EN_N",
	  NO_DEFINED }, //FM_IRIS_CLK_312MHZ_EN_N
	{ 1, VR_2_EN, 5, "FM_VPP_HBM0_HBM2_HBM4_HBM6_EN",
	  PWRGD_VPP_HBM0_HBM2_HBM4_HBM6_R }, //FM_VPP_HBM0_HBM2_HBM4_HBM6_EN
	{ 1, VR_2_EN, 1, "FM_VPP_HBM1_HBM3_HBM5_HBM7_EN",
	  PWRGD_VPP_HBM1_HBM3_HBM5_HBM7_R }, //FM_VPP_HBM1_HBM3_HBM5_HBM7_EN
	{ 1, VR_2_EN, 6, "FM_VDDQC_HBM0_HBM2_HBM4_HBM6_EN",
	  PWRGD_VDDQC_HBM0_HBM2_HBM4_HBM6_R }, //FM_VDDQC_HBM0_HBM2_HBM4_HBM6_EN
	{ 1, VR_2_EN, 2, "FM_VDDQC_HBM1_HBM3_HBM5_HBM7_EN",
	  PWRGD_VDDQC_HBM1_HBM3_HBM5_HBM7_R }, //FM_VDDQC_HBM1_HBM3_HBM5_HBM7_EN
	{ 1, VR_2_EN, 7, "FM_VDDQL_HBM0_HBM2_HBM4_HBM6_EN",
	  PWRGD_VDDQL_HBM0_HBM2_HBM4_HBM6_R }, //FM_VDDQL_HBM0_HBM2_HBM4_HBM6_EN
	{ 1, VR_2_EN, 3, "FM_VDDQL_HBM1_HBM3_HBM5_HBM7_EN",
	  PWRGD_VDDQL_HBM1_HBM3_HBM5_HBM7_R }, //FM_VDDQL_HBM1_HBM3_HBM5_HBM7_EN
	{ 1, VR_AND_CLK_EN, 1, "FM_HAMSA_AVDD_PCIE_EN",
	  PWRGD_HAMSA_AVDD_PCIE_R }, //FM_HAMSA_AVDD_PCIE_EN
	{ 1, VR_AND_CLK_EN, 5, "FM_OWL_E_TRVDD0P9_EN",
	  PWRGD_OWL_E_TRVDD0P9_R }, //FM_OWL_E_TRVDD0P9_EN
	{ 1, VR_AND_CLK_EN, 4, "FM_OWL_W_TRVDD0P9_EN",
	  PWRGD_OWL_W_TRVDD0P9_R }, //FM_OWL_W_TRVDD0P9_EN
	{ 1, VR_3_EN, 3, "FM_PVDD0P9_EN",
	  PWRGD_P0V9_OWL_E_PVDD }, //FM_PVDD0P9_EN (OWL_E_PVDD0P9, OWL_W_PVDD0P9)
	{ 1, VR_3_EN, 1, "FM_P1V5_RVDD_EN",
	  PWRGD_P1V5_E_RVDD }, //FM_P1V5_RVDD_EN (OWL_E_RVDD1P5, OWL_W_RVDD1P5)
	{ 1, VR_AND_CLK_EN, 0, "FM_HAMSA_VDDHRXTX_PCIE_EN",
	  PWRGD_HAMSA_VDDHRXTX_PCIE_R }, //FM_HAMSA_VDDHRXTX_PCIE_EN
	{ 1, VR_3_EN, 2, "FM_PVDD1P5_EN", PWRGD_PVDD1P5 }, //FM_PVDD1P5_EN
	{ 1, VR_CLK_ENABLE_PIN_CTRL_REG, 6, "ASIC_POWER_ON_RESET_N",
	  NO_DEFINED }, //ASIC_POWER_ON_RESET_N
	{ 1, VR_CLK_ENABLE_PIN_CTRL_REG, 5, "ASIC_SYS_RST_N", NO_DEFINED }, //ASIC_SYS_RST_N
	{ 1, VR_CLK_ENABLE_PIN_CTRL_REG, 4, "HAMSA_PCIE_PERST_B_PLD_N",
	  NO_DEFINED }, //HAMSA_PCIE_PERST_B_PLD_N
};

#define MAX_STEPS (sizeof(steps_on) / sizeof(steps_on[0]))
int power_steps = 0;
static bool iris_power_control(uint8_t onoff)
{
	uint8_t tmp = onoff ? 0x80 : 0x00;
	return plat_write_cpld(CPLD_OFFSET_MMC_PWR_EN, &tmp);
}
void cmd_iris_power_on(const struct shell *shell, size_t argc, char **argv)
{
	if (!iris_power_control(1))
		shell_warn(shell, "iris power on set cpld fail!");
	// wait 1s
	k_msleep(1000);
	if (gpio_get(RST_IRIS_PWR_ON_PLD_R1_N) == GPIO_HIGH) {
		shell_print(shell, "iris power on success!");
		set_pwr_steps_on_flag(0);
	} else {
		shell_warn(shell, "iris power on fail!");
	}
}
void cmd_iris_power_off(const struct shell *shell, size_t argc, char **argv)
{
	if (!iris_power_control(0))
		shell_warn(shell, "iris power off set cpld fail!");
	// wait 1s
	k_msleep(1000);
	if (gpio_get(FM_PLD_UBC_EN_R) == GPIO_LOW) {
		shell_print(shell, "iris power off success!");
	} else {
		shell_warn(shell, "iris power off fail!");
	}

	power_steps = 0;
	// init power steps
	set_pwr_steps_on_flag(0);
}

void cmd_iris_power_cycle(const struct shell *shell, size_t argc, char **argv)
{
	if (!iris_power_control(0))
		shell_warn(shell, "iris power cycle(off) fail!");
	k_msleep(5000);
	if (!iris_power_control(1))
		shell_warn(shell, "iris power cycle(on) fail!");
	set_pwr_steps_on_flag(0);
}

void cmd_iris_steps_on(const struct shell *shell, size_t argc, char **argv)
{
	if (power_steps >= MAX_STEPS) {
		LOG_WRN("no more steps");
		return;
	}

	// need to set VR_1STEP_FUNC_EN_REG to 0
	if (power_steps == 0) {
		if (gpio_get(RST_IRIS_PWR_ON_PLD_R1_N) == GPIO_HIGH) {
			shell_warn(shell, "iris power is already on, skip steps on operation.");
			set_pwr_steps_on_flag(0);
			return;
		}
		set_cpld_bit(VR_1STEP_FUNC_EN_REG, 0, 1);
	}

	// set pwr_steps_on_flag to void send event log to BMC
	set_pwr_steps_on_flag(1);
	// turn on the steps on bit
	if (set_cpld_bit(steps_on[power_steps].cpld_offset, steps_on[power_steps].bit,
			 steps_on[power_steps].power_on_value) == false) {
		shell_print(shell, "set %-35s fail", steps_on[power_steps].name);
	} else {
		shell_print(shell, "set %-35s success", steps_on[power_steps].name);
	}

	uint8_t pwrgd_idx = steps_on[power_steps].pwrgd_idx;
	if (pwrgd_idx == NO_DEFINED) {
		power_steps += 1;
		return;
	}
	//delay 1s
	k_msleep(1000);
	uint8_t offset = power_good_status_table_for_steps_on[pwrgd_idx].cpld_offsets;
	uint8_t bit = power_good_status_table_for_steps_on[pwrgd_idx].bit_loc;
	uint8_t reg_data = 0;
	uint8_t value = 0;
	if (pwrgd_idx == P12V_UBC1_PWRGD) {
		offset = power_good_status_table_for_steps_on[P12V_UBC1_PWRGD].cpld_offsets;
		bit = power_good_status_table_for_steps_on[P12V_UBC1_PWRGD].bit_loc;
		if (!plat_read_cpld(offset, &reg_data, 1))
			shell_error(shell, "Read CPLD offset 0x%x failed", offset);
		value = (reg_data >> bit) & 0x01;
		shell_print(shell, "%-20s %d",
			    power_good_status_table_for_steps_on[P12V_UBC2_PWRGD].power_rail_name,
			    value);
		offset = power_good_status_table_for_steps_on[P12V_UBC2_PWRGD].cpld_offsets;
		bit = power_good_status_table_for_steps_on[P12V_UBC2_PWRGD].bit_loc;
		if (!plat_read_cpld(offset, &reg_data, 1))
			shell_error(shell, "Read CPLD offset 0x%x failed", offset);
		value = (reg_data >> bit) & 0x01;
		shell_print(shell, "%-20s %d",
			    power_good_status_table_for_steps_on[P12V_UBC2_PWRGD].power_rail_name,
			    value);
	} else if (pwrgd_idx == PWRGD_P1V5_PLL_VDDA_OWL_E) {
		offset =
			power_good_status_table_for_steps_on[PWRGD_P1V5_PLL_VDDA_OWL_E].cpld_offsets;
		bit = power_good_status_table_for_steps_on[PWRGD_P1V5_PLL_VDDA_OWL_E].bit_loc;
		if (!plat_read_cpld(offset, &reg_data, 1))
			shell_error(shell, "Read CPLD offset 0x%x failed", offset);
		value = (reg_data >> bit) & 0x01;
		shell_print(shell, "%-20s %d",
			    power_good_status_table_for_steps_on[PWRGD_P1V5_PLL_VDDA_OWL_E]
				    .power_rail_name,
			    value);
		offset =
			power_good_status_table_for_steps_on[PWRGD_P1V5_PLL_VDDA_OWL_W].cpld_offsets;
		bit = power_good_status_table_for_steps_on[PWRGD_P1V5_PLL_VDDA_OWL_W].bit_loc;
		if (!plat_read_cpld(offset, &reg_data, 1))
			shell_error(shell, "Read CPLD offset 0x%x failed", offset);
		value = (reg_data >> bit) & 0x01;
		shell_print(shell, "%-20s %d",
			    power_good_status_table_for_steps_on[PWRGD_P1V5_PLL_VDDA_OWL_W]
				    .power_rail_name,
			    value);
	} else if (pwrgd_idx == PWRGD_P0V9_OWL_E_PVDD) {
		offset = power_good_status_table_for_steps_on[PWRGD_P0V9_OWL_E_PVDD].cpld_offsets;
		bit = power_good_status_table_for_steps_on[PWRGD_P0V9_OWL_E_PVDD].bit_loc;
		if (!plat_read_cpld(offset, &reg_data, 1))
			shell_error(shell, "Read CPLD offset 0x%x failed", offset);
		value = (reg_data >> bit) & 0x01;
		shell_print(
			shell, "%-20s %d",
			power_good_status_table_for_steps_on[PWRGD_P0V9_OWL_E_PVDD].power_rail_name,
			value);
		offset = power_good_status_table_for_steps_on[PWRGD_P0V9_OWL_W_PVDD].cpld_offsets;
		bit = power_good_status_table_for_steps_on[PWRGD_P0V9_OWL_W_PVDD].bit_loc;
		if (!plat_read_cpld(offset, &reg_data, 1))
			shell_error(shell, "Read CPLD offset 0x%x failed", offset);
		value = (reg_data >> bit) & 0x01;
		shell_print(
			shell, "%-20s %d",
			power_good_status_table_for_steps_on[PWRGD_P0V9_OWL_W_PVDD].power_rail_name,
			value);

	} else if (pwrgd_idx == PWRGD_P1V5_E_RVDD) {
		offset = power_good_status_table_for_steps_on[PWRGD_P1V5_E_RVDD].cpld_offsets;
		bit = power_good_status_table_for_steps_on[PWRGD_P1V5_E_RVDD].bit_loc;
		if (!plat_read_cpld(offset, &reg_data, 1))
			shell_error(shell, "Read CPLD offset 0x%x failed", offset);
		value = (reg_data >> bit) & 0x01;
		shell_print(shell, "%-20s %d",
			    power_good_status_table_for_steps_on[PWRGD_P1V5_E_RVDD].power_rail_name,
			    value);
		offset = power_good_status_table_for_steps_on[PWRGD_P1V5_W_RVDD].cpld_offsets;
		bit = power_good_status_table_for_steps_on[PWRGD_P1V5_W_RVDD].bit_loc;
		if (!plat_read_cpld(offset, &reg_data, 1))
			shell_error(shell, "Read CPLD offset 0x%x failed", offset);
		value = (reg_data >> bit) & 0x01;
		shell_print(shell, "%-20s %d",
			    power_good_status_table_for_steps_on[PWRGD_P1V5_W_RVDD].power_rail_name,
			    value);
	} else {
		//read from CPLD
		if (!plat_read_cpld(offset, &reg_data, 1))
			shell_error(shell, "Read CPLD offset 0x%x failed", offset);
		value = (reg_data >> bit) & 0x01;
		shell_print(shell, "%-20s %d",
			    power_good_status_table_for_steps_on[pwrgd_idx].power_rail_name, value);
	}

	power_steps += 1;
}

void cmd_iris_disable_steps_on(const struct shell *shell, size_t argc, char **argv)
{
	// init power steps
	power_steps = 0;
	set_pwr_steps_on_flag(0);
	// init value is reverse of power on value
	uint8_t power_init_value = 0;
	for (int i = 0; i < MAX_STEPS; i++) {
		power_init_value = steps_on[i].power_on_value ^ 1;
		// set all steps on value to init value
		if (set_cpld_bit(steps_on[i].cpld_offset, steps_on[i].bit, power_init_value) ==
		    false) {
			shell_print(shell, "set %-35s fail", steps_on[i].name);
		} else {
			shell_print(shell, "set %-35s success", steps_on[i].name);
		}
	}
	// need to disable VR_1STEP_FUNC_EN to turn off UBC
	set_cpld_bit(VR_1STEP_FUNC_EN_REG, 0, 0);
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_iris_power_cmd,
			       SHELL_CMD(on, NULL, "iris power on", cmd_iris_power_on),
			       SHELL_CMD(off, NULL, "iris power off", cmd_iris_power_off),
			       SHELL_CMD(cycle, NULL, "iris power cycle", cmd_iris_power_cycle),
			       SHELL_CMD(steps_on, NULL, "iris power steps_on", cmd_iris_steps_on),
			       SHELL_CMD(disable_steps_on, NULL, "iris power disable steps_on",
					 cmd_iris_disable_steps_on),
			       SHELL_SUBCMD_SET_END);

/* Root of command echo */
SHELL_CMD_REGISTER(iris_power, &sub_iris_power_cmd, "iris power commands", NULL);
