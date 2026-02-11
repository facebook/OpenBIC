#include <stdlib.h>
#include <shell/shell.h>
#include "plat_cpld.h"
#include <logging/log.h>
// #include <shell_plat_power_sequence.h>
#include "plat_gpio.h"
#include "plat_isr.h"
#include "plat_i2c.h"
// #include "plat_ioexp.h"
#include "plat_pldm_sensor.h"
#include "plat_class.h"
#include "shell_arke_power.h"
#include "plat_gpio.h"
// arke power command

// #define enable 0x01
// #define disable 0x00
// #define NO_DEFINED 0xFF

// LOG_MODULE_REGISTER(shell_arke_power);
// typedef struct power_good_status {
// 	uint8_t index;
// 	uint8_t bit_loc;
// 	uint8_t cpld_offsets;
// 	uint8_t *power_rail_name;

// } power_good_status;
// enum power_good_status_type_for_steps_on {
// 	//VR Power Good pin reading(0x07)
// 	MODULE_PWRGD,
// 	PWRGD_P1V8_AUX,
// 	PWRGD_OWL_E_TRVDD0P9_R,
// 	PWRGD_OWL_W_TRVDD0P9_R,
// 	PWRGD_OWL_E_TRVDD0P75_R,
// 	PWRGD_OWL_W_TRVDD0P75_R,
// 	PWRGD_HAMSA_AVDD_PCIE_R,
// 	PWRGD_HAMSA_VDDHRXTX_PCIE_R,
// 	//VR Power Good pin reading(0x08)
// 	PWRGD_MEDHA1_VDD,
// 	PWRGD_MEDHA0_VDD,
// 	PWRGD_OWL_E_VDD_R,
// 	PWRGD_OWL_W_VDD_R,
// 	PWRGD_HAMSA_VDD_R,
// 	PWRGD_MAX_S_VDD_R,
// 	PWRGD_MAX_M_VDD_R,
// 	PWRGD_MAX_N_VDD_R,
// 	//VR Power Good pin reading(0x09)
// 	PWRGD_VDDQL_HBM0_HBM2_HBM4_HBM6_R,
// 	PWRGD_VDDQC_HBM0_HBM2_HBM4_HBM6_R,
// 	PWRGD_VPP_HBM0_HBM2_HBM4_HBM6_R,
// 	PWRGD_VDDPHY_HBM0_HBM2_HBM4_HBM6_R,
// 	PWRGD_VDDQL_HBM1_HBM3_HBM5_HBM7_R,
// 	PWRGD_VDDQC_HBM1_HBM3_HBM5_HBM7_R,
// 	PWRGD_VPP_HBM1_HBM3_HBM5_HBM7_R,
// 	PWRGD_VDDPHY_HBM1_HBM3_HBM5_HBM7_R,
// 	//VR Power Good pin reading(0x0A)
// 	PWRGD_PLL_VDDA15_HBM0_HBM2,
// 	PWRGD_PLL_VDDA15_HBM4_HBM6,
// 	PWRGD_PLL_VDDA15_HBM1_HBM3,
// 	PWRGD_PLL_VDDA15_HBM5_HBM7,
// 	PWRGD_P0V9_OWL_E_PVDD,
// 	PWRGD_P0V9_OWL_W_PVDD,
// 	PWRGD_P1V5_E_RVDD,
// 	PWRGD_P1V5_W_RVDD,
// 	//VR Power Good pin reading(0x0B)
// 	P12V_UBC1_PWRGD,
// 	P12V_UBC2_PWRGD,
// 	PWRGD_P5V_R,
// 	PWRGD_P3V3_R,
// 	PWRGD_P1V8_R,
// 	PWRGD_LDO_IN_1V2_R,
// 	PWRGD_P1V5_PLL_VDDA_OWL_E,
// 	PWRGD_P1V5_PLL_VDDA_SOC,
// 	//VR Power Good pin reading(0x0C)
// 	PWRGD_PVDD1P5,
// 	PWRGD_P0V75_AVDD_HCSL,
// 	PWRGD_P1V5_PLL_VDDA_OWL_W,
// 	PWRGD_P4V2,
// 	PWRGD_MAX
// };
// power_good_status power_good_status_table_for_steps_on[] = {
// 	// VR Power Good pin reading(0x07)
// 	{ MODULE_PWRGD, 7, VR_PWRGD_PIN_READING_1_REG, "MODULE_PWRGD" },
// 	{ PWRGD_P1V8_AUX, 6, VR_PWRGD_PIN_READING_1_REG, "PWRGD_P1V8_AUX" },
// 	{ PWRGD_OWL_E_TRVDD0P9_R, 5, VR_PWRGD_PIN_READING_1_REG, "PWRGD_OWL_E_TRVDD0P9_R" },
// 	{ PWRGD_OWL_W_TRVDD0P9_R, 4, VR_PWRGD_PIN_READING_1_REG, "PWRGD_OWL_W_TRVDD0P9_R" },
// 	{ PWRGD_OWL_E_TRVDD0P75_R, 3, VR_PWRGD_PIN_READING_1_REG, "PWRGD_OWL_E_TRVDD0P75_R" },
// 	{ PWRGD_OWL_W_TRVDD0P75_R, 2, VR_PWRGD_PIN_READING_1_REG, "PWRGD_OWL_W_TRVDD0P75_R" },
// 	{ PWRGD_HAMSA_AVDD_PCIE_R, 1, VR_PWRGD_PIN_READING_1_REG, "PWRGD_HAMSA_AVDD_PCIE_R" },
// 	{ PWRGD_HAMSA_VDDHRXTX_PCIE_R, 0, VR_PWRGD_PIN_READING_1_REG,
// 	  "PWRGD_HAMSA_VDDHRXTX_PCIE_R" },
// 	// VR Power Good pin reading(0x08)
// 	{ PWRGD_MEDHA1_VDD, 7, VR_PWRGD_PIN_READING_2_REG, "PWRGD_MEDHA1_VDD" },
// 	{ PWRGD_MEDHA0_VDD, 6, VR_PWRGD_PIN_READING_2_REG, "PWRGD_MEDHA0_VDD" },
// 	{ PWRGD_OWL_E_VDD_R, 5, VR_PWRGD_PIN_READING_2_REG, "PWRGD_OWL_E_VDD_R" },
// 	{ PWRGD_OWL_W_VDD_R, 4, VR_PWRGD_PIN_READING_2_REG, "PWRGD_OWL_W_VDD_R" },
// 	{ PWRGD_HAMSA_VDD_R, 3, VR_PWRGD_PIN_READING_2_REG, "PWRGD_HAMSA_VDD_R" },
// 	{ PWRGD_MAX_S_VDD_R, 2, VR_PWRGD_PIN_READING_2_REG, "PWRGD_MAX_S_VDD_R" },
// 	{ PWRGD_MAX_M_VDD_R, 1, VR_PWRGD_PIN_READING_2_REG, "PWRGD_MAX_M_VDD_R" },
// 	{ PWRGD_MAX_N_VDD_R, 0, VR_PWRGD_PIN_READING_2_REG, "PWRGD_MAX_N_VDD_R" },
// 	// VR Power Good pin reading(0x09)
// 	{ PWRGD_VDDQL_HBM0_HBM2_HBM4_HBM6_R, 7, VR_PWRGD_PIN_READING_3_REG,
// 	  "PWRGD_VDDQL_HBM0_HBM2_HBM4_HBM6_R" },
// 	{ PWRGD_VDDQC_HBM0_HBM2_HBM4_HBM6_R, 6, VR_PWRGD_PIN_READING_3_REG,
// 	  "PWRGD_VDDQC_HBM0_HBM2_HBM4_HBM6_R" },
// 	{ PWRGD_VPP_HBM0_HBM2_HBM4_HBM6_R, 5, VR_PWRGD_PIN_READING_3_REG,
// 	  "PWRGD_VPP_HBM0_HBM2_HBM4_HBM6_R" },
// 	{ PWRGD_VDDPHY_HBM0_HBM2_HBM4_HBM6_R, 4, VR_PWRGD_PIN_READING_3_REG,
// 	  "PWRGD_VDDPHY_HBM0_HBM2_HBM4_HBM6_R" },
// 	{ PWRGD_VDDQL_HBM1_HBM3_HBM5_HBM7_R, 3, VR_PWRGD_PIN_READING_3_REG,
// 	  "PWRGD_VDDQL_HBM1_HBM3_HBM5_HBM7_R" },
// 	{ PWRGD_VDDQC_HBM1_HBM3_HBM5_HBM7_R, 2, VR_PWRGD_PIN_READING_3_REG,
// 	  "PWRGD_VDDQC_HBM1_HBM3_HBM5_HBM7_R" },
// 	{ PWRGD_VPP_HBM1_HBM3_HBM5_HBM7_R, 1, VR_PWRGD_PIN_READING_3_REG,
// 	  "PWRGD_VPP_HBM1_HBM3_HBM5_HBM7_R" },
// 	{ PWRGD_VDDPHY_HBM1_HBM3_HBM5_HBM7_R, 0, VR_PWRGD_PIN_READING_3_REG,
// 	  "PWRGD_VDDPHY_HBM1_HBM3_HBM5_HBM7_R" },
// 	//VR Power Good pin reading(0x0A)
// 	{ PWRGD_PLL_VDDA15_HBM0_HBM2, 7, VR_PWRGD_PIN_READING_4_REG, "PWRGD_PLL_VDDA15_HBM0_HBM2" },
// 	{ PWRGD_PLL_VDDA15_HBM4_HBM6, 6, VR_PWRGD_PIN_READING_4_REG, "PWRGD_PLL_VDDA15_HBM4_HBM6" },
// 	{ PWRGD_PLL_VDDA15_HBM1_HBM3, 5, VR_PWRGD_PIN_READING_4_REG, "PWRGD_PLL_VDDA15_HBM1_HBM3" },
// 	{ PWRGD_PLL_VDDA15_HBM5_HBM7, 4, VR_PWRGD_PIN_READING_4_REG, "PWRGD_PLL_VDDA15_HBM5_HBM7" },
// 	{ PWRGD_P0V9_OWL_E_PVDD, 3, VR_PWRGD_PIN_READING_4_REG, "PWRGD_P0V9_OWL_E_PVDD" },
// 	{ PWRGD_P0V9_OWL_W_PVDD, 2, VR_PWRGD_PIN_READING_4_REG, "PWRGD_P0V9_OWL_W_PVDD" },
// 	{ PWRGD_P1V5_E_RVDD, 1, VR_PWRGD_PIN_READING_4_REG, "PWRGD_P1V5_E_RVDD" },
// 	{ PWRGD_P1V5_W_RVDD, 0, VR_PWRGD_PIN_READING_4_REG, "PWRGD_P1V5_W_RVDD" },
// 	//VR Power Good pin reading(0x0B)
// 	{ P12V_UBC1_PWRGD, 7, VR_PWRGD_PIN_READING_5_REG, "P12V_UBC1_PWRGD" },
// 	{ P12V_UBC2_PWRGD, 6, VR_PWRGD_PIN_READING_5_REG, "P12V_UBC2_PWRGD" },
// 	{ PWRGD_P5V_R, 5, VR_PWRGD_PIN_READING_5_REG, "PWRGD_P5V_R" },
// 	{ PWRGD_P3V3_R, 4, VR_PWRGD_PIN_READING_5_REG, "PWRGD_P3V3_R" },
// 	{ PWRGD_P1V8_R, 3, VR_PWRGD_PIN_READING_5_REG, "PWRGD_P1V8_R" },
// 	{ PWRGD_LDO_IN_1V2_R, 2, VR_PWRGD_PIN_READING_5_REG, "PWRGD_LDO_IN_1V2_R" },
// 	{ PWRGD_P1V5_PLL_VDDA_OWL_E, 1, VR_PWRGD_PIN_READING_5_REG, "PWRGD_P1V5_PLL_VDDA_OWL_E" },
// 	{ PWRGD_P1V5_PLL_VDDA_SOC, 0, VR_PWRGD_PIN_READING_5_REG, "PWRGD_P1V5_PLL_VDDA_SOC" },
// 	//VR Power Good pin reading(0x0C)
// 	{ PWRGD_PVDD1P5, 7, VR_PWRGD_PIN_READING_6_REG, "PWRGD_PVDD1P5" },
// 	{ PWRGD_P0V75_AVDD_HCSL, 6, VR_PWRGD_PIN_READING_6_REG, "PWRGD_P0V75_AVDD_HCSL" },
// 	{ PWRGD_P1V5_PLL_VDDA_OWL_W, 5, VR_PWRGD_PIN_READING_6_REG, "PWRGD_P1V5_PLL_VDDA_OWL_W" },
// 	{ PWRGD_P4V2, 4, VR_PWRGD_PIN_READING_6_REG, "PWRGD_P4V2" },
// };
// typedef struct steps_on_struct {
// 	uint8_t power_on_value;
// 	uint8_t cpld_offset;
// 	uint8_t bit;
// 	uint8_t *name;
// 	uint8_t pwrgd_idx;
// } steps_on_struct;

// static steps_on_struct steps_on[] = {
// 	// follow pwr sequence order
// 	{ 1, VR_AND_CLK_EN_PIN_CTRL, 1, "FM_PLD_UBC_EN", P12V_UBC1_PWRGD }, //p12V
// 	{ 1, VR_4_EN, 6, "FM_P3V3_EN", PWRGD_P3V3_R }, //FM_P3V3_EN
// 	{ 1, VR_AND_CLK_EN_PIN_CTRL, 3, "FM_P4V2_EN_R", PWRGD_P4V2 }, //FM_P4V2_EN_R
// 	{ 1, VR_4_EN, 7, "FM_P5V_EN", PWRGD_P5V_R }, //FM_P5V_EN
// 	{ 1, VR_4_EN, 5, "LDO_IN_1V2_EN_R", PWRGD_LDO_IN_1V2_R }, //LDO_IN_1V2_EN_R
// 	{ 1, VR_4_EN, 4, "FM_P1V80_EN", PWRGD_P1V8_R }, //FM_P1V80_EN
// 	{ 1, VR_3_EN, 0, "FM_AVDD_HCSL_EN", PWRGD_P0V75_AVDD_HCSL }, //FM_AVDD_HCSL_EN
// 	{ 1, VR_1_EN, 3, "FM_HAMSA_VDD_EN", PWRGD_HAMSA_VDD_R }, //FM_HAMSA_VDD_EN
// 	{ 1, VR_1_EN, 7, "MEDHA1_VDD_EN", PWRGD_MEDHA1_VDD }, //MEDHA1_VDD_EN
// 	{ 1, VR_1_EN, 6, "FM_MEDHA0_VDD_EN", PWRGD_MEDHA0_VDD }, //FM_MEDHA0_VDD_EN
// 	{ 1, VR_1_EN, 5, "FM_OWL_E_VDD_EN", PWRGD_OWL_E_VDD_R }, //FM_OWL_E_VDD_EN
// 	{ 1, VR_1_EN, 4, "FM_OWL_W_VDD_EN", PWRGD_OWL_W_VDD_R }, //FM_OWL_W_VDD_EN
// 	{ 1, VR_1_EN, 1, "FM_MAX_M_VDD_EN", PWRGD_MAX_M_VDD_R }, //FM_MAX_M_VDD_EN
// 	{ 1, VR_1_EN, 0, "FM_MAX_N_VDD_EN", PWRGD_MAX_N_VDD_R }, //FM_MAX_N_VDD_EN
// 	{ 1, VR_1_EN, 2, "FM_MAX_S_VDD_EN", PWRGD_MAX_S_VDD_R }, //FM_MAX_S_VDD_EN
// 	{ 1, VR_AND_CLK_EN, 3, "FM_OWL_E_TRVDD0P75_EN",
// 	  PWRGD_OWL_E_TRVDD0P75_R }, //FM_OWL_E_TRVDD0P75_EN
// 	{ 1, VR_AND_CLK_EN, 2, "FM_OWL_W_TRVDD0P75_EN",
// 	  PWRGD_OWL_W_TRVDD0P75_R }, //FM_OWL_W_TRVDD0P75_EN
// 	{ 1, VR_2_EN, 4, "FM_VDDPHY_HBM0_HBM2_HBM4_HBM6_EN",
// 	  PWRGD_VDDPHY_HBM0_HBM2_HBM4_HBM6_R }, //FM_VDDPHY_HBM0_HBM2_HBM4_HBM6_EN
// 	{ 1, VR_2_EN, 0, "FM_VDDPHY_HBM1_HBM3_HBM5_HBM7_EN",
// 	  PWRGD_VDDPHY_HBM1_HBM3_HBM5_HBM7_R }, //FM_VDDPHY_HBM1_HBM3_HBM5_HBM7_EN
// 	{ 1, VR_4_EN, 1, "FM_P1V5_PLL_VDDA_OWL_EN",
// 	  PWRGD_P1V5_PLL_VDDA_OWL_E }, //FM_P1V5_PLL_VDDA_OWL_EN
// 	{ 1, VR_4_EN, 0, "FM_P1V5_PLL_VDDA_SOC_EN",
// 	  PWRGD_P1V5_PLL_VDDA_SOC }, //FM_P1V5_PLL_VDDA_SOC_EN
// 	{ 1, VR_3_EN, 7, "FM_PLL_VDDA15_HBM0_HBM2_EN",
// 	  PWRGD_PLL_VDDA15_HBM0_HBM2 }, //FM_PLL_VDDA15_HBM0_HBM2_EN
// 	{ 1, VR_3_EN, 5, "FM_PLL_VDDA15_HBM1_HBM3_EN",
// 	  PWRGD_PLL_VDDA15_HBM1_HBM3 }, //FM_PLL_VDDA15_HBM1_HBM3_EN
// 	{ 1, VR_3_EN, 6, "FM_PLL_VDDA15_HBM4_HBM6_EN",
// 	  PWRGD_PLL_VDDA15_HBM4_HBM6 }, //FM_PLL_VDDA15_HBM4_HBM6_EN
// 	{ 1, VR_3_EN, 4, "FM_PLL_VDDA15_HBM5_HBM7_EN",
// 	  PWRGD_PLL_VDDA15_HBM5_HBM7 }, //FM_PLL_VDDA15_HBM5_HBM7_EN
// 	{ 1, VR_AND_CLK_EN_PIN_CTRL, 2, "FM_P3V3_CLK_EN", NO_DEFINED }, //FM_P3V3_CLK_EN
// 	{ 0, VR_AND_CLK_EN, 6, "FM_RAINBOW_CLK_100MHZ_EN_N",
// 	  NO_DEFINED }, //FM_RAINBOW_CLK_100MHZ_EN_N
// 	{ 1, VR_AND_CLK_EN, 7, "FM_RAINBOW_CLK_48MHZ_EN", NO_DEFINED }, //FM_RAINBOW_CLK_48MHZ_EN
// 	{ 0, VR_AND_CLK_EN_PIN_CTRL, 0, "FM_IRIS_CLK_312MHZ_EN_N",
// 	  NO_DEFINED }, //FM_IRIS_CLK_312MHZ_EN_N
// 	{ 1, VR_2_EN, 5, "FM_VPP_HBM0_HBM2_HBM4_HBM6_EN",
// 	  PWRGD_VPP_HBM0_HBM2_HBM4_HBM6_R }, //FM_VPP_HBM0_HBM2_HBM4_HBM6_EN
// 	{ 1, VR_2_EN, 1, "FM_VPP_HBM1_HBM3_HBM5_HBM7_EN",
// 	  PWRGD_VPP_HBM1_HBM3_HBM5_HBM7_R }, //FM_VPP_HBM1_HBM3_HBM5_HBM7_EN
// 	{ 1, VR_2_EN, 6, "FM_VDDQC_HBM0_HBM2_HBM4_HBM6_EN",
// 	  PWRGD_VDDQC_HBM0_HBM2_HBM4_HBM6_R }, //FM_VDDQC_HBM0_HBM2_HBM4_HBM6_EN
// 	{ 1, VR_2_EN, 2, "FM_VDDQC_HBM1_HBM3_HBM5_HBM7_EN",
// 	  PWRGD_VDDQC_HBM1_HBM3_HBM5_HBM7_R }, //FM_VDDQC_HBM1_HBM3_HBM5_HBM7_EN
// 	{ 1, VR_2_EN, 7, "FM_VDDQL_HBM0_HBM2_HBM4_HBM6_EN",
// 	  PWRGD_VDDQL_HBM0_HBM2_HBM4_HBM6_R }, //FM_VDDQL_HBM0_HBM2_HBM4_HBM6_EN
// 	{ 1, VR_2_EN, 3, "FM_VDDQL_HBM1_HBM3_HBM5_HBM7_EN",
// 	  PWRGD_VDDQL_HBM1_HBM3_HBM5_HBM7_R }, //FM_VDDQL_HBM1_HBM3_HBM5_HBM7_EN
// 	{ 1, VR_AND_CLK_EN, 1, "FM_HAMSA_AVDD_PCIE_EN",
// 	  PWRGD_HAMSA_AVDD_PCIE_R }, //FM_HAMSA_AVDD_PCIE_EN
// 	{ 1, VR_AND_CLK_EN, 5, "FM_OWL_E_TRVDD0P9_EN",
// 	  PWRGD_OWL_E_TRVDD0P9_R }, //FM_OWL_E_TRVDD0P9_EN
// 	{ 1, VR_AND_CLK_EN, 4, "FM_OWL_W_TRVDD0P9_EN",
// 	  PWRGD_OWL_W_TRVDD0P9_R }, //FM_OWL_W_TRVDD0P9_EN
// 	{ 1, VR_3_EN, 3, "FM_PVDD0P9_EN",
// 	  PWRGD_P0V9_OWL_E_PVDD }, //FM_PVDD0P9_EN (OWL_E_PVDD0P9, OWL_W_PVDD0P9)
// 	{ 1, VR_3_EN, 1, "FM_P1V5_RVDD_EN",
// 	  PWRGD_P1V5_E_RVDD }, //FM_P1V5_RVDD_EN (OWL_E_RVDD1P5, OWL_W_RVDD1P5)
// 	{ 1, VR_3_EN, 2, "FM_PVDD1P5_EN", PWRGD_PVDD1P5 }, //FM_PVDD1P5_EN
// 	{ 1, VR_AND_CLK_EN, 0, "FM_HAMSA_VDDHRXTX_PCIE_EN",
// 	  PWRGD_HAMSA_VDDHRXTX_PCIE_R }, //FM_HAMSA_VDDHRXTX_PCIE_EN
// 	{ 1, VR_CLK_ENABLE_PIN_CTRL_REG, 6, "ASIC_POWER_ON_RESET_N",
// 	  NO_DEFINED }, //ASIC_POWER_ON_RESET_N
// 	{ 1, VR_CLK_ENABLE_PIN_CTRL_REG, 5, "ASIC_SYS_RST_N", NO_DEFINED }, //ASIC_SYS_RST_N
// 	{ 1, VR_CLK_ENABLE_PIN_CTRL_REG, 4, "HAMSA_PCIE_PERST_B_PLD_N",
// 	  NO_DEFINED }, //HAMSA_PCIE_PERST_B_PLD_N
// };

pwr_clock_compnt_mapping pwr_clock_compnt_mapping_table[] = {
	{ CLK_BUF_100M_U85, CLK_BUF_U85_ADDR, I2C_BUS1, "LOS_EVT_CLK_BUF_100M_U85" },
	{ CLK_BUF_100M_U690, CLK_BUF_U690_ADDR, I2C_BUS1, "LOS_EVT_CLK_BUF_100M_U690" },
	{ CLK_BUF_100M_U88, CLK_BUF_U88_ADDR, I2C_BUS3, "LOS_EVT_CLK_BUF_100M_U88" },
	{ CLK_GEN_100M_U86, CLK_GEN_100M_U86_ADDR, I2C_BUS3, "LOS_EVT_CLK_GEN_100M_U86" },
};

void clear_clock_status(const struct shell *shell, uint8_t clock_index)
{
	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;
	switch (clock_index) {
	case CLK_BUF_100M_U85:
	case CLK_BUF_100M_U690:
	case CLK_BUF_100M_U88:
		i2c_msg.bus = pwr_clock_compnt_mapping_table[clock_index].bus;
		i2c_msg.target_addr = pwr_clock_compnt_mapping_table[clock_index].addr;
		i2c_msg.tx_len = 1;
		i2c_msg.rx_len = 2;
		i2c_msg.data[0] = CLK_BUF_100M_WRITE_LOCK_CLEAR_LOS_EVENT_OFFSET;

		if (i2c_master_read(&i2c_msg, retry)) {
			if (!shell) {
				return;
			}
			shell_error(shell, "Failed to read reg, bus: %d, addr: 0x%x, reg: 0x%x",
				    pwr_clock_compnt_mapping_table[clock_index].bus,
				    pwr_clock_compnt_mapping_table[clock_index].addr,
				    CLK_BUF_100M_WRITE_LOCK_CLEAR_LOS_EVENT_OFFSET);
			return;
		}

		if (i2c_msg.data[1] & BIT(1)) {
			i2c_msg.tx_len = 3;
			i2c_msg.rx_len = 0;
			i2c_msg.data[0] = CLK_BUF_100M_WRITE_LOCK_CLEAR_LOS_EVENT_OFFSET;
			i2c_msg.data[1] = 1;
			i2c_msg.data[2] = 0x2;
			if (i2c_master_write(&i2c_msg, retry)) {
				if (!shell) {
					return;
				}
				shell_error(shell,
					    "Failed to write reg, bus: %d, addr: 0x%x, reg: 0x%x",
					    pwr_clock_compnt_mapping_table[clock_index].bus,
					    pwr_clock_compnt_mapping_table[clock_index].addr,
					    CLK_BUF_100M_WRITE_LOCK_CLEAR_LOS_EVENT_OFFSET);
				return;
			}
		}
		break;
	case CLK_GEN_100M_U86:
		i2c_msg.bus = pwr_clock_compnt_mapping_table[clock_index].bus;
		i2c_msg.target_addr = pwr_clock_compnt_mapping_table[clock_index].addr;
		i2c_msg.tx_len = 1;
		i2c_msg.rx_len = 2;
		i2c_msg.data[0] = CLK_GEN_LOSMON_EVENT_OFFSET;

		if (i2c_master_read(&i2c_msg, retry)) {
			if (!shell) {
				return;
			}
			shell_error(shell, "Failed to read reg, bus: %d, addr: 0x%x, reg: 0x%x",
				    pwr_clock_compnt_mapping_table[clock_index].bus,
				    pwr_clock_compnt_mapping_table[clock_index].addr,
				    CLK_GEN_LOSMON_EVENT_OFFSET);
			return;
		}

		if (i2c_msg.data[1] & BIT(1)) {
			i2c_msg.tx_len = 3;
			i2c_msg.rx_len = 0;
			i2c_msg.data[0] = CLK_GEN_LOSMON_EVENT_OFFSET;
			i2c_msg.data[1] = 1;
			i2c_msg.data[2] = 0x2;
			if (i2c_master_write(&i2c_msg, retry)) {
				if (!shell) {
					return;
				}
				shell_error(shell,
					    "Failed to write reg, bus: %d, addr: 0x%x, reg: 0x%x",
					    pwr_clock_compnt_mapping_table[clock_index].bus,
					    pwr_clock_compnt_mapping_table[clock_index].addr,
					    CLK_GEN_LOSMON_EVENT_OFFSET);
				return;
			}
		}
		break;
	default:
		if (!shell) {
			return;
		}
		shell_error(shell, "Type wrong clock");
	}

	return;
}

// typedef struct ioe_power_good_status {
// 	uint8_t bus;
// 	uint8_t addr;
// 	uint8_t bit_loc;
// 	uint8_t *ioe_pwrgd_name;

// } ioe_power_good_status;

// ioe_power_good_status ioe_pwrgd_status_table[] = {
// 	{ U200052_IO_I2C_BUS, U200052_IO_ADDR, 0, "PWRGD_P3V3_OSFP_P1" },
// 	{ U200052_IO_I2C_BUS, U200052_IO_ADDR, 1, "PWRGD_P3V3_OSFP_P2" },
// 	{ U200052_IO_I2C_BUS, U200052_IO_ADDR, 2, "PWRGD_P3V3_OSFP_P3" },
// 	{ U200052_IO_I2C_BUS, U200052_IO_ADDR, 3, "PWRGD_P3V3_OSFP_P4" },
// 	{ U200052_IO_I2C_BUS, U200052_IO_ADDR, 4, "PWRGD_P3V3_OSFP_P5" },
// 	{ U200052_IO_I2C_BUS, U200052_IO_ADDR, 5, "PWRGD_P3V3_OSFP_P6" },
// };

// typedef struct ioe_pwr_on {
// 	uint8_t bus;
// 	uint8_t addr;
// 	uint8_t bit_loc;
// 	uint8_t *ioe_enable_name;

// } ioe_pwr_on;

// ioe_pwr_on ioe_pwr_on_table[] = {
// 	{ U200051_IO_I2C_BUS, U200051_IO_ADDR, 0, "FM_P3V3_OSFP_P1_EN" },
// 	{ U200051_IO_I2C_BUS, U200051_IO_ADDR, 1, "FM_P3V3_OSFP_P2_EN" },
// 	{ U200051_IO_I2C_BUS, U200051_IO_ADDR, 2, "FM_P3V3_OSFP_P3_EN" },
// 	{ U200051_IO_I2C_BUS, U200051_IO_ADDR, 3, "FM_P3V3_OSFP_P4_EN" },
// 	{ U200051_IO_I2C_BUS, U200051_IO_ADDR, 4, "FM_P3V3_OSFP_P5_EN" },
// 	{ U200051_IO_I2C_BUS, U200051_IO_ADDR, 5, "FM_P3V3_OSFP_P6_EN" },
// };

// bool check_p3v3_p5v_pwrgd(void)
// {
// 	// read p3v3_pwrgf and p5v_pwrgf
// 	// PWRGD_P3V3_R, bit-4, VR_PWRGD_PIN_READING_5_REG
// 	uint8_t offset = VR_PWRGD_PIN_READING_5_REG;
// 	uint8_t reg_data = 0;
// 	if (!plat_read_cpld(offset, &reg_data, 1)) {
// 		LOG_ERR("Read CPLD offset 0x%x failed", offset);
// 	}
// 	uint8_t p3v3_value = (reg_data >> 4) & 0x01;
// 	// PWRGD_P5V_R, bit-5, VR_PWRGD_PIN_READING_5_REG
// 	uint8_t p5v_value = (reg_data >> 5) & 0x01;
// 	//if both p3v3 and p5v are all 1, return true
// 	if (p3v3_value == 1 && p5v_value == 1)
// 		return true;
// 	return false;
// }

// void power_on_p3v3_osfp()
// {
// 	uint8_t write_data = 0;
// 	uint8_t check_value = 0;
// 	k_msleep(1000);
// 	for (int i = 0; i < sizeof(ioe_pwr_on_table) / sizeof(ioe_pwr_on_table[0]); i++) {
// 		// set
// 		write_data |= BIT(ioe_pwr_on_table[i].bit_loc);
// 		//print write data
// 		LOG_DBG("%s : %20d", ioe_pwr_on_table[i].ioe_enable_name, write_data);
// 		set_pca6554apw_ioe_value(ioe_pwr_on_table[i].bus, ioe_pwr_on_table[i].addr,
// 					 OUTPUT_PORT, write_data);

// 		k_msleep(100);
// 		//check set value
// 		get_pca6554apw_ioe_value(ioe_pwr_on_table[i].bus, ioe_pwr_on_table[i].addr,
// 					 OUTPUT_PORT, &check_value);
// 		uint8_t temp_value = (check_value >> i) & 0x01;
// 		LOG_DBG("check_value : %d", check_value);
// 		if (!temp_value)
// 			LOG_INF("%s : %20s", ioe_pwr_on_table[i].ioe_enable_name, "fail");
// 	}
// }
// void power_off_p3v3_osfp(const struct shell *shell)
// {
// 	set_pca6554apw_ioe_value(ioe_pwr_on_table[0].bus, ioe_pwr_on_table[0].addr, OUTPUT_PORT,
// 				 0x0);
// 	// check is power off
// 	uint8_t check_value = 0;
// 	get_pca6554apw_ioe_value(ioe_pwr_on_table[0].bus, ioe_pwr_on_table[0].addr, OUTPUT_PORT,
// 				 &check_value);
// 	if (check_value) {
// 		shell_warn(shell, "%s ", "ioeU200052 power off fail");
// 	}
// }
// void pwer_gd_get_status(const struct shell *shell)
// {
// 	uint8_t check_value = 0;
// 	int ret = 0;
// 	ret = get_pca6554apw_ioe_value(ioe_pwrgd_status_table[0].bus,
// 				       ioe_pwrgd_status_table[0].addr, INPUT_PORT, &check_value);

// 	if (ret == -1) {
// 		return;
// 	}

// 	for (int i = 0; i < sizeof(ioe_pwrgd_status_table) / sizeof(ioe_pwrgd_status_table[0]);
// 	     i++) {
// 		uint8_t tmp_value = (check_value >> i) & 0x01;
// 		if (tmp_value) {
// 			shell_print(shell, "%s : %d", ioe_pwrgd_status_table[i].ioe_pwrgd_name,
// 				    tmp_value);
// 		} else {
// 			shell_print(shell, "%s : %d", ioe_pwrgd_status_table[i].ioe_pwrgd_name,
// 				    tmp_value);
// 		}
// 	}
// }
// void steps_on_p3v3_osfp(const struct shell *shell)
// {
// 	uint8_t reg = 0;
// 	uint8_t pwrgd_status = 0;
// 	uint8_t check_value = 0;
// 	for (int i = 0; i < sizeof(ioe_pwr_on_table) / sizeof(ioe_pwr_on_table[0]); i++) {
// 		reg |= BIT(ioe_pwr_on_table[i].bit_loc);

// 		set_pca6554apw_ioe_value(ioe_pwr_on_table[i].bus, ioe_pwr_on_table[i].addr,
// 					 OUTPUT_PORT, reg);

// 		k_msleep(100);
// 		//check set value
// 		get_pca6554apw_ioe_value(ioe_pwr_on_table[i].bus, ioe_pwr_on_table[i].addr,
// 					 OUTPUT_PORT, &check_value);
// 		// only get the last bit
// 		uint8_t temp_value = (check_value >> i) & 0x01;

// 		if (temp_value) {
// 			shell_print(shell, "%s %s %20s", "set", ioe_pwr_on_table[i].ioe_enable_name,
// 				    "success");
// 		} else {
// 			shell_print(shell, "%s %s %20s", "set", ioe_pwr_on_table[i].ioe_enable_name,
// 				    "fail");
// 		}
// 		//check pwrgd status
// 		get_pca6554apw_ioe_value(ioe_pwrgd_status_table[i].bus,
// 					 ioe_pwrgd_status_table[i].addr, INPUT_PORT, &pwrgd_status);
// 		pwrgd_status = (pwrgd_status >> ioe_pwrgd_status_table[i].bit_loc) & 0x01;
// 		//print name and value and status
// 		if (pwrgd_status) {
// 			shell_print(shell, "%s : %d %20s", ioe_pwrgd_status_table[i].ioe_pwrgd_name,
// 				    pwrgd_status, "success");
// 		} else {
// 			shell_print(shell, "%s : %d %20s", ioe_pwrgd_status_table[i].ioe_pwrgd_name,
// 				    pwrgd_status, "fail");
// 		}
// 		k_msleep(100);
// 	}
// }

void pwr_get_clock_status(const struct shell *shell, uint8_t clock_index)
{
	uint8_t addr = 0;
	uint8_t bus = 0;
	uint8_t *temp_clock_name = "";

	for (int i = 0;
	     i < sizeof(pwr_clock_compnt_mapping_table) / sizeof(pwr_clock_compnt_mapping_table[0]);
	     i++) {
		if (pwr_clock_compnt_mapping_table[i].clock_name_index == clock_index) {
			addr = pwr_clock_compnt_mapping_table[i].addr;
			bus = pwr_clock_compnt_mapping_table[i].bus;
			temp_clock_name = pwr_clock_compnt_mapping_table[i].clock_name;
			break;
		}
	}
	//shell_info(shell, "clock index: %d, addr: 0x%x, bus: %d", clock_index, addr, bus); //dbg info

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;
	switch (clock_index) {
	case CLK_BUF_100M_U85:
	case CLK_BUF_100M_U690:
	case CLK_BUF_100M_U88:
		i2c_msg.bus = bus;
		i2c_msg.target_addr = addr;
		i2c_msg.tx_len = 1;
		i2c_msg.rx_len = 2; //first byte is data length because it is block read
		i2c_msg.data[0] = CLK_BUF_100M_WRITE_LOCK_CLEAR_LOS_EVENT_OFFSET;
		if (i2c_master_read(&i2c_msg, retry)) {
			shell_error(shell, "Failed to read reg, bus: %d, addr: 0x%x, reg: 0x%x",
				    bus, addr, CLK_BUF_100M_WRITE_LOCK_CLEAR_LOS_EVENT_OFFSET);
			return;
		}
		//print clock name and value and status
		if (!i2c_msg.data[1]) {
			shell_print(shell, "%s : 0x%x %35s", temp_clock_name, i2c_msg.data[1],
				    "success");
		} else {
			shell_print(shell, "%s : 0x%x %35s", temp_clock_name, i2c_msg.data[1],
				    "fail");
		}

		break;
	case CLK_GEN_100M_U86:
		i2c_msg.bus = bus;
		i2c_msg.target_addr = addr;
		i2c_msg.tx_len = 1;
		i2c_msg.rx_len = 2; //first byte is data length because it is block read
		i2c_msg.data[0] = CLK_GEN_LOSMON_EVENT_OFFSET;
		if (i2c_master_read(&i2c_msg, retry)) {
			shell_error(shell, "Failed to read reg, bus: %d, addr: 0x%x, reg: 0x%x",
				    bus, addr, CLK_GEN_LOSMON_EVENT_OFFSET);
			return;
		}
		if (!i2c_msg.data[1]) {
			shell_print(shell, "%s : 0x%x %35s", temp_clock_name, i2c_msg.data[1],
				    "success");
		} else {
			shell_print(shell, "%s : 0x%x %35s", temp_clock_name, i2c_msg.data[1],
				    "fail");
		}
		break;
	default:
		shell_error(shell, "Type wrong clock index: %d", clock_index);
	}
}

// #define MAX_STEPS (sizeof(steps_on) / sizeof(steps_on[0]))
// int power_steps = 0;

static bool arke_power_control(uint8_t onoff)
{
	uint8_t tmp = onoff ? 0x80 : 0x00;
	return plat_write_cpld(0x38, &tmp);
}

void cmd_arke_power_on(const struct shell *shell, size_t argc, char **argv)
{
	if (!arke_power_control(1))
		shell_warn(shell, "arke power on set cpld fail!");
	// wait 1s
	k_msleep(1000);
	if (gpio_get(RST_ARKE_PWR_ON_PLD_R1_N) == GPIO_HIGH) {
		shell_print(shell, "arke power on success!");
		// set_pwr_steps_on_flag(0);
		// set_plat_sensor_one_step_enable_flag(false);
	} else {
		shell_warn(shell, "arke power on fail!");
	}
}
// void cmd_arke_power_off(const struct shell *shell, size_t argc, char **argv)
// {
// 	if (!arke_power_control(0))
// 		shell_warn(shell, "arke power off set cpld fail!");
// 	// wait 1s
// 	k_msleep(1000);
// 	if (gpio_get(FM_PLD_UBC_EN_R) == GPIO_LOW) {
// 		shell_print(shell, "arke power off success!");
// 	} else {
// 		shell_warn(shell, "arke power off fail!");
// 	}

// 	power_steps = 0;
// 	// init power steps
// 	set_pwr_steps_on_flag(0);
// 	set_plat_sensor_one_step_enable_flag(false);
// }

// void cmd_arke_power_cycle(const struct shell *shell, size_t argc, char **argv)
// {
// 	if (!arke_power_control(0))
// 		shell_warn(shell, "arke power cycle(off) fail!");
// 	k_msleep(5000);
// 	if (!arke_power_control(1))
// 		shell_warn(shell, "arke power cycle(on) fail!");
// 	set_pwr_steps_on_flag(0);
// 	set_plat_sensor_one_step_enable_flag(false);
// }

// void cmd_arke_steps_on(const struct shell *shell, size_t argc, char **argv)
// {
// 	if (power_steps >= MAX_STEPS) {
// 		LOG_WRN("no more steps");
// 		return;
// 	}

// 	// need to set VR_1STEP_FUNC_EN_REG to 0
// 	if (power_steps == 0) {
// 		if (gpio_get(RST_IRIS_PWR_ON_PLD_R1_N) == GPIO_HIGH) {
// 			shell_warn(shell, "arke power is already on, skip steps on operation.");
// 			set_pwr_steps_on_flag(0);
// 			return;
// 		}
// 		set_pwr_steps_on_flag(1);
// 		set_cpld_bit(VR_1STEP_FUNC_EN_REG, 0, 1);
// 	}

// 	// set pwr_steps_on_flag to void send event log to BMC
// 	set_pwr_steps_on_flag(1);
// 	// turn on the steps on bit
// 	if (set_cpld_bit(steps_on[power_steps].cpld_offset, steps_on[power_steps].bit,
// 			 steps_on[power_steps].power_on_value) == false) {
// 		shell_print(shell, "set %-35s fail", steps_on[power_steps].name);
// 	} else {
// 		shell_print(shell, "set %-35s success", steps_on[power_steps].name);
// 	}

// 	uint8_t pwrgd_idx = steps_on[power_steps].pwrgd_idx;
// 	if (pwrgd_idx == NO_DEFINED) {
// 		// if steps on name == FM_P3V3_CLK_EN, do next steps immediately
// 		if (strcmp(steps_on[power_steps].name, "FM_P3V3_CLK_EN") == 0) {
// 			power_steps += 1;
// 			// call self function
// 			// turn on the steps on bit
// 			if (set_cpld_bit(steps_on[power_steps].cpld_offset,
// 					 steps_on[power_steps].bit,
// 					 steps_on[power_steps].power_on_value) == false) {
// 				shell_print(shell, "set %-35s fail", steps_on[power_steps].name);
// 			} else {
// 				shell_print(shell, "set %-35s success", steps_on[power_steps].name);
// 			}
// 			k_msleep(1000);
// 			for (int i = 0; i < CLK_COMPONENT_MAX; i++) {
// 				k_msleep(100);
// 				clear_clock_status(shell, i);
// 				k_msleep(100);
// 				pwr_get_clock_status(shell, i);
// 			}
// 		}
// 		power_steps += 1;
// 		return;
// 	}
// 	//delay 1s
// 	k_msleep(1000);
// 	uint8_t offset = power_good_status_table_for_steps_on[pwrgd_idx].cpld_offsets;
// 	uint8_t bit = power_good_status_table_for_steps_on[pwrgd_idx].bit_loc;
// 	uint8_t reg_data = 0;
// 	uint8_t value = 0;
// 	if (pwrgd_idx == P12V_UBC1_PWRGD) {
// 		offset = power_good_status_table_for_steps_on[P12V_UBC1_PWRGD].cpld_offsets;
// 		bit = power_good_status_table_for_steps_on[P12V_UBC1_PWRGD].bit_loc;
// 		if (!plat_read_cpld(offset, &reg_data, 1))
// 			shell_error(shell, "Read CPLD offset 0x%x failed", offset);
// 		value = (reg_data >> bit) & 0x01;
// 		shell_print(shell, "%-20s %d",
// 			    power_good_status_table_for_steps_on[P12V_UBC2_PWRGD].power_rail_name,
// 			    value);
// 		offset = power_good_status_table_for_steps_on[P12V_UBC2_PWRGD].cpld_offsets;
// 		bit = power_good_status_table_for_steps_on[P12V_UBC2_PWRGD].bit_loc;
// 		if (!plat_read_cpld(offset, &reg_data, 1))
// 			shell_error(shell, "Read CPLD offset 0x%x failed", offset);
// 		value = (reg_data >> bit) & 0x01;
// 		shell_print(shell, "%-20s %d",
// 			    power_good_status_table_for_steps_on[P12V_UBC2_PWRGD].power_rail_name,
// 			    value);
// 	} else if (pwrgd_idx == PWRGD_P1V5_PLL_VDDA_OWL_E) {
// 		offset =
// 			power_good_status_table_for_steps_on[PWRGD_P1V5_PLL_VDDA_OWL_E].cpld_offsets;
// 		bit = power_good_status_table_for_steps_on[PWRGD_P1V5_PLL_VDDA_OWL_E].bit_loc;
// 		if (!plat_read_cpld(offset, &reg_data, 1))
// 			shell_error(shell, "Read CPLD offset 0x%x failed", offset);
// 		value = (reg_data >> bit) & 0x01;
// 		shell_print(shell, "%-20s %d",
// 			    power_good_status_table_for_steps_on[PWRGD_P1V5_PLL_VDDA_OWL_E]
// 				    .power_rail_name,
// 			    value);
// 		offset =
// 			power_good_status_table_for_steps_on[PWRGD_P1V5_PLL_VDDA_OWL_W].cpld_offsets;
// 		bit = power_good_status_table_for_steps_on[PWRGD_P1V5_PLL_VDDA_OWL_W].bit_loc;
// 		if (!plat_read_cpld(offset, &reg_data, 1))
// 			shell_error(shell, "Read CPLD offset 0x%x failed", offset);
// 		value = (reg_data >> bit) & 0x01;
// 		shell_print(shell, "%-20s %d",
// 			    power_good_status_table_for_steps_on[PWRGD_P1V5_PLL_VDDA_OWL_W]
// 				    .power_rail_name,
// 			    value);
// 	} else if (pwrgd_idx == PWRGD_P0V9_OWL_E_PVDD) {
// 		offset = power_good_status_table_for_steps_on[PWRGD_P0V9_OWL_E_PVDD].cpld_offsets;
// 		bit = power_good_status_table_for_steps_on[PWRGD_P0V9_OWL_E_PVDD].bit_loc;
// 		if (!plat_read_cpld(offset, &reg_data, 1))
// 			shell_error(shell, "Read CPLD offset 0x%x failed", offset);
// 		value = (reg_data >> bit) & 0x01;
// 		shell_print(
// 			shell, "%-20s %d",
// 			power_good_status_table_for_steps_on[PWRGD_P0V9_OWL_E_PVDD].power_rail_name,
// 			value);
// 		offset = power_good_status_table_for_steps_on[PWRGD_P0V9_OWL_W_PVDD].cpld_offsets;
// 		bit = power_good_status_table_for_steps_on[PWRGD_P0V9_OWL_W_PVDD].bit_loc;
// 		if (!plat_read_cpld(offset, &reg_data, 1))
// 			shell_error(shell, "Read CPLD offset 0x%x failed", offset);
// 		value = (reg_data >> bit) & 0x01;
// 		shell_print(
// 			shell, "%-20s %d",
// 			power_good_status_table_for_steps_on[PWRGD_P0V9_OWL_W_PVDD].power_rail_name,
// 			value);

// 	} else if (pwrgd_idx == PWRGD_P1V5_E_RVDD) {
// 		offset = power_good_status_table_for_steps_on[PWRGD_P1V5_E_RVDD].cpld_offsets;
// 		bit = power_good_status_table_for_steps_on[PWRGD_P1V5_E_RVDD].bit_loc;
// 		if (!plat_read_cpld(offset, &reg_data, 1))
// 			shell_error(shell, "Read CPLD offset 0x%x failed", offset);
// 		value = (reg_data >> bit) & 0x01;
// 		shell_print(shell, "%-20s %d",
// 			    power_good_status_table_for_steps_on[PWRGD_P1V5_E_RVDD].power_rail_name,
// 			    value);
// 		offset = power_good_status_table_for_steps_on[PWRGD_P1V5_W_RVDD].cpld_offsets;
// 		bit = power_good_status_table_for_steps_on[PWRGD_P1V5_W_RVDD].bit_loc;
// 		if (!plat_read_cpld(offset, &reg_data, 1))
// 			shell_error(shell, "Read CPLD offset 0x%x failed", offset);
// 		value = (reg_data >> bit) & 0x01;
// 		shell_print(shell, "%-20s %d",
// 			    power_good_status_table_for_steps_on[PWRGD_P1V5_W_RVDD].power_rail_name,
// 			    value);
// 	} else {
// 		//read from CPLD
// 		if (!plat_read_cpld(offset, &reg_data, 1))
// 			shell_error(shell, "Read CPLD offset 0x%x failed", offset);
// 		value = (reg_data >> bit) & 0x01;
// 		shell_print(shell, "%-20s %d",
// 			    power_good_status_table_for_steps_on[pwrgd_idx].power_rail_name, value);
// 	}

// 	if (strcmp(steps_on[power_steps].name, "FM_P5V_EN") == 0) {
// 		if (check_p3v3_p5v_pwrgd()) {
// 			set_plat_sensor_one_step_enable_flag(ONE_STEP_POWER_MAGIC_NUMBER);
// 		}
// 	}

// 	if (strcmp(steps_on[power_steps].name, "FM_P1V80_EN") == 0) {
// 		//if board id is evb, run steps_on_p3v3_osfp
// 		if (get_asic_board_id() == ASIC_BOARD_ID_EVB) {
// 			steps_on_p3v3_osfp(shell);
// 			power_steps += 1;
// 			return;
// 		}
// 	}
// 	power_steps += 1;
// }

// void cmd_arke_disable_steps_on(const struct shell *shell, size_t argc, char **argv)
// {
// 	// init power steps
// 	power_steps = 0;
// 	set_pwr_steps_on_flag(0);
// 	// init value is reverse of power on value
// 	uint8_t power_init_value = 0;
// 	set_plat_sensor_one_step_enable_flag(false);
// 	for (int i = 0; i < MAX_STEPS; i++) {
// 		power_init_value = steps_on[i].power_on_value ^ 1;
// 		// set all steps on value to init value
// 		if (set_cpld_bit(steps_on[i].cpld_offset, steps_on[i].bit, power_init_value) ==
// 		    false) {
// 			shell_print(shell, "set %-35s fail", steps_on[i].name);
// 		} else {
// 			shell_print(shell, "set %-35s success", steps_on[i].name);
// 		}
// 	}
// 	// need to disable VR_1STEP_FUNC_EN to turn off UBC
// 	set_cpld_bit(VR_1STEP_FUNC_EN_REG, 0, 0);
// }

SHELL_STATIC_SUBCMD_SET_CREATE(sub_arke_power_cmd,
			       SHELL_CMD(on, NULL, "arke power on", cmd_arke_power_on),
			    //    SHELL_CMD(off, NULL, "arke power off", cmd_arke_power_off),
			    //    SHELL_CMD(cycle, NULL, "arke power cycle", cmd_arke_power_cycle),
			    //    SHELL_CMD(steps_on, NULL, "arke power steps_on", cmd_arke_steps_on),
			    //    SHELL_CMD(disable_steps_on, NULL, "arke power disable steps_on",
				// 	 cmd_arke_disable_steps_on),
			       SHELL_SUBCMD_SET_END);

/* Root of command echo */
SHELL_CMD_REGISTER(arke_power, &sub_arke_power_cmd, "arke power commands", NULL);
