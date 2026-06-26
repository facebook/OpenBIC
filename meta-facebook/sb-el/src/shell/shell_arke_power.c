#include <stdlib.h>
#include <shell/shell.h>
#include "plat_cpld.h"
#include <logging/log.h>
// #include <shell_plat_power_sequence.h>
#include "plat_gpio.h"
#include "plat_isr.h"
#include "plat_i2c.h"
#include "plat_ioexp.h"
#include "plat_pldm_sensor.h"
#include "plat_class.h"
#include "shell_arke_power.h"
#include "plat_gpio.h"
// arke power command

#define enable 0x01
#define disable 0x00
#define NO_DEFINED 0xFF

LOG_MODULE_REGISTER(shell_arke_power);

// clang-format off

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
	{ PWRGD_NUWA1_VDD, 7, VR_PWRGD_PIN_READING_2_REG, "PWRGD_NUWA1_VDD" },
	{ PWRGD_NUWA0_VDD, 6, VR_PWRGD_PIN_READING_2_REG, "PWRGD_NUWA0_VDD" },
	{ PWRGD_OWL_E_VDD_R, 5, VR_PWRGD_PIN_READING_2_REG, "PWRGD_OWL_E_VDD_R" },
	{ PWRGD_OWL_W_VDD_R, 4, VR_PWRGD_PIN_READING_2_REG, "PWRGD_OWL_W_VDD_R" },
	{ PWRGD_HAMSA_VDD_R, 3, VR_PWRGD_PIN_READING_2_REG, "PWRGD_HAMSA_VDD_R" },
	{ PWRGD_MAX_S_VDD_R, 2, VR_PWRGD_PIN_READING_2_REG, "PWRGD_MAX_S_VDD_R" },
	{ PWRGD_MAX_M_VDD_R, 1, VR_PWRGD_PIN_READING_2_REG, "PWRGD_MAX_M_VDD_R" },
	{ PWRGD_MAX_N_VDD_R, 0, VR_PWRGD_PIN_READING_2_REG, "PWRGD_MAX_N_VDD_R" },
	// VR Power Good pin reading(0x09)
	{ PWRGD_VDDQL_HBM0_HBM2_HBM4_HBM6_R, 7, VR_PWRGD_PIN_READING_3_REG,
	  "PWRGD_VDDQL_HBM0_HBM2_HBM4_HBM6_R" },
	{ PWRGD_VDDC_HBM0_HBM2_HBM4_HBM6_R, 6, VR_PWRGD_PIN_READING_3_REG,
	  "PWRGD_VDDC_HBM0_HBM2_HBM4_HBM6_R" },
	{ PWRGD_VPP_HBM0_HBM2_HBM4_HBM6_R, 5, VR_PWRGD_PIN_READING_3_REG,
	  "PWRGD_VPP_HBM0_HBM2_HBM4_HBM6_R" },
	{ PWRGD_VDDPHY_HBM0_HBM2_HBM4_HBM6_R, 4, VR_PWRGD_PIN_READING_3_REG,
	  "PWRGD_VDDPHY_HBM0_HBM2_HBM4_HBM6_R" },
	{ PWRGD_VDDQL_HBM1_HBM3_HBM5_HBM7_R, 3, VR_PWRGD_PIN_READING_3_REG,
	  "PWRGD_VDDQL_HBM1_HBM3_HBM5_HBM7_R" },
	{ PWRGD_VDDC_HBM1_HBM3_HBM5_HBM7_R, 2, VR_PWRGD_PIN_READING_3_REG,
	  "PWRGD_VDDC_HBM1_HBM3_HBM5_HBM7_R" },
	{ PWRGD_VPP_HBM1_HBM3_HBM5_HBM7_R, 1, VR_PWRGD_PIN_READING_3_REG,
	  "PWRGD_VPP_HBM1_HBM3_HBM5_HBM7_R" },
	{ PWRGD_VDDPHY_HBM1_HBM3_HBM5_HBM7_R, 0, VR_PWRGD_PIN_READING_3_REG,
	  "PWRGD_VDDPHY_HBM1_HBM3_HBM5_HBM7_R" },
	//VR Power Good pin reading(0x0A)
	{ PWRGD_VDDQ_HBM0_HBM2_HBM4_HBM6_R, 7, VR_PWRGD_PIN_READING_4_REG, "PWRGD_VDDQ_HBM0_HBM2_HBM4_HBM6_R" },
	{ PWRGD_VDDQ_HBM1_HBM3_HBM5_HBM7_R, 6, VR_PWRGD_PIN_READING_4_REG, "PWRGD_VDDQ_HBM1_HBM3_HBM5_HBM7_R" },
	{ PWRGD_P1V2_PLL_VDDA_OWL, 5, VR_PWRGD_PIN_READING_4_REG, "PWRGD_P1V2_PLL_VDDA_OWL" },
	{ PWRGD_P1V2_PLL_VDDA_SOC, 4, VR_PWRGD_PIN_READING_4_REG, "PWRGD_P1V2_PLL_VDDA_SOC" },
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
	{ PWRGD_P4V2, 4, VR_PWRGD_PIN_READING_6_REG, "PWRGD_P4V2" },
};

const size_t power_good_status_table_for_steps_on_count =
	ARRAY_SIZE(power_good_status_table_for_steps_on);

static steps_on_struct steps_on[] = {
	// follow pwr sequence order
	{ 1, VR_AND_CLK_EN_PIN_CTRL, 1, "FM_PLD_UBC_EN", P12V_UBC1_PWRGD }, //p12V
	{ 1, VR_4_EN, 6, "FM_P3V3_EN", PWRGD_P3V3_R }, //FM_P3V3_EN
	{ 1, VR_AND_CLK_EN_PIN_CTRL, 3, "FM_P4V2_EN_R", PWRGD_P4V2 }, //FM_P4V2_EN_R
	{ 1, VR_4_EN, 7, "FM_P5V_EN", PWRGD_P5V_R }, //FM_P5V_EN
	{ 1, VR_4_EN, 5, "LDO_IN_1V2_EN_R", PWRGD_LDO_IN_1V2_R }, //LDO_IN_1V2_EN_R
	{ 1, VR_4_EN, 4, "FM_P1V80_EN", PWRGD_P1V8_R }, //FM_P1V80_EN
	{ 1, VR_3_EN, 0, "FM_AVDD_HCSL_EN", PWRGD_P0V75_AVDD_HCSL }, //FM_AVDD_HCSL_EN
	{ 1, VR_1_EN, 3, "FM_HAMSA_VDD_EN", PWRGD_HAMSA_VDD_R }, //FM_HAMSA_VDD_EN
	{ 1, VR_1_EN, 7, "NUWA1_VDD_EN", PWRGD_NUWA1_VDD }, //NUWA1_VDD_EN
	{ 1, VR_1_EN, 6, "FM_NUWA0_VDD_EN", PWRGD_NUWA0_VDD }, //FM_NUWA0_VDD_EN
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
	{ 1, VR_3_EN, 5, "FM_P1V2_PLL_VDDA_OWL_EN",
	  PWRGD_P1V2_PLL_VDDA_OWL }, //FM_P1V2_PLL_VDDA_OWL_EN
	{ 1, VR_3_EN, 4, "FM_P1V2_PLL_VDDA_SOC_EN",
	  PWRGD_P1V2_PLL_VDDA_SOC }, //FM_P1V2_PLL_VDDA_SOC_EN
	{ 1, VR_4_EN, 1, "FM_P1V5_PLL_VDDA_OWL_EN",
	  PWRGD_P1V5_PLL_VDDA_OWL_E }, //FM_P1V5_PLL_VDDA_OWL_EN
	{ 1, VR_4_EN, 0, "FM_P1V5_PLL_VDDA_SOC_EN",
	  PWRGD_P1V5_PLL_VDDA_SOC }, //FM_P1V5_PLL_VDDA_SOC_EN
	{ 1, VR_AND_CLK_EN_PIN_CTRL, 2, "FM_P3V3_CLK_EN", NO_DEFINED }, //FM_P3V3_CLK_EN
	{ 0, VR_AND_CLK_EN, 6, "FM_ELECTRA_CLK_100MHZ_EN_N",
	  NO_DEFINED }, //FM_ELECTRA_CLK_100MHZ_EN_N
	{ 1, VR_AND_CLK_EN, 7, "FM_ELECTRA_CLK_48MHZ_EN", NO_DEFINED }, //FM_ELECTRA_CLK_48MHZ_EN
	{ 0, VR_AND_CLK_EN_PIN_CTRL, 0, "FM_ARKE_CLK_312MHZ_EN_N",
	  NO_DEFINED }, //FM_ARKE_CLK_312MHZ_EN_N
	{ 1, VR_2_EN, 5, "FM_VPP_HBM0_HBM2_HBM4_HBM6_EN",
	  PWRGD_VPP_HBM0_HBM2_HBM4_HBM6_R }, //FM_VPP_HBM0_HBM2_HBM4_HBM6_EN
	{ 1, VR_2_EN, 1, "FM_VPP_HBM1_HBM3_HBM5_HBM7_EN",
	  PWRGD_VPP_HBM1_HBM3_HBM5_HBM7_R }, //FM_VPP_HBM1_HBM3_HBM5_HBM7_EN
	{ 1, VR_2_EN, 6, "FM_VDDC_HBM0_HBM2_HBM4_HBM6_EN",
	  PWRGD_VDDC_HBM0_HBM2_HBM4_HBM6_R }, //FM_VDDC_HBM0_HBM2_HBM4_HBM6_EN
	{ 1, VR_2_EN, 2, "FM_VDDC_HBM1_HBM3_HBM5_HBM7_EN",
	  PWRGD_VDDC_HBM1_HBM3_HBM5_HBM7_R }, //FM_VDDC_HBM1_HBM3_HBM5_HBM7_EN
	{ 1, VR_3_EN, 7, "FM_VDDQ_HBM0_HBM2_HBM4_HBM6_EN",
	  PWRGD_VDDQ_HBM0_HBM2_HBM4_HBM6_R }, //FM_VDDQ_HBM0_HBM2_HBM4_HBM6_EN
	{ 1, VR_3_EN, 6, "FM_VDDQ_HBM1_HBM3_HBM5_HBM7_EN",
	  PWRGD_VDDQ_HBM1_HBM3_HBM5_HBM7_R }, //FM_VDDQ_HBM1_HBM3_HBM5_HBM7_EN
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
	{ 1, VR_3_EN, 2, "FM_PVDD1P5_EN", PWRGD_PVDD1P5 }, //FM_PVDD1P5_EN
	{ 1, VR_AND_CLK_EN, 0, "FM_HAMSA_VDDHRXTX_PCIE_EN",
	  PWRGD_HAMSA_VDDHRXTX_PCIE_R }, //FM_HAMSA_VDDHRXTX_PCIE_EN
	{ 1, VR_AND_CLK_EN_PIN_CTRL, 6, "ASIC_POWER_ON_RESET_N",
	  NO_DEFINED }, //ASIC_POWER_ON_RESET_N
	{ 1, VR_AND_CLK_EN_PIN_CTRL, 5, "ASIC_SYS_RST_N", NO_DEFINED }, //ASIC_SYS_RST_N
	{ 1, VR_AND_CLK_EN_PIN_CTRL, 4, "HAMSA_PCIE_PERST_B_PLD_N",
	  NO_DEFINED }, //HAMSA_PCIE_PERST_B_PLD_N
};

pwr_clock_compnt_mapping pwr_clock_compnt_mapping_table[] = {
	{ CLK_BUF_100M_U85, CLK_BUF_U85_ADDR, I2C_BUS1, "LOS_EVT_CLK_BUF_100M_U85" },
	{ CLK_BUF_100M_U690, CLK_BUF_U690_ADDR, I2C_BUS1, "LOS_EVT_CLK_BUF_100M_U690" },
	{ CLK_BUF_100M_U88, CLK_BUF_U88_ADDR, I2C_BUS3, "LOS_EVT_CLK_BUF_100M_U88" },
	{ CLK_GEN_100M_U86, CLK_GEN_100M_U86_ADDR, I2C_BUS3, "LOS_EVT_CLK_GEN_100M_U86" },
};
// clang-format on

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

typedef struct ioe_power_good_status {
	uint8_t bus;
	uint8_t addr;
	uint8_t bit_loc;
	uint8_t *ioe_pwrgd_name;

} ioe_power_good_status;

ioe_power_good_status ioe_pwrgd_status_table[] = {
	{ U200052_IO_I2C_BUS, U200052_IO_ADDR, 0, "PWRGD_P3V3_OSFP_P1" },
	{ U200052_IO_I2C_BUS, U200052_IO_ADDR, 1, "PWRGD_P3V3_OSFP_P2" },
	{ U200052_IO_I2C_BUS, U200052_IO_ADDR, 2, "PWRGD_P3V3_OSFP_P3" },
	{ U200052_IO_I2C_BUS, U200052_IO_ADDR, 3, "PWRGD_P3V3_OSFP_P4" },
	{ U200052_IO_I2C_BUS, U200052_IO_ADDR, 4, "PWRGD_P3V3_OSFP_P5" },
	{ U200052_IO_I2C_BUS, U200052_IO_ADDR, 5, "PWRGD_P3V3_OSFP_P6" },
};

bool check_p3v3_p5v_pwrgd(void)
{
	// read p3v3_pwrgf and p5v_pwrgf
	// PWRGD_P3V3_R, bit-4, VR_PWRGD_PIN_READING_5_REG
	uint8_t offset = VR_PWRGD_PIN_READING_5_REG;
	uint8_t reg_data = 0;
	if (!plat_read_cpld(offset, &reg_data, 1)) {
		LOG_ERR("Read CPLD offset 0x%x failed", offset);
	}
	uint8_t p3v3_value = (reg_data >> 4) & 0x01;
	// PWRGD_P5V_R, bit-5, VR_PWRGD_PIN_READING_5_REG
	uint8_t p5v_value = (reg_data >> 5) & 0x01;
	//if both p3v3 and p5v are all 1, return true
	if (p3v3_value == 1 && p5v_value == 1)
		return true;
	return false;
}

void pwer_gd_get_status(const struct shell *shell)
{
	uint8_t check_value = 0;
	int ret = 0;
	ret = get_pca6554apw_ioe_value(ioe_pwrgd_status_table[0].bus,
				       ioe_pwrgd_status_table[0].addr, INPUT_PORT, &check_value);

	if (ret == -1) {
		return;
	}

	for (int i = 0; i < sizeof(ioe_pwrgd_status_table) / sizeof(ioe_pwrgd_status_table[0]);
	     i++) {
		uint8_t tmp_value = (check_value >> i) & 0x01;
		shell_print(shell, "%s : %d", ioe_pwrgd_status_table[i].ioe_pwrgd_name, tmp_value);
	}
}

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

#define MAX_STEPS (sizeof(steps_on) / sizeof(steps_on[0]))
int power_steps = 0;

static bool arke_power_control(uint8_t onoff)
{
	uint8_t tmp = onoff ? 0x80 : 0x00;
	return plat_write_cpld(CPLD_OFFSET_MMC_PWR_EN, &tmp);
}

void cmd_arke_power_on(const struct shell *shell, size_t argc, char **argv)
{
	if (!arke_power_control(1))
		shell_warn(shell, "arke power on set cpld fail!");
	// wait 1s
	k_msleep(1500);
	if (gpio_get(RST_ARKE_PWR_ON_PLD_R1_N) == GPIO_HIGH) {
		shell_print(shell, "arke power on success!");
		set_pwr_steps_on_flag(0);
		set_plat_sensor_one_step_enable_flag(false);
	} else {
		shell_warn(shell, "arke power on fail!");
	}
}

void cmd_arke_power_off(const struct shell *shell, size_t argc, char **argv)
{
	if (!arke_power_control(0))
		shell_warn(shell, "arke power off set cpld fail!");
	// wait 1s
	k_msleep(1500);
	if (gpio_get(FM_PLD_UBC_EN_R) == GPIO_LOW) {
		shell_print(shell, "arke power off success!");
	} else {
		shell_warn(shell, "arke power off fail!");
	}

	power_steps = 0;
	// init power steps
	set_pwr_steps_on_flag(0);
	set_plat_sensor_one_step_enable_flag(false);
}

void cmd_arke_power_cycle(const struct shell *shell, size_t argc, char **argv)
{
	if (!arke_power_control(0))
		shell_warn(shell, "arke power cycle(off) fail!");
	k_msleep(5000);
	if (!arke_power_control(1))
		shell_warn(shell, "arke power cycle(on) fail!");
	set_pwr_steps_on_flag(0);
	set_plat_sensor_one_step_enable_flag(false);
}

static power_good_status *find_pwrgd_entry(uint8_t idx)
{
	for (size_t i = 0; i < power_good_status_table_for_steps_on_count; i++) {
		if (power_good_status_table_for_steps_on[i].index == idx)
			return &power_good_status_table_for_steps_on[i];
	}
	return NULL;
}

void cmd_arke_steps_on(const struct shell *shell, size_t argc, char **argv)
{
	if (power_steps >= MAX_STEPS) {
		LOG_WRN("no more steps");
		return;
	}

	// need to set VR_1STEP_FUNC_EN_REG to 0
	if (power_steps == 0) {
		if (gpio_get(RST_ARKE_PWR_ON_PLD_R1_N) == GPIO_HIGH) {
			shell_warn(shell, "arke power is already on, skip steps on operation.");
			set_pwr_steps_on_flag(0);
			return;
		}
		set_pwr_steps_on_flag(1);
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
		// if steps on name == FM_P3V3_CLK_EN, do next steps immediately
		if (strcmp(steps_on[power_steps].name, "FM_P3V3_CLK_EN") == 0) {
			power_steps += 1;
			// call self function
			// turn on the steps on bit
			if (set_cpld_bit(steps_on[power_steps].cpld_offset,
					 steps_on[power_steps].bit,
					 steps_on[power_steps].power_on_value) == false) {
				shell_print(shell, "set %-35s fail", steps_on[power_steps].name);
			} else {
				shell_print(shell, "set %-35s success", steps_on[power_steps].name);
			}
			k_msleep(1000);
			for (int i = 0; i < CLK_COMPONENT_MAX; i++) {
				k_msleep(100);
				clear_clock_status(shell, i);
				k_msleep(100);
				pwr_get_clock_status(shell, i);
			}
		}
		power_steps += 1;
		return;
	}
	//delay 1s
	k_msleep(1500);

	power_good_status *entry = find_pwrgd_entry(pwrgd_idx);
	uint8_t reg_data = 0;
	uint8_t value = 0;

	if (pwrgd_idx == P12V_UBC1_PWRGD) {
		power_good_status *ubc2 = find_pwrgd_entry(P12V_UBC2_PWRGD);
		if (!plat_read_cpld(entry->cpld_offsets, &reg_data, 1))
			shell_error(shell, "Read CPLD offset 0x%x failed", entry->cpld_offsets);
		value = (reg_data >> entry->bit_loc) & 0x01;
		shell_print(shell, "%-20s %d", ubc2->power_rail_name, value);
		if (!plat_read_cpld(ubc2->cpld_offsets, &reg_data, 1))
			shell_error(shell, "Read CPLD offset 0x%x failed", ubc2->cpld_offsets);
		value = (reg_data >> ubc2->bit_loc) & 0x01;
		shell_print(shell, "%-20s %d", ubc2->power_rail_name, value);
	} else if (pwrgd_idx == PWRGD_P1V5_PLL_VDDA_OWL_E) {
		power_good_status *owl_w = find_pwrgd_entry(PWRGD_P1V5_PLL_VDDA_OWL_W);
		if (!plat_read_cpld(entry->cpld_offsets, &reg_data, 1))
			shell_error(shell, "Read CPLD offset 0x%x failed", entry->cpld_offsets);
		value = (reg_data >> entry->bit_loc) & 0x01;
		shell_print(shell, "%-20s %d", entry->power_rail_name, value);
		if (!plat_read_cpld(owl_w->cpld_offsets, &reg_data, 1))
			shell_error(shell, "Read CPLD offset 0x%x failed", owl_w->cpld_offsets);
		value = (reg_data >> owl_w->bit_loc) & 0x01;
		shell_print(shell, "%-20s %d", owl_w->power_rail_name, value);
	} else if (pwrgd_idx == PWRGD_P0V9_OWL_E_PVDD) {
		power_good_status *pvdd_w = find_pwrgd_entry(PWRGD_P0V9_OWL_W_PVDD);
		if (!plat_read_cpld(entry->cpld_offsets, &reg_data, 1))
			shell_error(shell, "Read CPLD offset 0x%x failed", entry->cpld_offsets);
		value = (reg_data >> entry->bit_loc) & 0x01;
		shell_print(shell, "%-20s %d", entry->power_rail_name, value);
		if (!plat_read_cpld(pvdd_w->cpld_offsets, &reg_data, 1))
			shell_error(shell, "Read CPLD offset 0x%x failed", pvdd_w->cpld_offsets);
		value = (reg_data >> pvdd_w->bit_loc) & 0x01;
		shell_print(shell, "%-20s %d", pvdd_w->power_rail_name, value);
	} else if (pwrgd_idx == PWRGD_P1V5_E_RVDD) {
		power_good_status *rvdd_w = find_pwrgd_entry(PWRGD_P1V5_W_RVDD);
		if (!plat_read_cpld(entry->cpld_offsets, &reg_data, 1))
			shell_error(shell, "Read CPLD offset 0x%x failed", entry->cpld_offsets);
		value = (reg_data >> entry->bit_loc) & 0x01;
		shell_print(shell, "%-20s %d", entry->power_rail_name, value);
		if (!plat_read_cpld(rvdd_w->cpld_offsets, &reg_data, 1))
			shell_error(shell, "Read CPLD offset 0x%x failed", rvdd_w->cpld_offsets);
		value = (reg_data >> rvdd_w->bit_loc) & 0x01;
		shell_print(shell, "%-20s %d", rvdd_w->power_rail_name, value);
	} else {
		//read from CPLD
		if (!plat_read_cpld(entry->cpld_offsets, &reg_data, 1))
			shell_error(shell, "Read CPLD offset 0x%x failed", entry->cpld_offsets);
		value = (reg_data >> entry->bit_loc) & 0x01;
		shell_print(shell, "%-20s %d", entry->power_rail_name, value);
	}

	if (strcmp(steps_on[power_steps].name, "FM_P5V_EN") == 0) {
		if (check_p3v3_p5v_pwrgd()) {
			set_plat_sensor_one_step_enable_flag(ONE_STEP_POWER_MAGIC_NUMBER);
		}
	}

	power_steps += 1;
}

void cmd_arke_disable_steps_on(const struct shell *shell, size_t argc, char **argv)
{
	// init power steps
	power_steps = 0;
	set_pwr_steps_on_flag(0);
	// init value is reverse of power on value
	uint8_t power_init_value = 0;
	set_plat_sensor_one_step_enable_flag(false);
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

SHELL_STATIC_SUBCMD_SET_CREATE(sub_arke_power_cmd,
			       SHELL_CMD(on, NULL, "arke power on", cmd_arke_power_on),
			       SHELL_CMD(off, NULL, "arke power off", cmd_arke_power_off),
			       SHELL_CMD(cycle, NULL, "arke power cycle", cmd_arke_power_cycle),
			       SHELL_CMD(steps_on, NULL, "arke power steps_on", cmd_arke_steps_on),
			       SHELL_CMD(disable_steps_on, NULL, "arke power disable steps_on",
					 cmd_arke_disable_steps_on),
			       SHELL_SUBCMD_SET_END);

/* Root of command echo */
SHELL_CMD_REGISTER(arke_power, &sub_arke_power_cmd, "arke power commands", NULL);
