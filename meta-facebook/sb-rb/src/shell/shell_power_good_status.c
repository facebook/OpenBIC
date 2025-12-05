/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <shell/shell.h>
#include <stdlib.h>
#include <stdio.h>
#include "plat_fru.h"
#include "plat_cpld.h"
#include "plat_class.h"
#include "shell_iris_power.h"
#include "plat_gpio.h"

LOG_MODULE_REGISTER(shell_plat_power_good_status, LOG_LEVEL_INF);

enum power_good_status_type {
	PWRGD_P1V8_AUX,
	P12V_UBC1_PWRGD,
	P12V_UBC2_PWRGD,
	PWRGD_P3V3_R,
	PWRGD_P4V2,
	PWRGD_P5V_R,
	PWRGD_LDO_IN_1V2_R,
	PWRGD_P1V8_R,
	PWRGD_P0V75_AVDD_HCSL,
	PWRGD_HAMSA_VDD_R,
	PWRGD_MEDHA1_VDD,
	PWRGD_MEDHA0_VDD,
	PWRGD_OWL_E_VDD_R,
	PWRGD_OWL_W_VDD_R,
	PWRGD_MAX_M_VDD_R,
	PWRGD_MAX_N_VDD_R,
	PWRGD_MAX_S_VDD_R,
	PWRGD_OWL_E_TRVDD0P75_R,
	PWRGD_OWL_W_TRVDD0P75_R,
	PWRGD_VDDPHY_HBM0_HBM2_HBM4_HBM6_R,
	PWRGD_VDDPHY_HBM1_HBM3_HBM5_HBM7_R,
	PWRGD_P1V5_PLL_VDDA_OWL_E,
	PWRGD_P1V5_PLL_VDDA_OWL_W,
	PWRGD_P1V5_PLL_VDDA_SOC,
	PWRGD_PLL_VDDA15_HBM0_HBM2,
	PWRGD_PLL_VDDA15_HBM1_HBM3,
	PWRGD_PLL_VDDA15_HBM4_HBM6,
	PWRGD_PLL_VDDA15_HBM5_HBM7,
	PWRGD_VPP_HBM0_HBM2_HBM4_HBM6_R,
	PWRGD_VPP_HBM1_HBM3_HBM5_HBM7_R,
	PWRGD_VDDQC_HBM0_HBM2_HBM4_HBM6_R,
	PWRGD_VDDQC_HBM1_HBM3_HBM5_HBM7_R,
	PWRGD_VDDQL_HBM0_HBM2_HBM4_HBM6_R,
	PWRGD_VDDQL_HBM1_HBM3_HBM5_HBM7_R,
	PWRGD_HAMSA_AVDD_PCIE_R,
	PWRGD_OWL_E_TRVDD0P9_R,
	PWRGD_OWL_W_TRVDD0P9_R,
	PWRGD_P0V9_OWL_E_PVDD,
	PWRGD_P0V9_OWL_W_PVDD,
	PWRGD_P1V5_E_RVDD,
	PWRGD_P1V5_W_RVDD,
	PWRGD_PVDD1P5,
	PWRGD_HAMSA_VDDHRXTX_PCIE_R,
	MODULE_PWRGD,
	PWRGD_MAX
};

typedef struct power_good_status {
	uint8_t index;
	uint8_t bit_loc;
	uint8_t cpld_offsets;
	uint8_t *power_rail_name;
} power_good_status;

power_good_status power_good_status_table[] = {
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
	{ PWRGD_P4V2, 4, VR_PWRGD_PIN_READING_6_REG, "PWRGD_P4V2" },
};

#define POWER_GOOD_STATUS_COUNT ARRAY_SIZE(power_good_status_table)
void show_power_good_status(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t last_offset = 0xFF;
	uint8_t reg_data = 0;

	for (int index = 0; index < PWRGD_MAX; index++) {
		for (int i = 0; i < POWER_GOOD_STATUS_COUNT; i++) {
			if (index != power_good_status_table[i].index)
				continue;
			uint8_t offset = power_good_status_table[i].cpld_offsets;
			uint8_t bit = power_good_status_table[i].bit_loc;

			// if offset is different, read from CPLD
			if (offset != last_offset) {
				if (!plat_read_cpld(offset, &reg_data, 1)) {
					shell_error(shell, "Read CPLD offset 0x%x failed", offset);
					continue;
				}
				last_offset = offset;
			}

			uint8_t value = (reg_data >> bit) & 0x01;

			shell_print(shell, "%-20s %d", power_good_status_table[i].power_rail_name,
				    value);
			if (get_asic_board_id() == ASIC_BOARD_ID_EVB &&
			    power_good_status_table[i].index == PWRGD_P1V8_R) {
				//check pwrgd PWRGD_P1V8_AUX status is on
				if (!value) {
					shell_print(
						shell,
						"P1V8 is off, skip p3v3_osfp power good status check.");
				} else
					pwer_gd_get_status(shell);
			}
		}
	}
}

SHELL_CMD_REGISTER(power_good_status, NULL, "Show all the power good status",
		   show_power_good_status);
