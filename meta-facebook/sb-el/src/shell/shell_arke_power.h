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
#ifndef PLAT_ARKE_POWER_H
#define PLAT_ARKE_POWER_H
typedef struct pwr_clock_compnt_mapping {
	uint8_t clock_name_index;
	uint8_t addr;
	uint8_t bus;
	uint8_t *clock_name;
} pwr_clock_compnt_mapping;

enum PWR_CLOCK_COMPONENT {
	CLK_BUF_100M_U85,
	CLK_BUF_100M_U690,
	CLK_BUF_100M_U88,
	CLK_GEN_100M_U86,
	CLK_COMPONENT_MAX
};

enum power_good_status_type_for_steps_on {
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
	PWRGD_NUWA1_VDD,
	PWRGD_NUWA0_VDD,
	PWRGD_OWL_E_VDD_R,
	PWRGD_OWL_W_VDD_R,
	PWRGD_MAX_M_VDD_R,
	PWRGD_MAX_N_VDD_R,
	PWRGD_MAX_S_VDD_R,
	PWRGD_OWL_E_TRVDD0P75_R,
	PWRGD_OWL_W_TRVDD0P75_R,
	PWRGD_VDDPHY_HBM0_HBM2_HBM4_HBM6_R,
	PWRGD_VDDPHY_HBM1_HBM3_HBM5_HBM7_R,
	PWRGD_P1V2_PLL_VDDA_OWL,
	PWRGD_P1V2_PLL_VDDA_SOC,
	PWRGD_P1V5_PLL_VDDA_OWL_E,
	PWRGD_P1V5_PLL_VDDA_OWL_W,
	PWRGD_P1V5_PLL_VDDA_SOC,
	PWRGD_VPP_HBM0_HBM2_HBM4_HBM6_R,
	PWRGD_VPP_HBM1_HBM3_HBM5_HBM7_R,
	PWRGD_VDDC_HBM0_HBM2_HBM4_HBM6_R,
	PWRGD_VDDC_HBM1_HBM3_HBM5_HBM7_R,
	PWRGD_VDDQ_HBM0_HBM2_HBM4_HBM6_R,
	PWRGD_VDDQ_HBM1_HBM3_HBM5_HBM7_R,
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

#define CLK_BUF_U85_ADDR (0xCE >> 1)
#define CLK_BUF_U690_ADDR (0xD8 >> 1)
#define CLK_BUF_U88_ADDR (0xDE >> 1)
#define CLK_GEN_100M_U86_ADDR 0x9
#define CLK_BUF_100M_WRITE_LOCK_CLEAR_LOS_EVENT_OFFSET 0x27
#define CLK_GEN_LOSMON_EVENT_OFFSET 0x5a
#define CLK_BUF_100M_BYTE_COUNT 0x7

typedef struct power_good_status {
	uint8_t index;
	uint8_t bit_loc;
	uint8_t cpld_offsets;
	uint8_t *power_rail_name;

} power_good_status;

extern power_good_status power_good_status_table_for_steps_on[];
extern const size_t power_good_status_table_for_steps_on_count;

typedef struct steps_on_struct {
	uint8_t power_on_value;
	uint8_t cpld_offset;
	uint8_t bit;
	uint8_t *name;
	uint8_t pwrgd_idx;
} steps_on_struct;

bool check_p3v3_p5v_pwrgd(void);
void pwer_gd_get_status(const struct shell *shell);
void clear_clock_status(const struct shell *shell, uint8_t clock_index);
#endif
