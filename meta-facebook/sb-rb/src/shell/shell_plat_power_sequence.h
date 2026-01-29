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

#ifndef PLAT_PWR_SEQUENCE_H
#define PLAT_PWR_SEQUENCE_H

#include <stdint.h>

// Power sequence On (0x44~0x72)
#define P12V_ON_REG 0x43
#define P3V3_ON_REG 0x44
#define P5V_ON_REG 0x45
#define LDO_IN_1V2_ON_REG 0x46
#define P1V8_ON_REG 0x47
#define P0V75_AVDD_HCSL_ON_REG 0x48
#define HAMSA_VDD_ON_REG 0x49
#define MEDHA1_VDD_ON_REG 0x4A
#define MEDHA0_VDD_ON_REG 0x4B
#define OWL_E_VDD_ON_REG 0x4C
#define OWL_W_VDD_ON_REG 0x4D
#define MAX_M_VDD_ON_REG 0x4E
#define MAX_N_VDD_ON_REG 0x4F
#define MAX_S_VDD_ON_REG 0x50
#define OWL_E_TRVDD0P75_ON_REG 0x51
#define OWL_W_TRVDD0P75_ON_REG 0x52
#define VDDPHY_HBM0_HBM2_HBM4_HBM6_ON_REG 0x53
#define VDDPHY_HBM1_HBM3_HBM5_HBM7_ON_REG 0x54
#define P1V5_PLL_VDDA_OWL_E_ON_REG 0x55
#define P1V5_PLL_VDDA_SOC_ON_REG 0x56
#define PLL_VDDA15_HBM0_HBM2_ON_REG 0x57
#define PLL_VDDA15_HBM1_HBM3_ON_REG 0x58
#define PLL_VDDA15_HBM4_HBM6_ON_REG 0x59
#define PLL_VDDA15_HBM5_HBM7_ON_REG 0x5A
#define IRIS_CLK_100MHZ_ON_REG 0x5B
#define IRIS_CLK_48MHZ_ON_REG 0x5C
#define IRIS_CLK_312_5_MHZ_ON_REG 0x5D
#define VPP_HBM0_HBM2_HBM4_HBM6_ON_REG 0x5E
#define VPP_HBM1_HBM3_HBM5_HBM7_ON_REG 0x5F
#define VDDQC_HBM0_HBM2_HBM4_HBM6_ON_REG 0x60
#define VDDQC_HBM1_HBM3_HBM5_HBM7_ON_REG 0x61
#define VDDQL_HBM0_HBM2_HBM4_HBM6_ON_REG 0x62
#define VDDQL_HBM1_HBM3_HBM5_HBM7_ON_REG 0x63
#define HAMSA_AVDD_PCIE_ON_REG 0x64
#define OWL_E_TRVDD0P9_ON_REG 0x65
#define OWL_W_TRVDD0P9_ON_REG 0x66
#define OWL_E_PVDD0P9_ON_REG 0x67
#define OWL_W_PVDD0P9_ON_REG 0x68
#define OWL_E_RVDD1P5_ON_REG 0x69
#define OWL_W_RVDD1P5_ON_REG 0x6A
#define HAMSA_VDDHRXTX_PCIE_ON_REG 0x6B
#define PVDD1P5_ON_REG 0x6C
#define HAMSA_POWER_ON_RESET_PLD_L_ON_REG 0x6D
#define MEDHA0_POWER_ON_RESET_PLD_L_ON_REG 0x6E
#define MEDHA1_POWER_ON_RESET_PLD_L_ON_REG 0x6F
#define HAMSA_SYS_RST_PLD_L_ON_REG 0x70
#define MEDHA0_SYS_RST_PLD_L_ON_REG 0x71
#define MEDHA1_SYS_RST_PLD_L_ON_REG 0x72
#define P4V2_ON_REG 0xAB
#define P1V5_PLL_VDDA_OWL_W_ON_REG 0xAC
#define FM_P3V3_CLK_ON_REG 0xAD

// Power sequence Off (0x73~0x9D)
#define PVDD1P5_DOWN_REG 0x73
#define HAMSA_VDDHRXTX_PCIE_DOWN_REG 0x74
#define P1V5_E_RVDD_DOWN_REG 0x75
#define P1V5_W_RVDD_DOWN_REG 0x76
#define P0V9_OWL_E_PVDD_DOWN_REG 0x77
#define P0V9_OWL_W_PVDD_DOWN_REG 0x78
#define OWL_E_TRVDD0P9_DOWN_REG 0x79
#define OWL_W_TRVDD0P9_DOWN_REG 0x7A
#define HAMSA_AVDD_PCIE_DOWN_REG 0x7B
#define VDDQL_HBM0_HBM2_HBM4_HBM6_DOWN_REG 0x7C
#define VDDQL_HBM1_HBM3_HBM5_HBM7_DOWN_REG 0x7D
#define VDDQC_HBM0_HBM2_HBM4_HBM6_DOWN_REG 0x7E
#define VDDQC_HBM1_HBM3_HBM5_HBM7_DOWN_REG 0x7F
#define VPP_HBM0_HBM2_HBM4_HBM6_DOWN_REG 0x80
#define VPP_HBM1_HBM3_HBM5_HBM7_DOWN_REG 0x81
#define FM_IRIS_CLK_100MHZ_EN_N_DOWN_REG 0x82
#define FM_CLK_48MHZ_EN_DOWN_REG 0x83
#define FM_IRIS_CLK_312MHZ_EN_N_DOWN_REG 0x84
#define PLL_VDDA15_HBM0_HBM2_DOWN_REG 0x85
#define PLL_VDDA15_HBM1_HBM3_DOWN_REG 0x86
#define PLL_VDDA15_HBM4_HBM6_DOWN_REG 0x87
#define PLL_VDDA15_HBM5_HBM7_DOWN_REG 0x88
#define P1V5_PLL_VDDA_SOC_DOWN_REG 0x89
#define P1V5_PLL_VDDA_OWL_E_DOWN_REG 0x8A
#define VDDPHY_HBM0_HBM2_HBM4_HBM6_DOWN_REG 0x8B
#define VDDPHY_HBM1_HBM3_HBM5_HBM7_DOWN_REG 0x8C
#define OWL_E_TRVDD0P75_DOWN_REG 0x8D
#define OWL_W_TRVDD0P75_DOWN_REG 0x8E
#define MAX_S_VDD_DOWN_REG 0x8F
#define MAX_N_VDD_DOWN_REG 0x90
#define MAX_M_VDD_DOWN_REG 0x91
#define OWL_E_VDD_DOWN_REG 0x92
#define OWL_W_VDD_DOWN_REG 0x93
#define MEDHA0_VDD_DOWN_REG 0x94
#define MEDHA1_VDD_DOWN_REG 0x95
#define HAMSA_VDD_DOWN_REG 0x96
#define P0V75_AVDD_HCSL_DOWN_REG 0x97
#define P1V8_DOWN_REG 0x98
#define LDO_IN_1V2_DOWN_REG 0x99
#define P5V_DOWN_REG 0x9A
#define P3V3_DOWN_REG 0x9B
#define P12V_UBC_DOWN_REG 0x9C
#define PERST_DELAY_DOWN_REG 0x9D
#define P4V2_DOWN_REG 0xAE
#define P1V5_PLL_VDDA_OWL_W_DOWN_REG 0xAF
#define FM_P3V3_CLK_DOWN_REG 0xB0

typedef struct power_sequence {
	uint8_t index;
	uint8_t cpld_offsets;
	uint8_t *power_rail_name;
	uint8_t value;
} power_sequence;

bool plat_find_power_seq_fail();
uint8_t plat_get_power_seq_fail_id();
void plat_get_power_seq_fail_name(uint8_t idx, uint8_t **name);

#endif
