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

#ifndef PLAT_EVENT_H
#define PLAT_EVENT_H

#define POLLING_CPLD_STACK_SIZE 2048

// Power sequence On (0x48~0x6F)
#define P12V_UBC_PWRGD_ON_REG 0x48
#define PWRGD_P5V_R_ON_REG 0x49
#define PWRGD_P3V3_OSC_ON_REG 0x4A
#define PWRGD_P3V3_ON_REG 0x4B
#define PWRGD_LDO_IN_1V2_R_ON_REG 0x4C
#define PWRGD_P0V85_PVDD_ON_REG 0x4D
#define PWRGD_P0V75_PVDD_CH_N_ON_REG 0x4E
#define PWRGD_P0V75_MAX_PHY_N_ON_REG 0x4F
#define PWRGD_P0V75_PVDD_CH_S_ON_REG 0x50
#define PWRGD_P0V75_MAX_PHY_S_ON_REG 0x51
#define PWRGD_P0V75_VDDPHY_HBM0_HBM2_HBM4_R_ON_REG 0x52
#define PWRGD_P0V75_VDDPHY_HBM1_HBM3_HBM5_R_ON_REG 0x53
#define PWRGD_P0V75_TRVDD_ZONEA_R_ON_REG 0x54
#define PWRGD_P0V75_TRVDD_ZONEB_R_ON_REG 0x55
#define PWRGD_P0V75_AVDD_HSCL_R_ON_REG 0x56
#define PWRGD_P0V75_VDDC_CLKOBS_R_ON_REG 0x57
#define PWRGD_LDO_IN_1V8_R_ON_REG 0x58
#define PWRGD_PLL_VDDA15_MAX_CORE_N_ON_REG 0x59
#define PWRGD_PLL_VDDA15_MAX_CORE_S_ON_REG 0x5A
#define PWRGD_PLL_VDDA15_PCIE_MAX_CORE_ON_REG 0x5B
#define PWRGD_PLL_VDDA15_HBM0_HBM2_HBM4_ON_REG 0x5C
#define PWRGD_PLL_VDDA15_HBM1_HBM3_HBM5_ON_REG 0x5D
#define PWRGD_VPP_HBM0_HBM2_HBM4_R_ON_REG 0x5E
#define PWRGD_VPP_HBM1_HBM3_HBM5_R_ON_REG 0x5F
#define PWRGD_P1V1_VDDC_HBM0_HBM2_HBM4_R_ON_REG 0x60
#define PWRGD_P1V1_VDDC_HBM1_HBM3_HBM5_R_ON_REG 0x61
#define PWRGD_VDDQL_HBM0_HBM2_HBM4_R_ON_REG 0x62
#define PWRGD_VDDQL_HBM1_HBM3_HBM5_R_ON_REG 0x63
#define FM_AEGIS_CLK_48MHZ_EN_ON_REG 0x64
#define FM_AEGIS_CLK_100MHZ_EN_N_ON_REG 0x65
#define FM_AEGIS_CLK_312MHZ_EN_ON_REG 0x66
#define PWRGD_VDDA_PCIE_R_ON_REG 0x67
#define PWRGD_P0V9_TRVDD_ZONEA_R_ON_REG 0x68
#define PWRGD_P0V9_TRVDD_ZONEB_R_ON_REG 0x69
#define PWRGD_PVDD0P9_N_ON_REG 0x6A
#define PWRGD_PVDD0P9_S_ON_REG 0x6B
#define PWRGD_PVDD1P5_N_ON_REG 0x6C
#define PWRGD_PVDD1P5_S_ON_REG 0x6D
#define PWRGD_VDDHTX_PCIE_R_ON_REG 0x6E
#define RST_ATH_PWR_ON_PLD_N_ON_REG 0x6F

// Power sequence Off (0x70~0x96)
#define PWRGD_VDDHTX_PCIE_R_OFF_REG 0x70
#define PWRGD_PVDD1P5_N_OFF_REG 0x71
#define PWRGD_PVDD1P5_S_OFF_REG 0x72
#define FM_AEGIS_CLK_48MHZ_EN_OFF_REG 0x73
#define FM_AEGIS_CLK_100MHZ_EN_N_OFF_REG 0x74
#define FM_AEGIS_CLK_312MHZ_EN_OFF_REG 0x75
#define PWRGD_VDDA_PCIE_R_OFF_REG 0x76
#define PWRGD_P0V9_TRVDD_ZONEA_R_OFF_REG 0x77
#define PWRGD_P0V9_TRVDD_ZONEB_R_OFF_REG 0x78
#define PWRGD_PVDD0P9_N_OFF_REG 0x79
#define PWRGD_PVDD0P9_S_OFF_REG 0x7A
#define PWRGD_VDDQL_HBM0_HBM2_HBM4_R_OFF_REG 0x7B
#define PWRGD_VDDQL_HBM1_HBM3_HBM5_R_OFF_REG 0x7C
#define PWRGD_P1V1_VDDC_HBM0_HBM2_HBM4_R_OFF_REG 0x7D
#define PWRGD_P1V1_VDDC_HBM1_HBM3_HBM5_R_OFF_REG 0x7E
#define PWRGD_VPP_HBM0_HBM2_HBM4_R_OFF_REG 0x7F
#define PWRGD_VPP_HBM1_HBM3_HBM5_R_OFF_REG 0x80
#define PWRGD_PLL_VDDA15_MAX_CORE_N_OFF_REG 0x81
#define PWRGD_PLL_VDDA15_MAX_CORE_S_OFF_REG 0x82
#define PWRGD_PLL_VDDA15_PCIE_MAX_CORE_OFF_REG 0x83
#define PWRGD_PLL_VDDA15_HBM0_HBM2_HBM4_OFF_REG 0x84
#define PWRGD_PLL_VDDA15_HBM1_HBM3_HBM5_OFF_REG 0x85
#define PWRGD_LDO_IN_1V8_R_OFF_REG 0x86
#define PWRGD_P0V75_TRVDD_ZONEA_R_OFF_REG 0x87
#define PWRGD_P0V75_TRVDD_ZONEB_R_OFF_REG 0x88
#define PWRGD_P0V75_AVDD_HSCL_R_OFF_REG 0x89
#define PWRGD_P0V75_VDDC_CLKOBS_R_OFF_REG 0x8A
#define PWRGD_P0V85_PVDD_OFF_REG 0x8B
#define PWRGD_P0V75_PVDD_CH_N_OFF_REG 0x8C
#define PWRGD_P0V75_MAX_PHY_N_OFF_REG 0x8D
#define PWRGD_P0V75_PVDD_CH_S_OFF_REG 0x8E
#define PWRGD_P0V75_MAX_PHY_S_OFF_REG 0x8F
#define PWRGD_P0V75_VDDPHY_HBM0_HBM2_HBM4_R_OFF_REG 0x90
#define PWRGD_P0V75_VDDPHY_HBM1_HBM3_HBM5_R_OFF_REG 0x91
#define PWRGD_LDO_IN_1V2_R_OFF_REG 0x92
#define PWRGD_P3V3_OFF_REG 0x93
#define PWRGD_P3V3_OSC_OFF_REG 0x94
#define PWRGD_P5V_R_OFF_REG 0x95
#define P12V_UBC_PWRGD_OFF_REG 0x96

#define POWER_AND_RESET_BUTTON_REG 0x00
#define VR_AND_CLK_ENABLE_PIN_READING_REG 0x01
#define VR_ENABLE_PIN_READING_1_REG 0x02
#define VR_ENABLE_PIN_READING_2_REG 0x03
#define VR_ENABLE_PIN_READING_3_REG 0x04
#define VR_ENABLE_PIN_READING_4_REG 0x05
#define MB_POWER_GOOD_AND_PERST_PIN_READING_REG 0x06
#define VR_POWER_GOOD_PIN_READING_1_REG 0x07
#define VR_POWER_GOOD_PIN_READING_2_REG 0x08
#define VR_POWER_GOOD_PIN_READING_3_REG 0x09
#define VR_POWER_GOOD_PIN_READING_4_REG 0x0A
#define VR_POWER_GOOD_PIN_READING_5_REG 0x0B
#define RSVD_1_REG 0x0C
#define VR_POWER_FAULT_1_REG 0x0D
#define VR_POWER_FAULT_2_REG 0x0E
#define VR_POWER_FAULT_3_REG 0x0F
#define VR_POWER_FAULT_4_REG 0x10
#define VR_POWER_FAULT_5_REG 0x11
#define RSVD_2_REG 0x12
#define RSVD_3_REG 0x13
#define OSFP_PRSNT_PIN_READING_1_REG 0x14
#define OSFP_PRSNT_PIN_READING_2_REG 0x15
#define OSFP_POWER_ENABLE_PIN_READING_1_REG 0x16
#define OSFP_POWER_ENABLE_PIN_READING_2_REG 0x17
#define OSFP_POWER_ENABLE_PIN_READING_3_REG 0x18
#define OSFP_POWER_ENABLE_PIN_READING_4_REG 0x19
#define BOARD_TYPE_REG 0x1A
#define BOARD_REV_ID_REG 0x1B
#define VR_VENDOR_TYPE_REG 0x1C
#define OWL_JTAG_SEL_MUX_REG 0x1D
#define ATH_JTAG_SEL_MUX_REG 0x1E
#define OWL_UART_SEL_MUX_REG 0x1F
#define AEGIS_JTAG_SWITCH_REG 0x20
#define ATH_BOOT_SOURCE_REG 0x21
#define S_OWL_BOOT_SOURCE_REG 0x22
#define N_OWL_BOOT_SOURCE_REG 0x23
#define VR_SMBUS_ALERT_1_REG 0x24
#define VR_SMBUS_ALERT_2_REG 0x25
#define RSVD_4_REG 0x26
#define ASIC_OC_WARN_REG 0x27
#define SYSTEM_ALERT_FAULT_REG 0x28
#define VR_HOT_FAULT_1_REG 0x29
#define VR_HOT_FAULT_2_REG 0x2A
#define TEMPERATURE_IC_OVERT_FAULT_REG 0x2B
#define VR_POWER_INPUT_FAULT_1_REG 0x2C
#define VR_POWER_INPUT_FAULT_2_REG 0x2D
#define LEAK_DETCTION_REG 0x2E
#define RESET_PIN_TO_ICS_STATUS_REG 0x2F
#define CRD_STATUS_REG 0x30
#define CMN_STATUS_REG 0x31
#define RSVD_GPIO_STATUS_REG 0x32
#define UART_IC_STATUS_REG 0x33
#define UBC_MODULE_OC_WARNING_REG 0x34
#define CPLD_EEPROM_STATUS_REG 0x35
#define MTIA_N_OWL_TEST_PIN_STATUS_REG 0x36
#define MTIA_S_OWL_TEST_PIN_STATUS_REG 0x37
#define MTIA_ATH_TEST_PIN_STATUS_REG 0x38
#define MTIA_VQPS_TO_EFUSE_PROGRAMMING_REG 0x39
#define BUFFER_100M_CLK_LOSE_OF_INPUT_SIGNAL_REG 0x3A
#define MTIA_QSPI_BOOT_DISABLE_REG 0x3B
#define ATH_RSVD_GPIO_REG 0x3C

void check_ubc_delayed_timer_handler(struct k_timer *timer);
void check_ubc_delayed(struct k_work *work);
void plat_set_ac_on_log();
void plat_set_dc_on_log(bool is_assert);
void check_cpld_polling_alert_status();
void set_cpld_polling_enable_flag(bool status);
bool get_cpld_polling_enable_flag(void);
void init_cpld_polling(void);
bool is_ubc_enabled_delayed_enabled(void);
void set_ubc_enabled_delayed_enabled(bool status);
void cancel_ubc_delayed_timer_handler(void);

#define TEMPERATURE_IC_OVERT_FAULT_2_REG 0x97
#define ASIC_OC_WARN_2_REG 0x98
#define SYSTEM_ALERT_FAULT_2_REG 0x99
#define VR_SMBUS_ALERT_3_REG 0x9A
#define VR_SMBUS_ALERT_4_REG 0x9B

typedef struct _aegis_cpld_info_ aegis_cpld_info;

typedef struct _aegis_cpld_info_ {
	uint8_t cpld_offset;
	uint8_t dc_off_defaut;
	uint8_t dc_on_defaut;
	bool is_fault_log; // if true, check the value is defaut or not
	uint8_t is_fault_bit_map; //flag for fault

	//flag for 1st polling
	bool is_first_polling;

	//flag for 1st polling after changing DC status
	bool is_first_polling_after_dc_change;

	//temp data for last polling
	uint8_t last_polling_value;

	bool (*status_changed_cb)(aegis_cpld_info *, uint8_t *);

	//bit check mask
	uint8_t bit_check_mask; //bit check mask

	uint8_t event_type;

} aegis_cpld_info;

typedef struct {
	uint8_t cpld_offset;
	const char *reg_name;
	const char *bit_name[8];
} cpld_bit_name_table_t;

enum event_type { VR_POWER_FAULT = 0x01 };

const char *get_cpld_reg_name(uint8_t cpld_offset);
const char *get_cpld_bit_name(uint8_t cpld_offset, uint8_t bit_pos);

#endif
