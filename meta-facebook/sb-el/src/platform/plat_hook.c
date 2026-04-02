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

#include <stdio.h>
#include <string.h>
#include "libutil.h"
#include <logging/log.h>
#include "plat_hook.h"
#include "pmbus.h"
#include "plat_gpio.h"
#include "plat_pldm_sensor.h"
#include "mp2971.h"
#include "mp29816a.h"
#include "raa228249.h"
#include "plat_i2c_target.h"
#include "plat_user_setting.h"
#include "plat_kernel_obj.h"
#include "plat_ioexp.h"
#include "plat_fru.h"
#include "plat_class.h"
#include "tmp431.h"
#include "emc1413.h"

LOG_MODULE_REGISTER(plat_hook);

bool post_sensor_reading_hook_func(uint8_t sensor_number);

static struct k_mutex vr_mutex[VR_MAX_NUM];
bool alert_level_is_assert = false;

#define EEPROM_MAX_WRITE_TIME 5 // the BR24G512 eeprom max write time is 3.5 ms
/* VR */
// clang-format off
vr_pre_proc_arg vr_pre_read_args[] = {
	{ .mutex = vr_mutex + 0, .vr_page = 0x0 },  { .mutex = vr_mutex + 0, .vr_page = 0x1 },
	{ .mutex = vr_mutex + 1, .vr_page = 0x0 },  { .mutex = vr_mutex + 1, .vr_page = 0x1 },
	{ .mutex = vr_mutex + 2, .vr_page = 0x0 },  { .mutex = vr_mutex + 2, .vr_page = 0x1 },
	{ .mutex = vr_mutex + 3, .vr_page = 0x0 },  { .mutex = vr_mutex + 3, .vr_page = 0x1 },
	{ .mutex = vr_mutex + 4, .vr_page = 0x0 },  { .mutex = vr_mutex + 4, .vr_page = 0x1 },
	{ .mutex = vr_mutex + 5, .vr_page = 0x0 },  { .mutex = vr_mutex + 5, .vr_page = 0x1 },
	{ .mutex = vr_mutex + 6, .vr_page = 0x0 },  { .mutex = vr_mutex + 6, .vr_page = 0x1 },
	{ .mutex = vr_mutex + 7, .vr_page = 0x0 },  { .mutex = vr_mutex + 7, .vr_page = 0x1 },
	{ .mutex = vr_mutex + 8, .vr_page = 0x0 },  { .mutex = vr_mutex + 8, .vr_page = 0x1 },
	{ .mutex = vr_mutex + 9, .vr_page = 0x0 },  { .mutex = vr_mutex + 9, .vr_page = 0x1 },
	{ .mutex = vr_mutex + 10, .vr_page = 0x0 }, { .mutex = vr_mutex + 10, .vr_page = 0x1 },
	{ .mutex = vr_mutex + 11, .vr_page = 0x0 }, { .mutex = vr_mutex + 11, .vr_page = 0x1 },
	{ .mutex = vr_mutex + 12, .vr_page = 0x0 }, { .mutex = vr_mutex + 12, .vr_page = 0x1 },
	{ .mutex = vr_mutex + 13, .vr_page = 0x0 }, { .mutex = vr_mutex + 13, .vr_page = 0x1 },
};

mp2971_init_arg mp2971_init_args[] = {
	[0] = { .vout_scale_enable = true },
};

/* the order is following enum VR_RAIL_E */
vr_mapping_sensor vr_rail_table[] = {
	{ VR_RAIL_E_ASIC_P0V75_NUWA0_VDD, SENSOR_NUM_ASIC_P0V75_NUWA0_VDD_VOLT_V,
	  "CB_ASIC_P0V75_NUWA0_VDD", 0xffffffff },

	{ VR_RAIL_E_ASIC_P0V75_NUWA1_VDD, SENSOR_NUM_ASIC_P0V75_NUWA1_VDD_VOLT_V,
	  "CB_ASIC_P0V75_NUWA1_VDD", 0xffffffff },

	{ VR_RAIL_E_ASIC_P0V9_OWL_E_TRVDD, SENSOR_NUM_ASIC_P0V9_OWL_E_TRVDD_VOLT_V,
	  "CB_ASIC_P0V9_OWL_E_TRVDD", 0xffffffff },
	{ VR_RAIL_E_ASIC_P0V75_OWL_E_TRVDD, SENSOR_NUM_ASIC_P0V75_OWL_E_TRVDD_VOLT_V,
	  "CB_ASIC_P0V75_OWL_E_TRVDD", 0xffffffff },

	{ VR_RAIL_E_ASIC_P0V75_MAX_M_VDD, SENSOR_NUM_ASIC_P0V75_MAX_M_VDD_VOLT_V,
	  "CB_ASIC_P0V75_MAX_M_VDD", 0xffffffff },
	{ VR_RAIL_E_ASIC_P0V75_VDDPHY_HBM1357, SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM1357_VOLT_V,
	  "CB_ASIC_P0V75_VDDPHY_HBM1357", 0xffffffff },

	{ VR_RAIL_E_ASIC_P0V75_OWL_E_VDD, SENSOR_NUM_ASIC_P0V75_OWL_E_VDD_VOLT_V,
	  "CB_ASIC_P0V75_OWL_E_VDD", 0xffffffff },
	{ VR_RAIL_E_ASIC_P0V4_VDDQL_HBM1357, SENSOR_NUM_ASIC_P0V4_VDDQL_HBM1357_VOLT_V,
	  "CB_ASIC_P0V4_VDDQL_HBM1357", 0xffffffff },

	{ VR_RAIL_E_ASIC_P1V05_VDDQC_HBM1357, SENSOR_NUM_ASIC_P1V05_VDDQC_HBM1357_VOLT_V,
	  "CB_ASIC_P1V05_VDDQC_HBM1357", 0xffffffff },
	{ VR_RAIL_E_ASIC_P1V8_VPP_HBM1357, SENSOR_NUM_ASIC_P1V8_VPP_HBM1357_VOLT_V,
	  "CB_ASIC_P1V8_VPP_HBM1357", 0xffffffff },

	{ VR_RAIL_E_ASIC_P0V9_VDDQ_HBM1357, SENSOR_NUM_ASIC_P0V9_VDDQ_HBM1357_VOLT_V,
	  "CB_ASIC_P0V9_VDDQ_HBM1357", 0xffffffff },
	{ VR_RAIL_E_ASIC_P0V85_HAMSA_VDD, SENSOR_NUM_ASIC_P0V85_HAMSA_VDD_VOLT_V,
	  "CB_ASIC_P0V85_HAMSA_VDD", 0xffffffff },

	{ VR_RAIL_E_ASIC_P0V75_MAX_N_VDD, SENSOR_NUM_ASIC_P0V75_MAX_N_VDD_VOLT_V,
	  "CB_ASIC_P0V75_MAX_N_VDD", 0xffffffff },
	{ VR_RAIL_E_ASIC_P0V8_HAMSA_AVDD_PCIE, SENSOR_NUM_ASIC_P0V8_HAMSA_AVDD_PCIE_VOLT_V,
	  "CB_ASIC_P0V8_HAMSA_AVDD_PCIE", 0xffffffff },

	{ VR_RAIL_E_ASIC_P0V9_VDDQ_HBM0246, SENSOR_NUM_ASIC_P0V9_VDDQ_HBM0246_VOLT_V,
	  "CB_ASIC_P0V9_VDDQ_HBM0246", 0xffffffff },
	{ VR_RAIL_E_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE, SENSOR_NUM_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_VOLT_V,
	  "CB_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE", 0xffffffff },

	{ VR_RAIL_E_ASIC_P1V05_VDDQC_HBM0246, SENSOR_NUM_ASIC_P1V05_VDDQC_HBM0246_VOLT_V,
	  "CB_ASIC_P1V05_VDDQC_HBM0246", 0xffffffff },
	{ VR_RAIL_E_ASIC_P1V8_VPP_HBM0246, SENSOR_NUM_ASIC_P1V8_VPP_HBM0246_VOLT_V,
	  "CB_ASIC_P1V8_VPP_HBM0246", 0xffffffff },

	{ VR_RAIL_E_ASIC_P0V4_VDDQL_HBM0246, SENSOR_NUM_ASIC_P0V4_VDDQL_HBM0246_VOLT_V,
	  "CB_ASIC_P0V4_VDDQL_HBM0246", 0xffffffff },
	{ VR_RAIL_E_ASIC_P0V75_VDDPHY_HBM0246, SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM0246_VOLT_V,
	  "CB_ASIC_P0V75_VDDPHY_HBM0246", 0xffffffff },

	{ VR_RAIL_E_ASIC_P0V75_OWL_W_VDD, SENSOR_NUM_ASIC_P0V75_OWL_W_VDD_VOLT_V,
	  "CB_ASIC_P0V75_OWL_W_VDD", 0xffffffff },
	{ VR_RAIL_E_ASIC_P0V75_MAX_S_VDD, SENSOR_NUM_ASIC_P0V75_MAX_S_VDD_VOLT_V,
	  "CB_ASIC_P0V75_MAX_S_VDD", 0xffffffff },

	{ VR_RAIL_E_ASIC_P0V9_OWL_W_TRVDD, SENSOR_NUM_ASIC_P0V9_OWL_W_TRVDD_VOLT_V,
	  "CB_ASIC_P0V9_OWL_W_TRVDD", 0xffffffff },
	{ VR_RAIL_E_ASIC_P0V75_OWL_W_TRVDD, SENSOR_NUM_ASIC_P0V75_OWL_W_TRVDD_VOLT_V,
	  "CB_ASIC_P0V75_OWL_W_TRVDD", 0xffffffff },

	{ VR_RAIL_E_P3V3_OSFP_VOLT_V, SENSOR_NUM_P3V3_OSFP_VOLT_V, "P3V3_OSFP_VOLT_V", 0xffffffff },
};

vr_mapping_status vr_status_table[] = {
	{ VR_STAUS_E_STATUS_BYTE, PMBUS_STATUS_BYTE, "STATUS_BYTE" },
	{ VR_STAUS_E_STATUS_WORD, PMBUS_STATUS_WORD, "STATUS_WORD" },
	{ VR_STAUS_E_STATUS_VOUT, PMBUS_STATUS_VOUT, "STATUS_VOUT" },
	{ VR_STAUS_E_STATUS_IOUT, PMBUS_STATUS_IOUT, "STATUS_IOUT" },
	{ VR_STAUS_E_STATUS_INPUT, PMBUS_STATUS_INPUT, "STATUS_INPUT" },
	{ VR_STAUS_E_STATUS_TEMPERATURE, PMBUS_STATUS_TEMPERATURE, "STATUS_TEMPERATURE" },
	{ VR_STAUS_E_STATUS_CML, PMBUS_STATUS_CML, "STATUS_CML_PMBUS" },
};

/* bootstrap */
bootstrap_mapping_register bootstrap_table[] = {
	{ STRAP_INDEX_HAMSA_TEST_STRAP_R, STRAP_TYPE_CPLD, HAMSA_STRAP, "HAMSA_TEST_STRAP_R", 4, 1, 0x0,
	  0x0, false },
	{ STRAP_INDEX_HAMSA_LS_STRAP_0, STRAP_TYPE_CPLD, HAMSA_STRAP, "HAMSA_LS_STRAP_0", 3, 1, 0x01, 0x01,
	  true },
	{ STRAP_INDEX_HAMSA_LS_STRAP_1, STRAP_TYPE_CPLD, HAMSA_STRAP, "HAMSA_LS_STRAP_1", 2, 1, 0x0, 0x0,
	  true },
	{ STRAP_INDEX_HAMSA_CRM_STRAP_0, STRAP_TYPE_CPLD, HAMSA_STRAP, "HAMSA_CRM_STRAP_0", 1, 1, 0x0, 0x0,
	  true },
	{ STRAP_INDEX_HAMSA_CRM_STRAP_1, STRAP_TYPE_CPLD, HAMSA_STRAP, "HAMSA_CRM_STRAP_1", 0, 1, 0x0, 0x0,
	  true },
	{ STRAP_INDEX_HAMSA_MFIO7, STRAP_TYPE_CPLD, HAMSA_MFIO_REG, "HAMSA_MFIO7", 4, 1, 0x01, 0x01, false },
	{ STRAP_INDEX_HAMSA_MFIO9, STRAP_TYPE_CPLD, HAMSA_MFIO_REG, "HAMSA_MFIO9", 3, 1, 0x0, 0x0, false },
	{ STRAP_INDEX_HAMSA_MFIO11, STRAP_TYPE_CPLD, HAMSA_MFIO_REG, "HAMSA_MFIO11", 2, 1, 0x0, 0x0, false },
	{ STRAP_INDEX_HAMSA_MFIO17, STRAP_TYPE_CPLD, HAMSA_MFIO_REG, "HAMSA_MFIO17", 1, 1, 0x0, 0x0, false },
	{ STRAP_INDEX_HAMSA_MFIO18, STRAP_TYPE_CPLD, HAMSA_MFIO_REG, "HAMSA_MFIO18", 0, 1, 0x01, 0x01,
	  false },
	{ STRAP_INDEX_HAMSA_CORE_TAP_CTRL_L, STRAP_TYPE_CPLD, HAMSA_CONTROL_IO, "HAMSA_CORE_TAP_CTRL_L", 3, 1,
	  0x01, 0x01, false },
	{ STRAP_INDEX_HAMSA_TRI_L, STRAP_TYPE_CPLD, HAMSA_CONTROL_IO, "HAMSA_TRI_L", 2, 1, 0x01, 0x1, false },
	{ STRAP_INDEX_HAMSA_ATPG_MODE_L, STRAP_TYPE_CPLD, HAMSA_CONTROL_IO, "HAMSA_ATPG_MODE_L", 1, 1, 0x01,
	  0x01, false },
	{ STRAP_INDEX_HAMSA_DFT_TAP_EN_L, STRAP_TYPE_CPLD, HAMSA_CONTROL_IO, "HAMSA_DFT_TAP_EN_L", 0, 1, 0x01,
	  0x01, false },
	{ STRAP_INDEX_FM_JTAG_HAMSA_JTCE_0_3, STRAP_TYPE_CPLD, HAMSA_JTAG_JTCE, "HAMSA_JTCE_0_3", 0, 4, 0x01,
	  0x01, false },
	{ STRAP_INDEX_NUWA0_TEST_STRAP, STRAP_TYPE_CPLD, NUWA0_STRAP, "NUWA0_TEST_STRAP", 4, 1, 0x0, 0x0,
	  false },
	{ STRAP_INDEX_NUWA0_CRM_STRAP_0, STRAP_TYPE_CPLD, NUWA0_STRAP, "NUWA0_CRM_STRAP_0", 3, 1, 0x0,
	  0x0, true },
	{ STRAP_INDEX_NUWA0_CRM_STRAP_1, STRAP_TYPE_CPLD, NUWA0_STRAP, "NUWA0_CRM_STRAP_1", 2, 1, 0x0,
	  0x0, true },
	{ STRAP_INDEX_NUWA0_CHIP_STRAP_0, STRAP_TYPE_CPLD, NUWA0_STRAP, "NUWA0_CHIP_STRAP_0", 1, 1, 0x01,
	  0x01, true },
	{ STRAP_INDEX_NUWA0_CHIP_STRAP_1, STRAP_TYPE_CPLD, NUWA0_STRAP, "NUWA0_CHIP_STRAP_1", 0, 1, 0x0,
	  0x0, true },
	{ STRAP_INDEX_NUWA0_CORE_TAP_CTRL_PLD_L, STRAP_TYPE_CPLD, NUWA0_CONTROL_IO,
	  "NUWA0_CORE_TAP_CTRL_PLD_L", 3, 1, 0x01, 0x01, false },
	{ STRAP_INDEX_NUWA0_TRI_L, STRAP_TYPE_CPLD, NUWA0_CONTROL_IO, "NUWA0_TRI_L", 2, 1, 0x01, 0x01,
	  false },
	{ STRAP_INDEX_NUWA0_ATPG_MODE_L, STRAP_TYPE_CPLD, NUWA0_CONTROL_IO, "NUWA0_ATPG_MODE_L", 1, 1, 0x01,
	  0x01, false },
	{ STRAP_INDEX_NUWA0_DFT_TAP_EN_PLD_L, STRAP_TYPE_CPLD, NUWA0_CONTROL_IO, "NUWA0_DFT_TAP_EN_PLD_L", 0,
	  1, 0x01, 0x01, false },
	{ STRAP_INDEX_NUWA1_TEST_STRAP, STRAP_TYPE_CPLD, NUWA1_STRAP, "NUWA1_TEST_STRAP", 4, 1, 0x0, 0x0,
	  false },
	{ STRAP_INDEX_NUWA1_CRM_STRAP_0, STRAP_TYPE_CPLD, NUWA1_STRAP, "NUWA1_CRM_STRAP_0", 3, 1, 0x0,
	  0x0, true },
	{ STRAP_INDEX_NUWA1_CRM_STRAP_1, STRAP_TYPE_CPLD, NUWA1_STRAP, "NUWA1_CRM_STRAP_1", 2, 1, 0x0,
	  0x0, true },
	{ STRAP_INDEX_NUWA1_CHIP_STRAP_0, STRAP_TYPE_CPLD, NUWA1_STRAP, "NUWA1_CHIP_STRAP_0", 1, 1, 0x01,
	  0x01, true },
	{ STRAP_INDEX_NUWA1_CHIP_STRAP_1, STRAP_TYPE_CPLD, NUWA1_STRAP, "NUWA1_CHIP_STRAP_1", 0, 1, 0x0,
	  0x0, true },
	{ STRAP_INDEX_NUWA1_CORE_TAP_CTRL_PLD_L, STRAP_TYPE_CPLD, NUWA1_CONTROL_IO,
	  "NUWA1_CORE_TAP_CTRL_PLD_L", 3, 1, 0x01, 0x01, false },
	{ STRAP_INDEX_NUWA1_TRI_L, STRAP_TYPE_CPLD, NUWA1_CONTROL_IO, "NUWA1_TRI_L", 2, 1, 0x01, 0x01,
	  false },
	{ STRAP_INDEX_NUWA1_ATPG_MODE_L, STRAP_TYPE_CPLD, NUWA1_CONTROL_IO, "NUWA1_ATPG_MODE_L", 1, 1, 0x01,
	  0x01, false },
	{ STRAP_INDEX_NUWA1_DFT_TAP_EN_PLD_L, STRAP_TYPE_CPLD, NUWA1_CONTROL_IO, "NUWA1_DFT_TAP_EN_PLD_L", 0,
	  1, 0x01, 0x01, false },
	{ STRAP_INDEX_FM_JTAG_NUWA0_JTCE_0_2, STRAP_TYPE_CPLD, NUWA_JTAG_JTCE, "NUWA0_JTCE_0_2", 3, 3, 0x01,
	  0x01, true },
	{ STRAP_INDEX_FM_JTAG_NUWA1_JTCE_0_2, STRAP_TYPE_CPLD, NUWA_JTAG_JTCE, "NUWA1_JTCE_0_2", 0, 3, 0x01,
	  0x01, true },
	{ STRAP_INDEX_PLD_OWL_E_DFT_TAP_EN_L, STRAP_TYPE_CPLD, OWL_CONTROL_IO, "PLD_OWL_E_DFT_TAP_EN_L", 7, 1,
	  0x01, 0x01, false },
	{ STRAP_INDEX_PLD_OWL_E_CORE_TAP_CTRL_L, STRAP_TYPE_CPLD, OWL_CONTROL_IO, "PLD_OWL_E_CORE_TAP_CTRL_L",
	  6, 1, 0x01, 0x01, false },
	{ STRAP_INDEX_PLD_OWL_E_PAD_TRI_L, STRAP_TYPE_CPLD, OWL_CONTROL_IO, "PLD_OWL_E_PAD_TRI_L", 5, 1, 0x01,
	  0x01, false },
	{ STRAP_INDEX_PLD_OWL_E_ATPG_MODE_L, STRAP_TYPE_CPLD, OWL_CONTROL_IO, "PLD_OWL_E_ATPG_MODE_L", 4, 1,
	  0x01, 0x01, false },
	{ STRAP_INDEX_PLD_OWL_W_DFT_TAP_EN_L, STRAP_TYPE_CPLD, OWL_CONTROL_IO, "PLD_OWL_W_DFT_TAP_EN_L", 3, 1,
	  0x01, 0x01, false },
	{ STRAP_INDEX_PLD_OWL_W_CORE_TAP_CTRL_L, STRAP_TYPE_CPLD, OWL_CONTROL_IO, "PLD_OWL_W_CORE_TAP_CTRL_L",
	  2, 1, 0x01, 0x01, false },
	{ STRAP_INDEX_PLD_OWL_W_PAD_TRI_L, STRAP_TYPE_CPLD, OWL_CONTROL_IO, "PLD_OWL_W_PAD_TRI_L", 1, 1, 0x01,
	  0x01, false },
	{ STRAP_INDEX_PLD_OWL_W_ATPG_MODE_L, STRAP_TYPE_CPLD, OWL_CONTROL_IO, "PLD_OWL_W_ATPG_MODE_L", 0, 1,
	  0x01, 0x01, false },
	{ STRAP_INDEX_OWL_E_JTAG_MUX_PLD_SEL_0_3, STRAP_TYPE_CPLD, OWL_JTAG_SEL,
	  "OWL_E_JTAG_MUX_PLD_SEL_0_3", 4, 4, 0x0, 0x0, true },
	{ STRAP_INDEX_OWL_W_JTAG_MUX_PLD_SEL_0_3, STRAP_TYPE_CPLD, OWL_JTAG_SEL,
	  "OWL_W_JTAG_MUX_PLD_SEL_0_3", 0, 4, 0x0, 0x0, true },
	{ STRAP_INDEX_OWL_E_UART_MUX_PLD_SEL_0_2, STRAP_TYPE_CPLD, OWL_UART_SEL,
	  "OWL_E_UART_MUX_PLD_SEL_0_2", 3, 3, 0x0, 0x0, true },
	{ STRAP_INDEX_OWL_W_UART_MUX_PLD_SEL_0_2, STRAP_TYPE_CPLD, OWL_UART_SEL,
	  "OWL_W_UART_MUX_PLD_SEL_0_2", 0, 3, 0x0, 0x0, true },
	{ STRAP_INDEX_OWL_E_DVT_ENABLE, STRAP_TYPE_CPLD, OWL_DVT_ENABLE, "OWL_E_DVT_ENABLE", 1, 1, 0x0, 0x0,
	  false },
	{ STRAP_INDEX_OWL_W_DVT_ENABLE, STRAP_TYPE_CPLD, OWL_DVT_ENABLE, "OWL_W_DVT_ENABLE", 0, 1, 0x0, 0x0,
	  false },
	{ STRAP_INDEX_OWL_E_BOOT_SOURCE_0_7, STRAP_TYPE_IOEXP_PCA6416A, PCA6414A_OUTPUT_PORT_0,
	  "OWL_E_BOOT_SOURCE_0_7", 0, 8, 0x0, 0x0, false },
	{ STRAP_INDEX_OWL_W_BOOT_SOURCE_0_7, STRAP_TYPE_IOEXP_PCA6416A, PCA6414A_OUTPUT_PORT_1,
	  "OWL_W_BOOT_SOURCE_0_7", 0, 8, 0x0, 0x0, false },
	{ STRAP_INDEX_HAMSA_MFIO6, STRAP_TYPE_IOEXP_TCA6424A, TCA6424A_OUTPUT_PORT_1, "HAMSA_MFIO6",
	  HAMSA_MFIO6_BIT, 1, 0x0, 0x0, false },
	{ STRAP_INDEX_HAMSA_MFIO8, STRAP_TYPE_IOEXP_TCA6424A, TCA6424A_OUTPUT_PORT_1, "HAMSA_MFIO8",
	  HAMSA_MFIO8_BIT, 1, 0x0, 0x0, false },
	{ STRAP_INDEX_HAMSA_MFIO10, STRAP_TYPE_IOEXP_TCA6424A, TCA6424A_OUTPUT_PORT_2,
	  "HAMSA_MFIO10", 0, 1, 0x0, 0x0, false },
	{ STRAP_INDEX_NUWA0_MFIO6, STRAP_TYPE_IOEXP_TCAL6408R, TCAL6408R_OUTPUT_PORT_0,
	  "NUWA0_MFIO6", NUWA0_MFIO6_BIT, 1, 0x0, 0x0, false },
	{ STRAP_INDEX_NUWA0_MFIO8, STRAP_TYPE_IOEXP_TCAL6408R, TCAL6408R_OUTPUT_PORT_0,
	  "NUWA0_MFIO8", NUWA0_MFIO8_BIT, 1, 0x0, 0x0, false },
	{ STRAP_INDEX_NUWA0_MFIO10, STRAP_TYPE_IOEXP_TCAL6408R, TCAL6408R_OUTPUT_PORT_0,
	  "NUWA0_MFIO10", NUWA0_MFIO10_BIT, 1, 0x0, 0x0, false },
	{ STRAP_INDEX_NUWA1_MFIO6, STRAP_TYPE_IOEXP_TCAL6408R, TCAL6408R_OUTPUT_PORT_0,
	  "NUWA1_MFIO6", NUWA1_MFIO6_BIT, 1, 0x0, 0x0, false },
	{ STRAP_INDEX_NUWA1_MFIO8, STRAP_TYPE_IOEXP_TCAL6408R, TCAL6408R_OUTPUT_PORT_0,
	  "NUWA1_MFIO8", NUWA1_MFIO8_BIT, 1, 0x0, 0x0, false },
	{ STRAP_INDEX_NUWA1_MFIO10, STRAP_TYPE_IOEXP_TCAL6408R, TCAL6408R_OUTPUT_PORT_0,
	  "NUWA1_MFIO10", NUWA1_MFIO10_BIT, 1, 0x0, 0x0, false },
};
// clang-format on

void *vr_mutex_get(enum VR_INDEX_E vr_index)
{
	if (vr_index >= VR_INDEX_MAX) {
		LOG_ERR("vr_mutex_get, invalid vr_index %d", vr_index);
		return NULL;
	}

	return vr_mutex + vr_index;
}

bool pre_vr_read(sensor_cfg *cfg, void *args)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(args, false);

	vr_pre_proc_arg *pre_proc_args = (vr_pre_proc_arg *)args;
	uint8_t retry = 5;
	I2C_MSG msg;

	/* mutex lock */
	if (pre_proc_args->mutex) {
		LOG_DBG("%x l %p", cfg->num, pre_proc_args->mutex);
		if (k_mutex_lock(pre_proc_args->mutex, K_MSEC(VR_MUTEX_LOCK_TIMEOUT_MS))) {
			LOG_ERR("0x%02x pre_vr_read, mutex lock fail", cfg->num);
			return false;
		}
	}

	/* set page */
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 2;
	msg.data[0] = 0x00;
	msg.data[1] = pre_proc_args->vr_page;
	if (i2c_master_write(&msg, retry)) {
		k_mutex_unlock(pre_proc_args->mutex);
		LOG_ERR("0x%02x pre_vr_read, set page fail", cfg->num);
		return false;
	}
	return true;
}

bool post_vr_read(sensor_cfg *cfg, void *args, int *const reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(args, false);

	// Voltage peak
	for (int i = 0; i < VR_RAIL_E_MAX; i++) {
		if (reading == NULL) {
			break;
		}

		if (((get_asic_board_id() != ASIC_BOARD_ID_EVB)) &&
		    (i == VR_RAIL_E_P3V3_OSFP_VOLT_V))
			continue; // skip osfp p3v3 on AEGIS BD

		if (cfg->num == vr_rail_table[i].sensor_id) {
			if (vr_rail_table[i].peak_value == 0xffffffff) {
				vr_rail_table[i].peak_value = *reading;
			} else {
				if (vr_rail_table[i].peak_value < *reading) {
					vr_rail_table[i].peak_value = *reading;
				}
			}
			break;
		}
	}

	vr_pre_proc_arg *pre_proc_args = (vr_pre_proc_arg *)args;

	/* mutex unlock */
	if (pre_proc_args->mutex) {
		LOG_DBG("%x u %p", cfg->num, pre_proc_args->mutex);
		if (k_mutex_unlock(pre_proc_args->mutex)) {
			LOG_ERR("0x%02x post_vr_read, mutex unlock fail", cfg->num);
			return false;
		}
	}

	/* set reading val to 0 if reading val is negative */
	sensor_val tmp_reading;
	tmp_reading.integer = (int16_t)(*reading & 0xFFFF);
	tmp_reading.fraction = (int16_t)((*reading >> 16) & 0xFFFF);

	/* sensor_value = 1000 times of true value */
	int32_t sensor_value = tmp_reading.integer * 1000 + tmp_reading.fraction;

	if (sensor_value < 0) {
		LOG_DBG("Original sensor reading: integer = %d, fraction = %d (combined value * 1000: %d)",
			tmp_reading.integer, tmp_reading.fraction, sensor_value);
		*reading = 0;
		LOG_DBG("Negative sensor reading detected. Set reading to 0x%x", *reading);
	}
	// post_sensor_reading_hook_func(cfg->num);

	return true;
}

uint8_t emc1413_cache_status_0 = 0;
uint8_t emc1413_cache_status_1 = 0;
uint8_t emc1413_cache_status_2 = 0;
uint8_t emc1413_cache_status_3 = 0;

bool emc1413_check_open_status(sensor_cfg *cfg, uint8_t status)
{
	uint8_t bit = (cfg->offset == EMC1413_REMOTE_TEMPERATRUE_1) ? BIT(1) :
		      (cfg->offset == EMC1413_REMOTE_TEMPERATRUE_2) ? BIT(2) :
								      0;
	// only check BIT(1), BIT(2)
	if (status & bit) {
		cfg->cache_status = SENSOR_OPEN_CIRCUIT;
		return false;
	}
	return true;
}

uint8_t get_emc1413_cache_status(uint8_t idx)
{
	const uint8_t cache_status[4] = { emc1413_cache_status_0, emc1413_cache_status_1,
					  emc1413_cache_status_2, emc1413_cache_status_3 };
	if (idx > 3) {
		LOG_ERR("Invalid emc1413 cache status index %u", idx);
		return 0;
	}
	return cache_status[idx];
}

bool post_tmp_read(sensor_cfg *cfg, void *args, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	ARG_UNUSED(args);
	ARG_UNUSED(reading);
	uint8_t bit = 0;

	if (cfg->type == sensor_dev_tmp431) {
		uint8_t status = 0;
		if (tmp432_get_temp_open_status(cfg, &status)) {
			bit = (cfg->offset == TMP432_REMOTE_TEMPERATRUE_1) ? BIT(1) :
			      (cfg->offset == TMP432_REMOTE_TEMPERATRUE_2) ? BIT(2) :
									     0;
			// only check BIT(1), BIT(2)
			if (status & bit) {
				cfg->cache_status = SENSOR_OPEN_CIRCUIT;
				return false;
			}
		}
	} else if (cfg->type == sensor_dev_emc1413) {
		switch (cfg->num) {
		/*
		SENSOR_NUM_ASIC_MEDHA0_SENSOR0_TEMP_C
		SENSOR_NUM_ASIC_MEDHA0_SENSOR1_TEMP_C
		SENSOR_NUM_ASIC_OWL_W_TEMP_C
		SENSOR_NUM_ASIC_OWL_E_TEMP_C
		SENSOR_NUM_ASIC_MEDHA1_SENSOR0_TEMP_C
		SENSOR_NUM_ASIC_MEDHA1_SENSOR1_TEMP_C
		SENSOR_NUM_ASIC_HAMSA_CRM_TEMP_C
		SENSOR_NUM_ASIC_HAMSA_LS_TEMP_C
		*/
		case SENSOR_NUM_ASIC_NUWA0_SENSOR0_TEMP_C:
			if (emc1413_get_temp_open_status(cfg, &emc1413_cache_status_0)) {
				// update the open status to cache
				if (!emc1413_check_open_status(cfg, emc1413_cache_status_0))
					return false;
			}
			break;
		case SENSOR_NUM_ASIC_NUWA0_SENSOR1_TEMP_C:
			if (!emc1413_check_open_status(cfg, emc1413_cache_status_0))
				return false;
			break;
		case SENSOR_NUM_ASIC_OWL_W_TEMP_C:
			if (emc1413_get_temp_open_status(cfg, &emc1413_cache_status_1)) {
				// update the open status to cache
				if (!emc1413_check_open_status(cfg, emc1413_cache_status_1))
					return false;
			}
			break;
		case SENSOR_NUM_ASIC_OWL_E_TEMP_C:
			if (!emc1413_check_open_status(cfg, emc1413_cache_status_1))
				return false;
			break;
		case SENSOR_NUM_ASIC_NUWA1_SENSOR0_TEMP_C:
			if (emc1413_get_temp_open_status(cfg, &emc1413_cache_status_2)) {
				// update the open status to cache
				if (!emc1413_check_open_status(cfg, emc1413_cache_status_2))
					return false;
			}
			break;
		case SENSOR_NUM_ASIC_NUWA1_SENSOR1_TEMP_C:
			if (!emc1413_check_open_status(cfg, emc1413_cache_status_2))
				return false;
			break;
		case SENSOR_NUM_ASIC_HAMSA_CRM_TEMP_C:
			if (emc1413_get_temp_open_status(cfg, &emc1413_cache_status_3)) {
				// update the open status to cache
				if (!emc1413_check_open_status(cfg, emc1413_cache_status_3))
					return false;
			}
			break;
		case SENSOR_NUM_ASIC_HAMSA_LS_TEMP_C:
			if (!emc1413_check_open_status(cfg, emc1413_cache_status_3))
				return false;
			break;
		default:
			break;
		}
	} else {
		LOG_ERR("Unsupported sensor type 0x%x for post_tmp_read", cfg->type);
		return false;
	}

	return post_common_sensor_read(cfg, args, reading);
}

bool plat_get_alert_level_is_assert(void)
{
	return alert_level_is_assert;
}

bool post_ubc_read(sensor_cfg *cfg, void *args, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	ARG_UNUSED(args);

	// post reading value update
	uint8_t sensor_number = cfg->num;
	if (!post_sensor_reading_hook_func(sensor_number))
		LOG_ERR("pldm update sensor value failed");

	const sensor_val *reading_val = (sensor_val *)reading;

	int sensor_reading = 0;
	static int ubc1_current_mA;

	if (cfg->num == SENSOR_NUM_UBC1_P12V_CURR_A) {
		if (reading != NULL) {
			if (reading_val->integer >= 0) {
				sensor_reading =
					reading_val->integer * 1000 + reading_val->fraction;
			} else {
				sensor_reading =
					reading_val->integer * 1000 - reading_val->fraction;
			}

			ubc1_current_mA = sensor_reading;
		} else {
			ubc1_current_mA = 0;
		}
	}

	if (cfg->num == SENSOR_NUM_UBC2_P12V_CURR_A) {
		if (reading != NULL) {
			if (reading_val->integer >= 0) {
				sensor_reading =
					reading_val->integer * 1000 + reading_val->fraction;
			} else {
				sensor_reading =
					reading_val->integer * 1000 - reading_val->fraction;
			}
		} else {
			sensor_reading = 0;
		}

		if (pwr_level_mutex_lock(K_MSEC(1000)) != 0) {
			LOG_ERR("failed to lock pwrlevel mutex");
			return false;
		}

		if ((ubc1_current_mA + sensor_reading) > plat_get_alert_level_mA_user_setting()) {
			if (alert_level_is_assert == false) {
				alert_level_is_assert = true;
				power_level_send_event(alert_level_is_assert, ubc1_current_mA,
						       sensor_reading);
			}
		} else {
			if (alert_level_is_assert == true) {
				alert_level_is_assert = false;
				power_level_send_event(alert_level_is_assert, ubc1_current_mA,
						       sensor_reading);
			}
		}

		pwr_level_mutex_unlock();
	}

	return true;
}

bool is_mb_dc_on()
{
	/* RST_ARKE_PWR_ON_PLD_R1_N is low active,
   * 1 -> power on
   * 0 -> power off
   */
	return gpio_get(RST_ARKE_PWR_ON_PLD_R1_N);
}

void vr_mutex_init(void)
{
	for (uint8_t i = 0; i < ARRAY_SIZE(vr_mutex); i++) {
		k_mutex_init(vr_mutex + i);
		LOG_DBG("vr_mutex[%d] %p init", i, vr_mutex + i);
	}
}

bool vr_rail_name_get(uint8_t rail, uint8_t **name)
{
	CHECK_NULL_ARG_WITH_RETURN(name, false);

	if (rail >= VR_RAIL_E_MAX) {
		*name = NULL;
		return false;
	}

	*name = (uint8_t *)vr_rail_table[rail].sensor_name;
	return true;
}

bool vr_status_name_get(uint8_t rail, uint8_t **name)
{
	CHECK_NULL_ARG_WITH_RETURN(name, false);

	if (rail >= VR_STAUS_E_MAX) {
		*name = NULL;
		return false;
	}

	*name = (uint8_t *)vr_status_table[rail].vr_status_name;
	return true;
}

bool vr_rail_enum_get(uint8_t *name, uint8_t *num)
{
	CHECK_NULL_ARG_WITH_RETURN(name, false);
	CHECK_NULL_ARG_WITH_RETURN(num, false);

	for (int i = 0; i < VR_RAIL_E_MAX; i++) {
		if (strcmp(name, vr_rail_table[i].sensor_name) == 0) {
			*num = i;
			return true;
		}
	}

	LOG_ERR("invalid rail name %s", name);
	return false;
}

bool vr_status_enum_get(uint8_t *name, uint8_t *num)
{
	CHECK_NULL_ARG_WITH_RETURN(name, false);
	CHECK_NULL_ARG_WITH_RETURN(num, false);

	for (int i = 0; i < VR_STAUS_E_MAX; i++) {
		if (strcmp(name, vr_status_table[i].vr_status_name) == 0) {
			*num = i;
			return true;
		}
	}

	LOG_ERR("invalid vr status name %s", name);
	return false;
}

bool plat_get_vr_status(uint8_t rail, uint8_t vr_status_rail, uint16_t *vr_status)
{
	CHECK_NULL_ARG_WITH_RETURN(vr_status, false);

	if (rail >= VR_RAIL_E_MAX) {
		LOG_ERR("invalid rail %u", rail);
		return false;
	}

	if (vr_status_rail >= VR_STAUS_E_MAX) {
		LOG_ERR("invalid vr_status_rail %u", vr_status_rail);
		return false;
	}

	bool ret = false;
	uint8_t sensor_id = vr_rail_table[rail].sensor_id;
	sensor_cfg *cfg = get_sensor_cfg_by_sensor_id(sensor_id);
	CHECK_NULL_ARG_WITH_RETURN(cfg, ret);

	if ((cfg->pre_sensor_read_hook)) {
		if ((cfg->pre_sensor_read_hook)(cfg, cfg->pre_sensor_read_args) == false) {
			LOG_DBG("%d read vr status pre hook fail!", sensor_id);
			return false;
		}
	};

	vr_pre_proc_arg *pre_proc_args = (vr_pre_proc_arg *)cfg->pre_sensor_read_args;

	uint16_t pmbus_reg_id = vr_status_table[vr_status_rail].pmbus_reg;

	switch (cfg->type) {
	case sensor_dev_mp2971:
		if (!mp2971_get_vr_status(cfg, pre_proc_args->vr_page, pmbus_reg_id, vr_status)) {
			LOG_ERR("The VR MPS2971 vr status reading failed");
			goto err;
		}
		break;
	case sensor_dev_mp29816a:
		if (!mp29816a_get_vr_status(cfg, pre_proc_args->vr_page, pmbus_reg_id, vr_status)) {
			LOG_ERR("The VR MPS29816a vr status reading failed");
			goto err;
		}
		break;
	case sensor_dev_raa228249:
		if (!raa228249_get_vr_status(cfg, pre_proc_args->vr_page, pmbus_reg_id,
					     vr_status)) {
			LOG_ERR("The VR RAA228249 vr status reading failed");
			goto err;
		}
		break;
	default:
		LOG_ERR("Unsupport VR type(%x)", cfg->type);
		goto err;
	}

	ret = true;
err:
	if (cfg->post_sensor_read_hook) {
		if (cfg->post_sensor_read_hook(cfg, cfg->post_sensor_read_args, NULL) == false) {
			LOG_ERR("%d read vr status post hook fail!", sensor_id);
		}
	}
	return ret;
}

bool plat_clear_vr_status(uint8_t rail)
{
	if (rail >= VR_RAIL_E_MAX) {
		LOG_ERR("invalid rail %u", rail);
		return false;
	}

	bool ret = false;
	uint8_t sensor_id = vr_rail_table[rail].sensor_id;
	sensor_cfg *cfg = get_sensor_cfg_by_sensor_id(sensor_id);

	if (cfg == NULL) {
		LOG_ERR("Failed to get sensor config for sensor 0x%x", sensor_id);
		return false;
	}

	vr_pre_proc_arg *pre_proc_args = (vr_pre_proc_arg *)cfg->pre_sensor_read_args;

	if (cfg->pre_sensor_read_hook) {
		if (!cfg->pre_sensor_read_hook(cfg, cfg->pre_sensor_read_args)) {
			LOG_ERR("%d clear vr status pre hook fail!", sensor_id);
			goto err;
		}
	}

	switch (cfg->type) {
	case sensor_dev_mp2971:
		if (!mp2971_clear_vr_status(cfg, pre_proc_args->vr_page)) {
			LOG_ERR("The VR MPS2971 vr status clear failed");
			goto err;
		}
		break;
	case sensor_dev_mp29816a:
		if (!mp29816a_clear_vr_status(cfg, pre_proc_args->vr_page)) {
			LOG_ERR("The VR MPS29816a vr status clear failed");
			goto err;
		}
		break;
	case sensor_dev_raa228249:
		if (!raa228249_clear_vr_status(cfg, pre_proc_args->vr_page)) {
			LOG_ERR("The VR RAA228249 vr status clear failed");
			goto err;
		}
		break;
	default:
		LOG_ERR("Unsupport VR type(%x)", cfg->type);
		goto err;
	}

	ret = true;
err:
	if (cfg->post_sensor_read_hook) {
		if (cfg->post_sensor_read_hook(cfg, cfg->post_sensor_read_args, NULL) == false) {
			LOG_ERR("%d clear vr status post hook fail!", sensor_id);
		}
	}
	return ret;
}

bool plat_get_vout_command(uint8_t rail, uint16_t *millivolt)
{
	CHECK_NULL_ARG_WITH_RETURN(millivolt, false);

	if (rail >= VR_RAIL_E_MAX) {
		LOG_ERR("invalid rail %u", rail);
		return false;
	}

	bool ret = false;
	uint8_t sensor_id = vr_rail_table[rail].sensor_id;
	sensor_cfg *cfg = get_sensor_cfg_by_sensor_id(sensor_id);

	if (cfg == NULL) {
		LOG_ERR("Failed to get sensor config for sensor 0x%x", sensor_id);
		return false;
	}

	vr_pre_proc_arg *pre_proc_args = vr_pre_read_args + rail;

	if (cfg->pre_sensor_read_hook) {
		if (!cfg->pre_sensor_read_hook(cfg, cfg->pre_sensor_read_args)) {
			LOG_ERR("sensor id: 0x%x pre-read fail", sensor_id);
			goto err;
		}
	}

	switch (cfg->type) {
	case sensor_dev_mp2971:
		if (!mp2971_get_vout_command(cfg, pre_proc_args->vr_page, millivolt)) {
			LOG_ERR("The VR MPS2971 vout reading failed");
			goto err;
		}
		break;
	case sensor_dev_mp29816a:
		if (!mp29816a_get_vout_command(cfg, pre_proc_args->vr_page, millivolt)) {
			LOG_ERR("The VR MPS29816a vout reading failed");
			goto err;
		}
		break;
	case sensor_dev_raa228249:
		if (!raa228249_get_vout_command(cfg, pre_proc_args->vr_page, millivolt)) {
			LOG_ERR("The VR RAA228249 vout reading failed");
			goto err;
		}
		break;
	default:
		LOG_ERR("Unsupport VR type(%x)", cfg->type);
		goto err;
	}

	ret = true;
err:
	if (cfg->post_sensor_read_hook) {
		if (cfg->post_sensor_read_hook(cfg, cfg->post_sensor_read_args, NULL) == false) {
			LOG_ERR("sensor id: 0x%x post-read fail", sensor_id);
		}
	}
	return ret;
}

bool post_sensor_reading_hook_func(uint8_t sensor_number)
{
	update_sensor_reading_by_sensor_number(sensor_number);
	return true;
}

bool post_common_sensor_read(sensor_cfg *cfg, void *args, int *const reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	ARG_UNUSED(args);

	uint8_t sensor_number = cfg->num;
	if (!post_sensor_reading_hook_func(sensor_number))
		return false;
	return true;
}

// struct vr_vout_user_settings voltage_command_get = { 0 };
vr_vout_range_user_settings_struct vout_range_user_settings = { 0 };

bool plat_get_vout_range(uint8_t rail, uint16_t *vout_max_millivolt, uint16_t *vout_min_millivolt)
{
	CHECK_NULL_ARG_WITH_RETURN(vout_max_millivolt, false);
	CHECK_NULL_ARG_WITH_RETURN(vout_min_millivolt, false);

	uint8_t sensor_id = vr_rail_table[rail].sensor_id;
	PDR_numeric_sensor *pdr_sensor = get_pdr_numeric_sensor_by_sensor_id(sensor_id);
	CHECK_NULL_ARG_WITH_RETURN(pdr_sensor, false);

	*vout_max_millivolt = (uint16_t)pdr_sensor->critical_high;
	*vout_min_millivolt = (uint16_t)pdr_sensor->critical_low;

	return true;
}

bool vr_rail_voltage_peak_get(uint8_t *name, int *peak_value)
{
	CHECK_NULL_ARG_WITH_RETURN(name, false);
	CHECK_NULL_ARG_WITH_RETURN(peak_value, false);

	for (int i = 0; i < VR_RAIL_E_MAX; i++) {
		if (strcmp(name, vr_rail_table[i].sensor_name) == 0) {
			*peak_value = vr_rail_table[i].peak_value;
			return true;
		}
	}

	return false;
}

bool vr_rail_voltage_peak_clear(uint8_t rail_index)
{
	if (rail_index >= VR_RAIL_E_MAX) {
		return false;
	}

	vr_rail_table[rail_index].peak_value = 0xffffffff;

	return true;
}

bool vr_vout_range_user_settings_init(void)
{
	for (int i = 0; i < VR_RAIL_E_MAX; i++) {
		if ((get_asic_board_id() != ASIC_BOARD_ID_EVB) &&
		    (i == VR_RAIL_E_P3V3_OSFP_VOLT_V)) {
			vout_range_user_settings.default_vout_max[i] = 0xffff;
			vout_range_user_settings.default_vout_min[i] = 0xffff;
			vout_range_user_settings.change_vout_max[i] = 0xffff;
			vout_range_user_settings.change_vout_min[i] = 0xffff;
			continue; // skip osfp p3v3 on AEGIS BD
		}
		uint16_t vout_max = 0;
		uint16_t vout_min = 0;
		if (!plat_get_vout_range(i, &vout_max, &vout_min)) {
			LOG_ERR("Can't find vout range default by rail index: %d", i);
			return false;
		}
		vout_range_user_settings.default_vout_max[i] = vout_max;
		vout_range_user_settings.default_vout_min[i] = vout_min;
		vout_range_user_settings.change_vout_max[i] = vout_max;
		vout_range_user_settings.change_vout_min[i] = vout_min;
	}

	return true;
}

uint8_t get_strap_index_max()
{
	return (get_asic_board_id() == ASIC_BOARD_ID_EVB) ? STRAP_INDEX_MAX :
							    STRAP_INDEX_EXCEPT_EVB_MAX;
}

static uint8_t reverse_bits(uint8_t byte, uint8_t bit_cnt)
{
	uint8_t reversed_byte = 0;
	for (int i = 0; i < bit_cnt; i++) {
		if (byte & 1)
			reversed_byte = (reversed_byte << 1) + 1;
		else
			reversed_byte = reversed_byte << 1;

		byte = byte >> 1;
	}
	return reversed_byte;
}

bool set_ioexp_val_to_bootstrap_table(void)
{
	uint8_t data[2] = { 0x00, 0x00 };
	uint8_t direction[2] = { 0x00, 0x00 };
	if (is_mb_dc_on()) {
		if (!pca6416a_i2c_read(PCA6414A_OUTPUT_PORT_0, data, 2)) {
			LOG_ERR("Can't find bootstrap value from pca6416a");
			return false;
		}
		if (!pca6416a_i2c_read(PCA6414A_CONFIG_0, direction, 2)) {
			LOG_ERR("Can't find bootstrap direction from pca6416a");
			return false;
		}
		// set when output only
		bootstrap_table[STRAP_INDEX_OWL_E_BOOT_SOURCE_0_7].change_setting_value =
			(data[0] & (~direction[0]));
		bootstrap_table[STRAP_INDEX_OWL_W_BOOT_SOURCE_0_7].change_setting_value =
			(data[1] & (~direction[1]));
	}

	if (is_evb_ioe_accessible()) {
		if (!tca6424a_i2c_read(TCA6424A_OUTPUT_PORT_1, data, 2)) {
			LOG_ERR("Can't find bootstrap value from tca6424a");
			return false;
		}
		if (!tca6424a_i2c_read(TCA6424A_CONFIG_1, direction, 2)) {
			LOG_ERR("Can't find bootstrap direction from tca6424a");
			return false;
		}
		for (uint8_t i = STRAP_INDEX_HAMSA_MFIO6; i <= STRAP_INDEX_NUWA1_MFIO10; i++) {
			// check data from port1 or port2
			uint8_t dir = (bootstrap_table[i].cpld_offsets == TCA6424A_OUTPUT_PORT_1) ?
					      direction[0] :
					      direction[1];
			uint8_t tmp = (bootstrap_table[i].cpld_offsets == TCA6424A_OUTPUT_PORT_1) ?
					      data[0] :
					      data[1];
			// set when output only
			if ((dir >> bootstrap_table[i].bit_offset) & 0x01) {
				continue;
			}
			bootstrap_table[i].change_setting_value =
				(tmp >> bootstrap_table[i].bit_offset) & 0x01;
		}
	}

	return true;
}

bool bootstrap_default_settings_init(void)
{
	// read cpld value and write to bootstrap_table
	for (int i = 0; i <= STRAP_INDEX_OWL_W_DVT_ENABLE; i++) {
		uint8_t data = 0;
		if (!plat_read_cpld(bootstrap_table[i].cpld_offsets, &data, 1)) {
			LOG_ERR("Can't find bootstrap default by rail index from cpld: %d", i);
			return false;
		}
		uint8_t mask =
			GENMASK(bootstrap_table[i].bit_offset + bootstrap_table[i].bit_count - 1,
				bootstrap_table[i].bit_offset);
		bootstrap_table[i].change_setting_value =
			(data & mask) >> bootstrap_table[i].bit_offset;
		if (bootstrap_table[i].reverse)
			bootstrap_table[i].change_setting_value =
				reverse_bits(bootstrap_table[i].change_setting_value,
					     bootstrap_table[i].bit_count);
	}

	// read io-exp value and write to bootstrap_table
	return set_ioexp_val_to_bootstrap_table();
}

static inline uint8_t get_val_from_strap_index(uint8_t strap_index)
{
	return ((bootstrap_table[strap_index].change_setting_value & 0x01)
		<< bootstrap_table[strap_index].bit_offset);
}

bool set_bootstrap_table_val_to_ioexp(void)
{
	uint8_t data[2] = { 0 };

	/* 1) PCA6416A: OWL boot source */
	data[0] = bootstrap_table[STRAP_INDEX_OWL_E_BOOT_SOURCE_0_7].change_setting_value;
	data[1] = bootstrap_table[STRAP_INDEX_OWL_W_BOOT_SOURCE_0_7].change_setting_value;
	if (!pca6416a_i2c_write(PCA6414A_OUTPUT_PORT_0, data, 2)) {
		LOG_ERR("Can't set pca6416a from bootstrap_table");
		return false;
	}

	/* 2) EVB-only IO expanders */
	if (is_evb_ioe_accessible()) {
		/* 2-1) HAMSA straps -> TCA6424A (legacy) */
		uint8_t port1_data = 0;
		uint8_t port2_data = 0;

		/* Only HAMSA_MFIO6/8/10 stay on TCA6424A */
		for (uint8_t i = STRAP_INDEX_HAMSA_MFIO6; i <= STRAP_INDEX_HAMSA_MFIO10; i++) {
			if (bootstrap_table[i].cpld_offsets == TCA6424A_OUTPUT_PORT_1) {
				port1_data |= get_val_from_strap_index(i);
			} else {
				port2_data |= get_val_from_strap_index(i);
			}
		}

		data[0] = port1_data;
		data[1] = port2_data;
		if (!tca6424a_i2c_write(TCA6424A_OUTPUT_PORT_1, data, 2)) {
			LOG_ERR("Can't set tca6424a (HAMSA) from bootstrap_table");
			return false;
		}

		uint8_t nuwa_out = 0;
		/* Only NUWA_MFIO6/8/10 stay on TCAL6408R */
		for (uint8_t i = STRAP_INDEX_NUWA0_MFIO6; i <= STRAP_INDEX_NUWA1_MFIO10; i++) {
			nuwa_out |= get_val_from_strap_index(i);
		}

		if (!tcal6408r_i2c_write(TCAL6408R_OUTPUT_PORT_0, &nuwa_out, 1)) {
			LOG_ERR("Can't set tcal6408r (NUWA) from bootstrap_table");
			return false;
		}
	}

	return true;
}

bootstrap_user_settings_struct bootstrap_user_settings = { 0 };

bool bootstrap_user_settings_get(void *bootstrap_user_settings)
{
	CHECK_NULL_ARG_WITH_RETURN(bootstrap_user_settings, false);

	/* read the bootstrap_user_settings from eeprom */
	if (!plat_eeprom_read(BOOTSTRAP_USER_SETTINGS_OFFSET, bootstrap_user_settings,
			      sizeof(struct bootstrap_user_settings_struct))) {
		LOG_ERR("Failed to read eeprom when get bootstrap_user_settings");
		return false;
	}

	return true;
}

bool bootstrap_user_settings_set(void *bootstrap_user_settings)
{
	CHECK_NULL_ARG_WITH_RETURN(bootstrap_user_settings, false);

	/* write the bootstrap_user_settings to eeprom */
	if (!plat_eeprom_write(BOOTSTRAP_USER_SETTINGS_OFFSET, bootstrap_user_settings,
			       sizeof(struct bootstrap_user_settings_struct))) {
		LOG_ERR("bootstrap Failed to write eeprom");
		return false;
	}
	k_msleep(EEPROM_MAX_WRITE_TIME);

	return true;
}
bool temp_threshold_user_settings_get(void *temp_threshold_user_settings)
{
	CHECK_NULL_ARG_WITH_RETURN(temp_threshold_user_settings, false);

	/* TODO: read the temp_threshold_user_settings from eeprom */
	if (!plat_eeprom_read(TEMP_THRESHOLD_USER_SETTINGS_OFFSET, temp_threshold_user_settings,
			      sizeof(struct temp_threshold_user_settings_struct))) {
		LOG_ERR("Failed to read eeprom when get temp_threshold_user_settings");
		return false;
	}

	return true;
}

bool check_is_bootstrap_setting_value_valid(uint8_t rail, uint8_t value)
{
	int critical_value = 1 << bootstrap_table[rail].bit_count;

	return value < critical_value;
}

bool find_bootstrap_by_rail(uint8_t rail, bootstrap_mapping_register *result)
{
	CHECK_NULL_ARG_WITH_RETURN(result, false);

	if (rail >= get_strap_index_max()) {
		return false;
	}

	*result = bootstrap_table[rail];
	return true;
}

bool get_bootstrap_change_drive_level(int rail, int *drive_level)
{
	CHECK_NULL_ARG_WITH_RETURN(drive_level, false);

	bootstrap_mapping_register bootstrap_item;
	if (!find_bootstrap_by_rail((uint8_t)rail, &bootstrap_item)) {
		LOG_ERR("Can't find strap_item by rail index: %d", rail);
		return false;
	}

	*drive_level = bootstrap_item.change_setting_value;
	LOG_DBG("rail %d, drive_level = %x", rail, *drive_level);
	return true;
}

bool strap_name_get(uint8_t rail, uint8_t **name)
{
	CHECK_NULL_ARG_WITH_RETURN(name, false);

	if (rail >= get_strap_index_max()) {
		*name = NULL;
		return false;
	}

	*name = (uint8_t *)bootstrap_table[rail].strap_name;
	return true;
}

bool strap_enum_get(uint8_t *name, uint8_t *num)
{
	CHECK_NULL_ARG_WITH_RETURN(name, false);
	CHECK_NULL_ARG_WITH_RETURN(num, false);

	for (int i = 0; i < get_strap_index_max(); i++) {
		if (strcmp(name, bootstrap_table[i].strap_name) == 0) {
			*num = i;
			return true;
		}
	}

	LOG_ERR("invalid rail name %s", name);
	return false;
}

bool set_bootstrap_table_and_user_settings(uint8_t rail, uint8_t *change_setting_value,
					   uint8_t drive_index_level, bool is_perm, bool is_default)
{
	if (rail >= get_strap_index_max())
		return false;

	*change_setting_value = 0;
	for (int i = 0; i < get_strap_index_max(); i++) {
		if (bootstrap_table[i].index == rail) {
			drive_index_level = (is_default) ?
						    bootstrap_table[i].default_setting_value :
						    drive_index_level;
			if (!check_is_bootstrap_setting_value_valid(rail, drive_index_level)) {
				LOG_ERR("hex-value :0x%x is out of range", drive_index_level);
				return false;
			}
			bootstrap_table[i].change_setting_value = drive_index_level;

			// get whole cpld register value to set
			for (int j = 0; j < get_strap_index_max(); j++) {
				if ((bootstrap_table[j].cpld_offsets ==
				     bootstrap_table[i].cpld_offsets) &&
				    (bootstrap_table[j].type == bootstrap_table[i].type)) {
					uint8_t tmp_reverse = 0;
					if (bootstrap_table[j].reverse)
						tmp_reverse = reverse_bits(
							bootstrap_table[j].change_setting_value,
							bootstrap_table[j].bit_count);

					for (int k = 0; k < bootstrap_table[j].bit_count; k++) {
						if (bootstrap_table[j].reverse) {
							if (tmp_reverse & BIT(k))
								*change_setting_value |= BIT(
									k + bootstrap_table[j]
										    .bit_offset);
						} else {
							if (bootstrap_table[j].change_setting_value &
							    BIT(k))
								*change_setting_value |= BIT(
									k + bootstrap_table[j]
										    .bit_offset);
						}
					}
				}
			}

			LOG_DBG("set [%2d]%s: %02x", rail, bootstrap_table[i].strap_name,
				*change_setting_value);
			/*
				save perm parameter to bootstrap_user_settings
				bit 8: not perm(ff); perm(1)
				bit 0: get_bootstrap_change_drive_level -> low(0); high(1)
				ex: 0x0101 -> set high perm; ex: 0x0100 -> set low perm
			*/
			if (is_perm) {
				int drive_level = -1;
				if (!get_bootstrap_change_drive_level(i, &drive_level)) {
					LOG_ERR("Can't get_bootstrap_change_drive_level by rail index: %x",
						i);
					return false;
				}
				bootstrap_user_settings.user_setting_value[i] =
					((drive_level & 0x00FF) | 0x0100);
				bootstrap_user_settings_set(&bootstrap_user_settings);
			}
			return true;
		}
	}

	return false;
}

bool set_bootstrap_val_to_device(uint8_t strap, uint8_t val)
{
	uint8_t type = bootstrap_table[strap].type;

	switch (type) {
	case STRAP_TYPE_CPLD:
		if (!plat_write_cpld(bootstrap_table[strap].cpld_offsets, &val)) {
			return false;
		}
		/* when TEST_STRAP to 0, change MFIO 6 8 10 to INPUT  */
		/* when TEST_STRAP to 1, change MFIO 6 8 10 to OUTPUT */
		if (is_evb_ioe_accessible()) {
			if (bootstrap_table[strap].index == STRAP_INDEX_HAMSA_TEST_STRAP_R) {
				if ((val & BIT(bootstrap_table[strap].bit_offset)) == 0) {
					set_hamsa_mfio_6_8_10_input();
				} else {
					set_hamsa_mfio_6_8_10_output();
					set_bootstrap_table_val_to_ioexp();
				}
			} else if (bootstrap_table[strap].index == STRAP_INDEX_NUWA0_TEST_STRAP) {
				if ((val & BIT(bootstrap_table[strap].bit_offset)) == 0) {
					set_nuwa0_mfio_6_8_10_input();
				} else {
					set_nuwa0_mfio_6_8_10_output();
					set_bootstrap_table_val_to_ioexp();
				}
			} else if (bootstrap_table[strap].index == STRAP_INDEX_NUWA1_TEST_STRAP) {
				if ((val & BIT(bootstrap_table[strap].bit_offset)) == 0) {
					set_nuwa1_mfio_6_8_10_input();
				} else {
					set_nuwa1_mfio_6_8_10_output();
					set_bootstrap_table_val_to_ioexp();
				}
			}
		}
		break;
	case STRAP_TYPE_IOEXP_PCA6416A:
		if (is_mb_dc_on()) {
			if (!pca6416a_i2c_write(bootstrap_table[strap].cpld_offsets, &val, 1))
				return false;
		}
		break;
	case STRAP_TYPE_IOEXP_TCA6424A:
		if (is_evb_ioe_accessible()) {
			if (!tca6424a_i2c_write(bootstrap_table[strap].cpld_offsets, &val, 1))
				return false;
		}
		break;
	case STRAP_TYPE_IOEXP_TCAL6408R:
		if (is_evb_ioe_accessible()) {
			if (!tcal6408r_i2c_write(bootstrap_table[strap].cpld_offsets, &val, 1))
				return false;
		}
		break;
	default:
		LOG_ERR("Invalid bootstrap_table[%d] type: %d", strap, type);
		return false;
	}

	return true;
}

bool bootstrap_user_settings_init(void)
{
	if (bootstrap_user_settings_get(&bootstrap_user_settings) == false) {
		LOG_ERR("get bootstrap user settings fail");
		return false;
	}

	for (int i = 0; i < get_strap_index_max(); i++) {
		uint8_t is_perm = ((bootstrap_user_settings.user_setting_value[i] >> 8) != 0xff) ?
					  true :
					  false;
		if (is_perm) {
			// write bootstrap_table
			uint8_t change_setting_value;
			uint8_t drive_index_level =
				bootstrap_user_settings.user_setting_value[i] & 0xFF;
			if (!set_bootstrap_table_and_user_settings(
				    i, &change_setting_value, drive_index_level, false, false)) {
				LOG_ERR("set bootstrap_table[%2d]:%d failed", i, drive_index_level);
				return false;
			}

			// write cpld or io-exp
			if (!set_bootstrap_val_to_device(i, change_setting_value))
				LOG_ERR("Can't set bootstrap[%2d]=%02x by user settings", i,
					change_setting_value);

			LOG_INF("set [%2d]%s: %02x", i, bootstrap_table[i].strap_name,
				change_setting_value);
		}
	}

	return true;
}

void plat_pldm_sensor_post_load_init(int thread_id)
{
	if (thread_id == TEMP_SENSOR_THREAD_ID) {
		temp_threshold_default_settings_init();
		temp_threshold_user_settings_init();
	}
	LOG_INF("plat_pldm_sensor_post_load init done");
}