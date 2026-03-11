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
#include <stdlib.h>
#include <string.h>
#include "plat_event.h"
#include "plat_hook.h"
#include "plat_kernel_obj.h"
#include "pldm_oem.h"
#include "plat_cpld.h"
#include "plat_log.h"
#include "pmbus.h"

LOG_MODULE_REGISTER(plat_event);

const vr_fault_info vr_fault_table[] = {
	// { arke_event_source, cpld_reg_offset, cpld_reg_bit }
    // "VR_RAIL_E_ASIC_P0V9_VDDQ_HBM1357" and "VR_RAIL_E_ASIC_P0V9_VDDQ_HBM0246" waiting for CPLD table
	// VR Power Fault 1
	{ ARKE_OWL_E_TRVDD0P9, VR_POWER_FAULT_1_REG, BIT(7), true,
	  VR_RAIL_E_ASIC_P0V9_OWL_E_TRVDD },
	{ ARKE_OWL_W_TRVDD0P9, VR_POWER_FAULT_1_REG, BIT(6), true,
	  VR_RAIL_E_ASIC_P0V9_OWL_W_TRVDD },
	{ ARKE_OWL_E_TRVDD0P75, VR_POWER_FAULT_1_REG, BIT(5), true,
	  VR_RAIL_E_ASIC_P0V75_OWL_E_TRVDD },
	{ ARKE_OWL_W_TRVDD0P75, VR_POWER_FAULT_1_REG, BIT(4), true,
	  VR_RAIL_E_ASIC_P0V75_OWL_W_TRVDD },
	{ ARKE_HAMSA_AVDD_PCIE, VR_POWER_FAULT_1_REG, BIT(3), true,
	  VR_RAIL_E_ASIC_P0V8_HAMSA_AVDD_PCIE },
	{ ARKE_HAMSA_VDDHRXTX_PCIE, VR_POWER_FAULT_1_REG, BIT(2), true,
	  VR_RAIL_E_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE },
	{ ARKE_4V2, VR_POWER_FAULT_1_REG, BIT(1), false },
	{ ARKE_P0V75_AVDD_HCSL, VR_POWER_FAULT_1_REG, BIT(0), false },
	// VR Power Fault 2
	{ ARKE_NUWA1_VDD, VR_POWER_FAULT_2_REG, BIT(7), true, VR_RAIL_E_ASIC_P0V75_NUWA1_VDD },
	{ ARKE_NUWA0_VDD, VR_POWER_FAULT_2_REG, BIT(6), true, VR_RAIL_E_ASIC_P0V75_NUWA0_VDD },
	{ ARKE_OWL_E_VDD, VR_POWER_FAULT_2_REG, BIT(5), true, VR_RAIL_E_ASIC_P0V75_OWL_E_VDD },
	{ ARKE_OWL_W_VDD, VR_POWER_FAULT_2_REG, BIT(4), true, VR_RAIL_E_ASIC_P0V75_OWL_W_VDD },
	{ ARKE_HAMSA_VDD, VR_POWER_FAULT_2_REG, BIT(3), true, VR_RAIL_E_ASIC_P0V85_HAMSA_VDD },
	{ ARKE_MAX_S_VDD, VR_POWER_FAULT_2_REG, BIT(2), true, VR_RAIL_E_ASIC_P0V75_MAX_S_VDD },
	{ ARKE_MAX_M_VDD, VR_POWER_FAULT_2_REG, BIT(1), true, VR_RAIL_E_ASIC_P0V75_MAX_M_VDD },
	{ ARKE_MAX_N_VDD, VR_POWER_FAULT_2_REG, BIT(0), true, VR_RAIL_E_ASIC_P0V75_MAX_N_VDD },
	// VR Power Fault 3
	{ ARKE_VDDQL_HBM0_HBM2_HBM4_HBM6, VR_POWER_FAULT_3_REG, BIT(7), true,
	  VR_RAIL_E_ASIC_P0V4_VDDQL_HBM0246 },
	{ ARKE_VDDQC_HBM0_HBM2_HBM4_HBM6, VR_POWER_FAULT_3_REG, BIT(6), true,
	  VR_RAIL_E_ASIC_P1V05_VDDQC_HBM0246 },
	{ ARKE_VPP_HBM0_HBM2_HBM4_HBM6, VR_POWER_FAULT_3_REG, BIT(5), true,
	  VR_RAIL_E_ASIC_P1V8_VPP_HBM0246 },
	{ ARKE_VDDPHY_HBM0_HBM2_HBM4_HBM6, VR_POWER_FAULT_3_REG, BIT(4), true,
	  VR_RAIL_E_ASIC_P0V75_VDDPHY_HBM0246 },
	{ ARKE_VDDQL_HBM1_HBM3_HBM5_HBM7, VR_POWER_FAULT_3_REG, BIT(3), true,
	  VR_RAIL_E_ASIC_P0V4_VDDQL_HBM1357 },
	{ ARKE_VDDQC_HBM1_HBM3_HBM5_HBM7, VR_POWER_FAULT_3_REG, BIT(2), true,
	  VR_RAIL_E_ASIC_P1V05_VDDQC_HBM1357 },
	{ ARKE_VPP_HBM1_HBM3_HBM5_HBM7, VR_POWER_FAULT_3_REG, BIT(1), true,
	  VR_RAIL_E_ASIC_P1V8_VPP_HBM1357 },
	{ ARKE_VDDPHY_HBM1_HBM3_HBM5_HBM7, VR_POWER_FAULT_3_REG, BIT(0), true,
	  VR_RAIL_E_ASIC_P0V75_VDDPHY_HBM1357 },
	// VR Power Fault 4
	{ ARKE_PLL_VDDA15_HBM0_HBM2, VR_POWER_FAULT_4_REG, BIT(7), false },
	{ ARKE_PLL_VDDA15_HBM4_HBM6, VR_POWER_FAULT_4_REG, BIT(6), false },
	{ ARKE_PLL_VDDA15_HBM1_HBM3, VR_POWER_FAULT_4_REG, BIT(5), false },
	{ ARKE_PLL_VDDA15_HBM5_HBM7, VR_POWER_FAULT_4_REG, BIT(4), false },
	{ ARKE_P0V9_OWL_E_PVDD, VR_POWER_FAULT_4_REG, BIT(3), false },
	{ ARKE_P0V9_OWL_W_PVDD, VR_POWER_FAULT_4_REG, BIT(2), false },
	{ ARKE_P1V5_E_RVDD, VR_POWER_FAULT_4_REG, BIT(1), false },
	{ ARKE_P1V5_W_RVDD, VR_POWER_FAULT_4_REG, BIT(0), false },
	// VR Power Fault 5
	{ ARKE_P12V_UBC_PWRGD, VR_POWER_FAULT_5_REG, BIT(7), false }, // if true, is it 1 or 2?
	{ ARKE_P5V, VR_POWER_FAULT_5_REG, BIT(6), false },
	{ ARKE_P3V3, VR_POWER_FAULT_5_REG, BIT(5), false }, // osfp 3v3?
	{ ARKE_P1V8, VR_POWER_FAULT_5_REG, BIT(4), false },
	{ ARKE_LDO_IN_1V2, VR_POWER_FAULT_5_REG, BIT(3), false },
	{ ARKE_P1V5_PLL_VDDA_OWL, VR_POWER_FAULT_5_REG, BIT(2), false },
	{ ARKE_P1V5_PLL_VDDA_SOC, VR_POWER_FAULT_5_REG, BIT(1), false },
	{ ARKE_PVDD1P5, VR_POWER_FAULT_5_REG, BIT(0), false },
	// ASIC TEMP OVER
	{ ARKE_ASIC_THERMTRIP, ASIC_TEMP_OVER_REG, BIT(7), false },
	// HBM CATTRIP
	{ ARKE_NUWA1_HBM_CATTRIP, HBM_CATTRIP_REG, BIT(6), false },
	{ ARKE_NUWA0_HBM_CATTRIP, HBM_CATTRIP_REG, BIT(7), false },
	// VR SMBUS ALERT
	{ ARKE_MAX_N_VDDRXTX_SMBALRT_N, VR_SMBUS_ALERT_EVENT_LOG_REG, BIT(1), false },
	{ ARKE_VDDQC_VDDQL_0246_SMBALRT_N, VR_SMBUS_ALERT_EVENT_LOG_REG, BIT(2), false },
	{ ARKE_MAX_M_VDDQC_1357_SMBALRT_N, VR_SMBUS_ALERT_EVENT_LOG_REG, BIT(3), false },
	{ ARKE_OWL_W_SMBALRT_N, VR_SMBUS_ALERT_EVENT_LOG_REG, BIT(4), false },
	{ ARKE_OWL_E_SMBALRT_N, VR_SMBUS_ALERT_EVENT_LOG_REG, BIT(5), false },
	{ ARKE_NUWA1_VDD_ALERT_R_N, VR_SMBUS_ALERT_EVENT_LOG_REG, BIT(6), false },
	{ ARKE_NUWA0_VDD_ALERT_R_N, VR_SMBUS_ALERT_EVENT_LOG_REG, BIT(7), false },
	// VR TEMP OVER
};
const vr_mapping_status vr_status_rail_list[] = {
	{ .index = VR_STAUS_E_STATUS_BYTE, .pmbus_reg = PMBUS_STATUS_BYTE },
	{ .index = VR_STAUS_E_STATUS_WORD, .pmbus_reg = PMBUS_STATUS_WORD },
	{ .index = VR_STAUS_E_STATUS_VOUT, .pmbus_reg = PMBUS_STATUS_VOUT },
	{ .index = VR_STAUS_E_STATUS_IOUT, .pmbus_reg = PMBUS_STATUS_IOUT },
	{ .index = VR_STAUS_E_STATUS_INPUT, .pmbus_reg = PMBUS_STATUS_INPUT },
	{ .index = VR_STAUS_E_STATUS_TEMPERATURE, .pmbus_reg = PMBUS_STATUS_TEMPERATURE },
	{ .index = VR_STAUS_E_STATUS_CML, .pmbus_reg = PMBUS_STATUS_CML },
};
void process_mtia_vr_power_fault_sel(cpld_info *cpld_info, uint8_t *current_cpld_value)
{
	CHECK_NULL_ARG(cpld_info);
	CHECK_NULL_ARG(current_cpld_value);
	LOG_INF("process_mtia_vr_power_fault_sel");
	bool dc_status = plat_get_ubc_status(); // get_ubc_enabled_delayed_status in rainbow
	uint8_t expected_val =
		dc_status ? cpld_info->dc_on_defaut : cpld_info->dc_off_defaut;

	uint8_t current_fault = (*current_cpld_value ^ expected_val) & cpld_info->bit_check_mask;
	uint8_t status_changed_bit = current_fault ^ cpld_info->is_fault_bit_map;

	for (int i = 0; i < ARRAY_SIZE(vr_fault_table); i++) {
		const vr_fault_info *vr = &vr_fault_table[i];

		if (vr->cpld_reg_offset != cpld_info->cpld_offset)
			continue;

		if (!(status_changed_bit & vr->cpld_reg_bit))
			continue;

		uint8_t bit_val = (*current_cpld_value & vr->cpld_reg_bit) ? 1 : 0;
		uint8_t expected_bit_val = (expected_val & vr->cpld_reg_bit) ? 1 : 0;

		// Determine event type: ASSERT / DEASSERT
		bool is_assert = (bit_val != expected_bit_val);

		LOG_INF("VR[0x%02X] reg[0x%02X] bit[0x%02X] is %s ", vr->mtia_event_source,
			vr->cpld_reg_offset, vr->cpld_reg_bit, is_assert ? "ASSERT" : "DEASSERT");

		if (vr_fault_table[i].is_pmbus_vr == false) {
			// non-PMBus VR
			struct pldm_addsel_data sel_msg = { 0 };

			sel_msg.assert_type = is_assert ? LOG_ASSERT : LOG_DEASSERT;
			sel_msg.event_type = ARKE_FAULT;
			sel_msg.event_data_1 = vr_fault_table[i].mtia_event_source;

			if (PLDM_SUCCESS != send_event_log_to_bmc(sel_msg)) {
				LOG_ERR("Fail send event: 0x%x 0x%x 0x%x", sel_msg.event_data_1,
					sel_msg.event_data_2, sel_msg.event_data_3);
			} else {
				LOG_INF("Send event: 0x%x 0x%x 0x%x", sel_msg.event_data_1,
					sel_msg.event_data_2, sel_msg.event_data_3);
			}
		} else {
			set_plat_sensor_polling_enable_flag(false);
			// wait 10ms for vr monitor stop
			k_msleep(10);

			uint8_t vr_reg_list_len = ARRAY_SIZE(vr_status_rail_list);
			struct pldm_addsel_data sel_msg[vr_reg_list_len];
			memset(sel_msg, 0, sizeof(sel_msg));
			uint8_t sel_msg_idx = 0;
			for (int j = 0; j < ARRAY_SIZE(vr_status_rail_list); j++) {
				uint8_t vr_status_rail = vr_status_rail_list[j].index;
				uint16_t vr_status = 0xFFFF;
				if (!plat_get_vr_status(vr->rail_id, vr_status_rail, &vr_status)) {
					LOG_ERR("Fail get VR st: VR[0x%02X] reg[0x%02X]",
						vr->mtia_event_source,
						vr_status_rail_list[j].pmbus_reg);
				}
				LOG_INF("VR rail id[0x%02X] status: reg[0x%02X] 0x%04X",
					vr->rail_id, vr_status_rail_list[j].pmbus_reg, vr_status);

				sel_msg[sel_msg_idx].assert_type =
					is_assert ? LOG_ASSERT : LOG_DEASSERT;
				sel_msg[sel_msg_idx].event_type = ARKE_FAULT;
				sel_msg[sel_msg_idx].event_data_1 = vr->mtia_event_source;
				sel_msg[sel_msg_idx].event_data_2 =
					(vr_status_rail_list[j].pmbus_reg == PMBUS_STATUS_WORD) ?
						(uint8_t)((vr_status >> 8) & 0xFF) :
						vr_status_rail_list[j].pmbus_reg;
				sel_msg[sel_msg_idx].event_data_3 = (uint8_t)(vr_status & 0xFF);
				sel_msg_idx += 1;
			}
			set_plat_sensor_polling_enable_flag(true);
			// Send SEL to BMC
			for (int k = 0; k < sel_msg_idx; k++) {
				if (PLDM_SUCCESS != send_event_log_to_bmc(sel_msg[k])) {
					LOG_ERR("Fail send event: 0x%x 0x%x 0x%x",
						sel_msg[k].event_data_1, sel_msg[k].event_data_2,
						sel_msg[k].event_data_3);
				} else {
					LOG_INF("Send event: 0x%x 0x%x 0x%x",
						sel_msg[k].event_data_1, sel_msg[k].event_data_2,
						sel_msg[k].event_data_3);
				}
			}
		}
	}
}

void plat_set_ac_on_log(void)
{
	uint16_t error_code = (AC_ON_TRIGGER_CAUSE << 13);
	error_log_event(error_code, LOG_ASSERT);
	LOG_INF("Generated AC on error code: 0x%x", error_code);
}

void plat_set_dc_on_log(bool is_assert)
{
	uint16_t error_code = (DC_ON_TRIGGER_CAUSE << 13);
	error_log_event(error_code, (is_assert ? LOG_ASSERT : LOG_DEASSERT));

	if (is_assert == LOG_ASSERT) {
		LOG_INF("Generated DC on error code: 0x%x", error_code);
	} else if (is_assert == LOG_DEASSERT) {
		LOG_INF("DC on error code deasserted");
	}
}