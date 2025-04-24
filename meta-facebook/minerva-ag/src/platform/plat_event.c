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
#include <logging/log.h>
#include "libutil.h"
#include "plat_i2c.h"
#include "plat_class.h"
#include "plat_pldm_sensor.h"
#include "plat_gpio.h"
#include "plat_log.h"
#include "plat_event.h"
#include "plat_hook.h"
#include "plat_isr.h"

LOG_MODULE_REGISTER(plat_event);

#define CPLD_POLLING_INTERVAL_MS 1000 // 1 second polling interval

#ifndef AEGIS_CPLD_ADDR
#define AEGIS_CPLD_ADDR (0x4C >> 1)
#endif

void check_cpld_handler();
K_WORK_DELAYABLE_DEFINE(check_cpld_work, check_cpld_handler);

K_TIMER_DEFINE(init_ubc_delayed_timer, check_ubc_delayed_timer_handler, NULL);

K_WORK_DEFINE(check_ubc_delayed_work, check_ubc_delayed);

void check_ubc_delayed_timer_handler(struct k_timer *timer)
{
	k_work_submit(&check_ubc_delayed_work);
}

K_THREAD_STACK_DEFINE(cpld_polling_stack, POLLING_CPLD_STACK_SIZE);
struct k_thread cpld_polling_thread;
k_tid_t cpld_polling_tid;

void get_vr_vout_handler(struct k_work *work);
K_WORK_DEFINE(vr_vout_work, get_vr_vout_handler);

typedef struct _vr_error_callback_info_ {
	uint8_t cpld_offset;
	uint8_t vr_status_word_access_map;
	uint8_t bit_mapping_vr_sensor_num[8];
} vr_error_callback_info;

bool vr_error_callback(aegis_cpld_info *cpld_info, uint8_t *current_cpld_value);

// clang-format off
aegis_cpld_info aegis_cpld_info_table[] = {
	{ VR_POWER_FAULT_1_REG, 			0x00, 0x00, true, 0x00, false, false, 0x00,  .status_changed_cb = vr_error_callback },
	{ VR_POWER_FAULT_2_REG, 			0x00, 0x00, true, 0x00, false, false, 0x00,  .status_changed_cb = vr_error_callback },
	{ VR_POWER_FAULT_3_REG, 			0x00, 0x00, true, 0x00, false, false, 0x00,  .status_changed_cb = vr_error_callback },
	{ VR_POWER_FAULT_4_REG, 			0x00, 0x00, true, 0x00, false, false, 0x00,  .status_changed_cb = vr_error_callback },
	{ VR_POWER_FAULT_5_REG, 			0x00, 0x00, true, 0x00, false, false, 0x00,  .status_changed_cb = vr_error_callback },
	{ VR_SMBUS_ALERT_1_REG, 			0xFF, 0xFF, true, 0x00, false, false, 0x00,  .status_changed_cb = vr_error_callback },
	{ VR_SMBUS_ALERT_2_REG, 			0xFF, 0xFF, true, 0x00, false, false, 0x00,  .status_changed_cb = vr_error_callback },
	{ ASIC_OC_WARN_REG, 				0xFF, 0xFF, true, 0x00, false, false, 0x00,  .status_changed_cb = vr_error_callback },
	{ SYSTEM_ALERT_FAULT_REG, 			0xFF, 0xFF, true, 0x00, false, false, 0x00,  .status_changed_cb = vr_error_callback },
	{ TEMPERATURE_IC_OVERT_FAULT_REG, 	0xFF, 0xFF, true, 0x00, false, false, 0x00,  .status_changed_cb = vr_error_callback },
	{ LEAK_DETCTION_REG, 				0xDF, 0xDF, true, 0x00, false, false, 0x00,  .status_changed_cb = vr_error_callback },

	{ TEMPERATURE_IC_OVERT_FAULT_2_REG, 0xFF, 0xFF, true, 0x00, false, false, 0x00,  .status_changed_cb = vr_error_callback },
	{ ASIC_OC_WARN_2_REG, 				0xDF, 0xDF, true, 0x00, false, false, 0x00,  .status_changed_cb = vr_error_callback },
	{ SYSTEM_ALERT_FAULT_2_REG, 		0xFF, 0xFF, true, 0x00, false, false, 0x00,  .status_changed_cb = vr_error_callback },
	{ VR_SMBUS_ALERT_3_REG, 			0xFF, 0xFF, true, 0x00, false, false, 0x00,  .status_changed_cb = vr_error_callback },
	{ VR_SMBUS_ALERT_4_REG, 			0xFF, 0xFF, true, 0x00, false, false, 0x00,  .status_changed_cb = vr_error_callback },	
};
// clang-format on

const cpld_bit_name_table_t cpld_bit_name_table[] = {
	{ VR_POWER_FAULT_1_REG,
	  "VR Power Fault   (1:Power Fault, 0=Normal)",
	  {
		  "RSVD",
		  "P0V75_TRVDD_ZONEA",
		  "P0V75_TRVDD_ZONEB",
		  "P12V_UBC2",
		  "P12V_UBC1",
		  "P0V75_MAX_PHY_S",
		  "P0V75_MAX_PHY_N",
		  "RSVD",
	  } },
	{ VR_POWER_FAULT_2_REG,
	  "VR Power Fault   (1:Power Fault, 0=Normal)",
	  {
		  "P0V75_VDDPHY_HBM1_HBM3_HBM5",
		  "P0V75_VDDPHY_HBM0_HBM2_HBM4",
		  "VPP_HBM1_HBM3_HBM5",
		  "VPP_HBM0_HBM2_HBM4",
		  "VDDHTX_PCIE",
		  "PLL_VDDA15_PCIE&MAX&CORE",
		  "P0V9_TRVDD_ZONEA",
		  "P0V9_TRVDD_ZONEB",
	  } },
	{ VR_POWER_FAULT_3_REG,
	  "VR Power Fault   (1:Power Fault, 0=Normal)",
	  {
		  "P0V75_PVDD_CH_S",
		  "P0V75_PVDD_CH_N",
		  "P1V1_VDDC_HBM1_HBM3_HBM5",
		  "P0V75_AVDD_HSCL",
		  "P1V1_VDDC_HBM0_HBM2_HBM4",
		  "P0V75_VDDC_CLKOBS",
		  "VDDQL_HBM1_HBM3_HBM5",
		  "VDDQL_HBM0_HBM2_HBM4",
	  } },
	{ VR_POWER_FAULT_4_REG,
	  "VR Power Fault   (1:Power Fault, 0=Normal)",
	  {
		  "PLL_VDDA15_MAX&CORE_N",
		  "PVDD1P5_S",
		  "PVDD1P5_N",
		  "PVDD0P9_S",
		  "PVDD0P9_N",
		  "PLL_VDDA15_HBM0_HBM2_HBM4",
		  "PLL_VDDA15_HBM1_HBM3_HBM5",
		  "P0V85_PVDD",
	  } },
	{ VR_POWER_FAULT_5_REG,
	  "VR Power Fault   (1:Power Fault, 0=Normal)",
	  {
		  "1'b0",
		  "P3V3_OSC",
		  "P5V",
		  "P3V3",
		  "LDO_IN_1V8",
		  "LDO_IN_1V2",
		  "VDDA_PCIE",
		  "PLL_VDDA15_MAX&CORE_S",
	  } },
	{ VR_SMBUS_ALERT_1_REG,
	  "VR SMBUS Alert",
	  {
		  "P3V3_OSFP_ALERT_N (1-->0)",
		  "P1V1_VDDC_HBM1_HBM3_HBM5_SMBALERT_R_N (1-->0)",
		  "P1V1_VDDC_HBM0_HBM2_HBM4_SMBALERT_R_N (1-->0)",
		  "P0V9_TRVDD_ZONEA_SMBALERT_R_N (1-->0)",
		  "P0V9_TRVDD_ZONEB_SMBALERT_R_N (1-->0)",
		  "P0V85_PVDD_ALERT_R_N (1-->0)",
		  "P0V75_PVDD_CH_S_SMBALERT_R_N (1-->0)",
		  "P0V75_PVDD_CH_N_SMBALERT_R_N (1-->0)",
	  } },
	{ VR_SMBUS_ALERT_2_REG,
	  "VR SMBUS Alert",
	  {
		  "RSVD",
		  "RSVD",
		  "RSVD",
		  "VDDA_PCIE_SMBALERT_R_N (1-->0)",
		  "P0V75_TRVDD_ZONEA_SMBALERT_R_N (1-->0)",
		  "P0V75_TRVDD_ZONEB_SMBALERT_R_N (1-->0)",
		  "RSVD",
		  "RSVD",
	  } },
	{ ASIC_OC_WARN_REG,
	  "ASIC OC WARN ,HBM CATTRIP and ASIC Alert to BMC/MMC",
	  {
		  "RSVD",
		  "RSVD",
		  "RSVD",
		  "RSVD",
		  "RSVD",
		  "FM_ATH_PLD_HBM3_CATTRIP_ALARM (0-->1)",
		  "FM_CPLD_ATH_CURRENT_SENSE_1 (1-->0)",
		  "FM_CPLD_ATH_CURRENT_SENSE_0 (1-->0)",
	  } },
	{ SYSTEM_ALERT_FAULT_REG,
	  "System Alert Fault",
	  {
		  "RSVD",
		  "RSVD",
		  "RSVD",
		  "RSVD",
		  "RSVD",
		  "RSVD",
		  "RSVD",
		  "FM_MODULE_PWRBRK_R_N (MB CPLD to Aegis CPLD) (1-->0)",
	  } },
	{ TEMPERATURE_IC_OVERT_FAULT_REG,
	  "Temperature IC OVERT fault",
	  {
		  "RSVD",
		  "RSVD",
		  "IRQ_TMP75_1_ALERT_R_N (1-->0)",
		  "IRQ_TMP75_2_ALERT_R_N (1-->0)",
		  "IRQ_TMP75_3_ALERT_R_N (1-->0)",
		  "IRQ_TMP75_4_ALERT_R_N (1-->0)",
		  "TEMP_MON_OVERT_N (1-->0)",
		  "FM_ASIC_0_THERMTRIP_N (1-->0)",
	  } },
	{ LEAK_DETCTION_REG,
	  "Leak Detection",
	  {
		  "RSVD",
		  "RSVD",
		  "RSVD",
		  "RSVD",
		  "RSVD",
		  "RSVD",
		  "LEAK_DETECT_ALERT_CPLD_N",
		  "RSVD",
	  } },
	{ TEMPERATURE_IC_OVERT_FAULT_2_REG,
	  "Temperature IC OVERT Fault Status",
	  {
		  "RSVD",
		  "RSVD",
		  "IRQ_TMP75_1_ALERT_R_N",
		  "IRQ_TMP75_2_ALERT_R_N",
		  "IRQ_TMP75_3_ALERT_R_N",
		  "IRQ_TMP75_4_ALERT_R_N",
		  "TEMP_MON_OVERT_N",
		  "FM_ASIC_0_THERMTRIP_N",
	  } },
	{ ASIC_OC_WARN_2_REG,
	  "ASIC OC WARN ,HBM CATTRIP and ASIC Alert to BMC/MMC Status",
	  {
		  "RSVD",
		  "RSVD",
		  "RSVD",
		  "RSVD",
		  "RSVD",
		  "FM_ATH_PLD_HBM3_CATTRIP_ALARM",
		  "FM_CPLD_ATH_CURRENT_SENSE_1",
		  "FM_CPLD_ATH_CURRENT_SENSE_0",
	  } },
	{ SYSTEM_ALERT_FAULT_2_REG,
	  "System Alert Status",
	  {
		  "RSVD",
		  "RSVD",
		  "RSVD",
		  "RSVD",
		  "RSVD",
		  "RSVD",
		  "RSVD",
		  "FM_MODULE_PWRBRK_R_N (MB CPLD to Aegis CPLD)",
	  } },
	{ VR_SMBUS_ALERT_3_REG,
	  "VR SMBUS Alert Status",
	  {
		  "P3V3_OSFP_ALERT_N (EVB only,Aegis no support)",
		  "P1V1_VDDC_HBM1_HBM3_HBM5_SMBALERT_R_N",
		  "P1V1_VDDC_HBM0_HBM2_HBM4_SMBALERT_R_N",
		  "P0V9_TRVDD_ZONEA_SMBALERT_R_N",
		  "P0V9_TRVDD_ZONEB_SMBALERT_R_N",
		  "P0V85_PVDD_ALERT_R_N",
		  "P0V75_PVDD_CH_S_SMBALERT_R_N",
		  "P0V75_PVDD_CH_N_SMBALERT_R_N",
	  } },
	{ VR_SMBUS_ALERT_4_REG,
	  "VR SMBUS Alert Status",
	  {
		  "RSVD",
		  "RSVD",
		  "RSVD",
		  "VDDA_PCIE_SMBALERT_R_N",
		  "P0V75_TRVDD_ZONEA_SMBALERT_R_N",
		  "P0V75_TRVDD_ZONEB_SMBALERT_R_N",
		  "RSVD",
		  "RSVD",
	  } },
};

const char *get_cpld_reg_name(uint8_t cpld_offset)
{
	for (int i = 0; i < ARRAY_SIZE(cpld_bit_name_table); i++) {
		if (cpld_bit_name_table[i].cpld_offset == cpld_offset) {
			return cpld_bit_name_table[i].reg_name;
		}
	}
	return "NA";
}

const char *get_cpld_bit_name(uint8_t cpld_offset, uint8_t bit_pos)
{
	for (int i = 0; i < ARRAY_SIZE(cpld_bit_name_table); i++) {
		if (cpld_bit_name_table[i].cpld_offset == cpld_offset) {
			if (bit_pos < 8 && cpld_bit_name_table[i].bit_name[bit_pos]) {
				return cpld_bit_name_table[i].bit_name[bit_pos];
			}
		}
	}
	return "NA";
}

bool cpld_polling_alert_status = false; // only polling cpld when alert status is true
bool cpld_polling_enable_flag = true;

void check_cpld_polling_alert_status(void)
{
	cpld_polling_alert_status = (gpio_get(ALL_VR_PM_ALERT_R_N) == GPIO_LOW);
}

void set_cpld_polling_enable_flag(bool status)
{
	cpld_polling_enable_flag = status;
}

bool get_cpld_polling_enable_flag(void)
{
	return cpld_polling_enable_flag;
}

bool ubc_enabled_delayed_status = false;

void check_ubc_delayed(struct k_work *work)
{
	/* FM_PLD_UBC_EN_R
	 * 1 -> UBC is enabled
	 * 0 -> UBC is disabled
	 */
	bool is_ubc_enabled = (gpio_get(FM_PLD_UBC_EN_R) == GPIO_HIGH);

	bool is_dc_on = is_mb_dc_on();

	if (is_ubc_enabled) {
		if (is_dc_on != is_ubc_enabled) {
			//send event to bmc
			uint16_t error_code = (POWER_ON_SEQUENCE_TRIGGER_CAUSE << 13);
			error_log_event(error_code, LOG_ASSERT);
			LOG_ERR("Generated error code: 0x%x", error_code);
		}
	}

	ubc_enabled_delayed_status = is_ubc_enabled;

	LOG_DBG("UBC enabled delayed status: %d", ubc_enabled_delayed_status);

	if (is_ubc_enabled == true) {
		k_work_submit(&vr_vout_work);
	}
}

void plat_set_ac_on_log()
{
	uint16_t error_code = (AC_ON_TRIGGER_CAUSE << 13);
	error_log_event(error_code, LOG_ASSERT);
	LOG_ERR("Generated AC on error code: 0x%x", error_code);
}

void plat_set_dc_on_log(bool is_assert)
{
	uint16_t error_code = (DC_ON_TRIGGER_CAUSE << 13);
	error_log_event(error_code, (is_assert ? LOG_ASSERT : LOG_DEASSERT));

	if (is_assert == LOG_ASSERT) {
		LOG_ERR("Generated DC on error code: 0x%x", error_code);
	} else if (is_assert == LOG_DEASSERT) {
		LOG_INF("DC on error code deasserted");
	}
}

void get_vr_vout_handler(struct k_work *work)
{
	vr_vout_default_settings_init();
	vr_vout_user_settings_init();
}

bool is_ubc_enabled_delayed_enabled(void)
{
	return ubc_enabled_delayed_status;
}

bool vr_error_callback(aegis_cpld_info *cpld_info, uint8_t *current_cpld_value)
{
	CHECK_NULL_ARG_WITH_RETURN(cpld_info, false);
	CHECK_NULL_ARG_WITH_RETURN(current_cpld_value, false);

	// Get the expected value based on the current UBC status
	uint8_t expected_val =
		ubc_enabled_delayed_status ? cpld_info->dc_on_defaut : cpld_info->dc_off_defaut;

	// Calculate current faults and new faults
	uint8_t current_fault = *current_cpld_value ^ expected_val;
	uint8_t status_changed_bit = current_fault ^ cpld_info->is_fault_bit_map;

	if (!status_changed_bit)
		return true; // No new faults, return early

	LOG_DBG("CPLD register 0x%02X has status changed 0x%02X", cpld_info->cpld_offset,
		status_changed_bit);

	// Iterate through each bit in status_changed_bit to handle the corresponding VR
	for (uint8_t bit = 0; bit < 8; bit++) {
		if (!(status_changed_bit & BIT(bit)))
			continue;

		// Dynamically generate the error code
		uint16_t error_code = (CPLD_UNEXPECTED_VAL_TRIGGER_CAUSE << 13) | (bit << 8) |
				      cpld_info->cpld_offset;

		uint8_t bit_val = (*current_cpld_value & BIT(bit)) >> bit;
		uint8_t expected_bit_val = (expected_val & BIT(bit)) >> bit;
		LOG_DBG("cpld offset 0x%02X, bit %d, bit_val %d, expected_bit_val %d",
			cpld_info->cpld_offset, bit, bit_val, expected_bit_val);

		if (bit_val != expected_bit_val) {
			LOG_ERR("Generated error code: 0x%04X (bit %d, CPLD offset 0x%02X)",
				error_code, bit, cpld_info->cpld_offset);

			// Perform additional operations if needed
			LOG_DBG("ASSERT");
			error_log_event(error_code, LOG_ASSERT);
		} else {
			LOG_DBG("DEASSERT");
			error_log_event(error_code, LOG_DEASSERT);
		}
	}

	return true;
}

void poll_cpld_registers()
{
	uint8_t data = 0;

	while (1) {
		/* Sleep for the polling interval */
		k_msleep(CPLD_POLLING_INTERVAL_MS);

		LOG_DBG("cpld_polling_alert_status = %d, cpld_polling_enable_flag = %d",
			cpld_polling_alert_status, cpld_polling_enable_flag);

		if (!cpld_polling_alert_status || !cpld_polling_enable_flag)
			continue;

		LOG_DBG("Polling CPLD registers");

		for (size_t i = 0; i < ARRAY_SIZE(aegis_cpld_info_table); i++) {
			uint8_t expected_val = ubc_enabled_delayed_status ?
						       aegis_cpld_info_table[i].dc_on_defaut :
						       aegis_cpld_info_table[i].dc_off_defaut;

			// Read from CPLD
			if (!plat_read_cpld(aegis_cpld_info_table[i].cpld_offset, &data)) {
				LOG_ERR("Failed to read CPLD register 0x%02X",
					aegis_cpld_info_table[i].cpld_offset);
				continue;
			}

			if (!aegis_cpld_info_table[i].is_fault_log)
				continue;

			uint8_t new_fault_map = (data ^ expected_val);

			// get unrecorded fault bit map
			uint8_t is_status_changed =
				new_fault_map ^ aegis_cpld_info_table[i].is_fault_bit_map;

			if (is_status_changed) {
				if (aegis_cpld_info_table[i].status_changed_cb) {
					aegis_cpld_info_table[i].status_changed_cb(
						&aegis_cpld_info_table[i], &data);
				}
				// update map
				aegis_cpld_info_table[i].is_fault_bit_map = new_fault_map;
				aegis_cpld_info_table[i].last_polling_value = data;
			}
		}
	}
}

void check_cpld_handler()
{
	uint8_t data[4] = { 0 };
	uint32_t version = 0;
	if (!plat_i2c_read(I2C_BUS5, AEGIS_CPLD_ADDR, 0x44, data, 4)) {
		LOG_ERR("Failed to read cpld version from cpld");
	}
	version = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
	LOG_DBG("The cpld version: %08x", version);

	k_work_schedule(&check_cpld_work, K_MSEC(5000));
}

void init_cpld_polling(void)
{
	check_cpld_polling_alert_status();

	cpld_polling_tid =
		k_thread_create(&cpld_polling_thread, cpld_polling_stack,
				K_THREAD_STACK_SIZEOF(cpld_polling_stack), poll_cpld_registers,
				NULL, NULL, NULL, CONFIG_MAIN_THREAD_PRIORITY, 0,
				K_MSEC(1000)); /* Start accessing CPLD 2 seconds after BIC reboot 
                   (1-second thread start delay + 1-second CPLD_POLLING_INTERVAL_MS) 
                   to prevent DC status changes during BIC reboot */
	k_thread_name_set(&cpld_polling_thread, "cpld_polling_thread");

	k_timer_start(&init_ubc_delayed_timer, K_MSEC(1000), K_NO_WAIT);

	k_work_schedule(&check_cpld_work, K_MSEC(100));
}
