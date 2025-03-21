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
	{ ASIC_OC_WARN_2_REG, 				0xFF, 0xDF, true, 0x00, false, false, 0x00,  .status_changed_cb = vr_error_callback },
	{ SYSTEM_ALERT_FAULT_2_REG, 		0xFF, 0xFF, true, 0x00, false, false, 0x00,  .status_changed_cb = vr_error_callback },
	{ VR_SMBUS_ALERT_3_REG, 			0xFF, 0xFF, true, 0x00, false, false, 0x00,  .status_changed_cb = vr_error_callback },
	{ VR_SMBUS_ALERT_4_REG, 			0xFF, 0xFF, true, 0x00, false, false, 0x00,  .status_changed_cb = vr_error_callback },	
};
// clang-format on

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
	uint8_t new_fault = current_fault & ~cpld_info->is_fault_bit_map;

	if (!new_fault) {
		return true; // No new faults, return early
	}

	LOG_DBG("CPLD register 0x%02X has fault 0x%02X", cpld_info->cpld_offset, new_fault);

	// Iterate through each bit in new_fault to handle the corresponding VR
	for (uint8_t bit = 0; bit < 8; bit++) {
		if (!(new_fault & (1 << bit))) {
			continue; // Skip if the current bit has no new fault
		}

		// Dynamically generate the error code
		uint16_t error_code = (CPLD_UNEXPECTED_VAL_TRIGGER_CAUSE << 13) | (bit << 8) |
				      cpld_info->cpld_offset;

		LOG_ERR("Generated error code: 0x%04X (bit %d, CPLD offset 0x%02X)", error_code,
			bit, cpld_info->cpld_offset);

		// Perform additional operations if needed
		error_log_event(error_code, LOG_ASSERT);
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
			uint8_t unrecorded_fault_map =
				new_fault_map & ~aegis_cpld_info_table[i].is_fault_bit_map;

			if (unrecorded_fault_map) {
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
				K_MSEC(3000)); /* Start accessing CPLD 4 seconds after BIC reboot 
                   (3-second thread start delay + 1-second CPLD_POLLING_INTERVAL_MS) 
                   to prevent DC status changes during BIC reboot */
	k_thread_name_set(&cpld_polling_thread, "cpld_polling_thread");

	k_timer_start(&init_ubc_delayed_timer, K_MSEC(3000), K_NO_WAIT);

	k_work_schedule(&check_cpld_work, K_MSEC(100));
}
