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

#include "adm1272.h"
#include "hal_gpio.h"
#include "sensor.h"
#include "plat_isr.h"
#include "plat_gpio.h"
#include "plat_pwm.h"
#include "plat_hwmon.h"
#include "plat_sensor_table.h"
#include "plat_led.h"
#include "plat_log.h"
#include "plat_threshold.h"
#include "plat_status.h"
#include "plat_fru.h"

#define SURPRISE_SHUTDOWN_TOTAL_TIME_LENGTH                                                        \
	(EEPROM_UPTIME_SIZE + EEPROM_PUMP1_UPTIME_SIZE + EEPROM_PUMP2_UPTIME_SIZE +                \
	 EEPROM_PUMP3_UPTIME_SIZE)

LOG_MODULE_REGISTER(plat_isr);

void deassert_all_rpu_ready_pin(void)
{
	gpio_set(BIC_RPU_READY0, 0);
	gpio_set(BIC_RPU_READY1, 0);
	gpio_set(BIC_RPU_READY2, 0);
	gpio_set(BIC_RPU_READY3, 0);
}

void set_all_rpu_ready_pin_normal(void)
{
	gpio_set(BIC_RPU_READY0, 1);
	gpio_set(BIC_RPU_READY1, 1);
	gpio_set(BIC_RPU_READY2, 1);
	gpio_set(BIC_RPU_READY3, 1);
}

/* TO DO:
	1. turn off pump
	2. change hx fan to min speed
	3. store to non volatile memory
*/
void emergency_button_action()
{
	static bool pressed = false;
	set_status_flag(STATUS_FLAG_FAILURE, PUMP_FAIL_EMERGENCY_BUTTON, 1);
	deassert_all_rpu_ready_pin();
	if (!pressed)
		error_log_event(SENSOR_NUM_EMERGENCY_BUTTON_TRIGGERED, IS_ABNORMAL_VAL);
	pressed = true;
}

void fault_leak_action()
{
	set_status_flag(STATUS_FLAG_FAILURE, PUMP_FAIL_LEAK, 1);
	deassert_all_rpu_ready_pin();
	gpio_set(RPU_LEAK_ALERT_N, 0);
}

void it_leak_handler(uint8_t idx)
{
	uint8_t sen_num = (idx == IT_LEAK_E_0) ? SENSOR_NUM_IT_LEAK_0_GPIO :
			  (idx == IT_LEAK_E_1) ? SENSOR_NUM_IT_LEAK_1_GPIO :
			  (idx == IT_LEAK_E_2) ? SENSOR_NUM_IT_LEAK_2_GPIO :
			  (idx == IT_LEAK_E_3) ? SENSOR_NUM_IT_LEAK_3_GPIO :
						 0xFF;

	uint8_t gpio = (idx == IT_LEAK_E_0) ? IT_LEAK_ALERT0_R :
		       (idx == IT_LEAK_E_1) ? IT_LEAK_ALERT1_R :
		       (idx == IT_LEAK_E_2) ? IT_LEAK_ALERT2_R :
		       (idx == IT_LEAK_E_3) ? IT_LEAK_ALERT3_R :
					      0xFF;

	if (!gpio_get(gpio)) {
		LOG_WRN("IT_LEAK_ALERT%d_R is low, the high level time is less than 1s, ignore it",
			idx);
		return;
	}

	LOG_WRN("detect IT_LEAK_ALERT%d_R", idx);

	fault_leak_action();
	error_log_event(sen_num, IS_ABNORMAL_VAL);

	switch (idx) {
	case IT_LEAK_E_0:
		set_sticky_sensor_status(STICKY_ITRACK_CHASSIS0_LEAKAGE, 1);
		set_status_flag(STATUS_FLAG_LEAK, AALC_STATUS_IT_LEAK_0, 1);
		break;
	case IT_LEAK_E_1:
		set_sticky_sensor_status(STICKY_ITRACK_CHASSIS1_LEAKAGE, 1);
		set_status_flag(STATUS_FLAG_LEAK, AALC_STATUS_IT_LEAK_1, 1);
		break;
	case IT_LEAK_E_2:
		set_sticky_sensor_status(STICKY_ITRACK_CHASSIS2_LEAKAGE, 1);
		set_status_flag(STATUS_FLAG_LEAK, AALC_STATUS_IT_LEAK_2, 1);
		break;
	case IT_LEAK_E_3:
		set_sticky_sensor_status(STICKY_ITRACK_CHASSIS3_LEAKAGE, 1);
		set_status_flag(STATUS_FLAG_LEAK, AALC_STATUS_IT_LEAK_3, 1);
		break;
	}

	fault_led_control();
}

#define IT_LEAK_ALERT_HANDLER(idx)                                                                 \
	void it_leak_handle_##idx(struct k_work *work)                                             \
	{                                                                                          \
		it_leak_handler(IT_LEAK_E_##idx);                                                  \
	}                                                                                          \
	K_WORK_DELAYABLE_DEFINE(it_leak_work_##idx, it_leak_handle_##idx);                         \
	void it_leak_action_##idx(void)                                                            \
	{                                                                                          \
		k_work_schedule(&it_leak_work_##idx, K_MSEC(1000));                                \
	}

IT_LEAK_ALERT_HANDLER(0);
IT_LEAK_ALERT_HANDLER(1);
IT_LEAK_ALERT_HANDLER(2);
IT_LEAK_ALERT_HANDLER(3);

void aalc_leak_behavior(uint8_t sensor_num)
{
	fault_leak_action();
	// set pwm to 0 before fsc
	ctl_all_pwm_dev(0);
	uint8_t led_leak = (sensor_num == SENSOR_NUM_BPB_CDU_COOLANT_LEAKAGE_VOLT_V) ?
				   AALC_STATUS_CDU_LEAKAGE :
			   (sensor_num == SENSOR_NUM_BPB_RACK_COOLANT_LEAKAGE_VOLT_V) ?
				   AALC_STATUS_RACK_LEAKAGE :
				   AALC_STATUS_LEAK_E_MAX;

	if (!(get_status_flag(STATUS_FLAG_LEAK) & BIT(led_leak)))
		error_log_event(sensor_num, IS_ABNORMAL_VAL);

	set_status_flag(STATUS_FLAG_LEAK, led_leak, 1);
	set_sticky_sensor_status((sensor_num == SENSOR_NUM_BPB_CDU_COOLANT_LEAKAGE_VOLT_V) ?
					 STICKY_RPU_INTERNAL_LEAKAGE_ABNORMAL :
					 STICKY_HEX_RACK_PAN_LEAKAGE,
				 1);
	if (get_led_status(LED_IDX_E_LEAK) != LED_START_BLINK)
		led_ctrl(LED_IDX_E_LEAK, LED_START_BLINK);
	fault_led_control();
	gpio_set(RPU_LEAK_ALERT_N, 0);
}

void shutdown_save_uptime_action()
{
	// get total uptime
	uint8_t pre_time[EEPROM_UPTIME_SIZE] = { 0 };
	if (!plat_eeprom_read(EEPROM_UPTIME_OFFSET, pre_time, EEPROM_UPTIME_SIZE))
		LOG_ERR("read uptime fail!");

	uint32_t old_uptime =
		(pre_time[0] << 24) | (pre_time[1] << 16) | (pre_time[2] << 8) | pre_time[3];
	LOG_DBG("old uptime: %d\n", old_uptime);

	uint32_t get_uptime_mins = (k_uptime_get() / 60000); // mins(60 *1000 ms)
	LOG_DBG("uptime: %d\n", get_uptime_mins);

	get_uptime_mins += old_uptime;

	uint8_t temp[SURPRISE_SHUTDOWN_TOTAL_TIME_LENGTH] = { 0 };
	temp[0] = get_uptime_mins >> 24;
	temp[1] = get_uptime_mins >> 16;
	temp[2] = get_uptime_mins >> 8;
	temp[3] = get_uptime_mins;

	// get pump uptime
	uint32_t pump_uptime_secs = 0;
	for (int i = 0; i < PUMP_MAX_NUM; i++) {
		if (get_pump_uptime_secs(i, &pump_uptime_secs)) {
			temp[4 + i * 4] = (pump_uptime_secs >> 24);
			temp[5 + i * 4] = (pump_uptime_secs >> 16);
			temp[6 + i * 4] = (pump_uptime_secs >> 8);
			temp[7 + i * 4] = (pump_uptime_secs);
		}
	}

	// write total uptime
	if (!plat_eeprom_write(EEPROM_UPTIME_OFFSET, temp, SURPRISE_SHUTDOWN_TOTAL_TIME_LENGTH))
		LOG_ERR("write uptime fail!");
}
