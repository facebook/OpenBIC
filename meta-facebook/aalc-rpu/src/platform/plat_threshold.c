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

#include "sensor.h"
#include "plat_sensor_table.h"
#include "plat_isr.h"
#include "plat_gpio.h"
#include "plat_pwm.h"
#include "plat_hwmon.h"
#include "plat_led.h"
#include "plat_log.h"
#include <logging/log.h>
#include "plat_threshold.h"
#include "plat_fsc.h"
#include <hal_i2c.h>
#include <nct7363.h>
#include <plat_hook.h>
#include "libutil.h"
#include "plat_status.h"
#include "adm1272.h"
#include "plat_class.h"

#define THRESHOLD_POLL_STACK_SIZE 2048
#define FAN_PUMP_PWRGD_STACK_SIZE 2048

#define ENABLE_REDUNDANCY 1
#define DISABLE_REDUNDANCY 0

#define READ_ERROR -1
#define FAN_BOARD_FAULT_PWM_OFFSET 0xA2
#define FAN_BOARD_FAULT_PWM_OFFSET_DVT 0x9E
#define FAN_BOARD_FAULT_PWM_DUTY_1 0x00
#define FAN_BOARD_FAULT_PWM_DUTY_50 0x7F
#define FAN_BOARD_FAULT_PWM_DUTY_100 0xFF

#define EVT_FAN_BOARD_FAULT_BIT_LOCATION 1
#define EVT_FAN_BOARD_PWRGD_BIT_LOCATION 0
#define DVT_FAN_BOARD_FAULT_BIT_LOCATION 7
#define DVT_FAN_BOARD_PWRGD_BIT_LOCATION 3
#define DVT_FAN_BOARD_FAULT_BLINK_ENABLE 0x0C
#define DVT_FAN_BOARD_FAULT_BLINK_DISABLE 0

#define EVT_PUMP_BOARD_FAULT_BIT_LOCATION 1
#define EVT_PUMP_BOARD_PWRGD_BIT_LOCATION 0
#define DVT_PUMP_BOARD_FAULT_BIT_LOCATION 3
#define DVT_PUMP_BOARD_PWRGD_BIT_LOCATION 7

#define THRESHOLD_ARG0_TABLE_INDEX 0xFFFFFFFF

#define HEX_FAN_NUM 14
static void fb_prsnt_handle(uint32_t thres_tbl_idx, uint32_t changed_status);
void pump_failure_do(uint32_t thres_tbl_idx, uint32_t status);
void abnormal_flow_do(uint32_t thres_tbl_idx, uint32_t status);

struct k_thread threshold_poll;
K_KERNEL_STACK_MEMBER(threshold_poll_stack, THRESHOLD_POLL_STACK_SIZE);

struct k_thread fan_pump_pwrgd_thread;
K_KERNEL_STACK_MEMBER(fan_pump_pwrgd_stack, FAN_PUMP_PWRGD_STACK_SIZE);

LOG_MODULE_REGISTER(plat_threshold);

static bool threshold_poll_enable_flag = true;

enum THRESHOLD_TYPE {
	THRESHOLD_DISABLE,
	THRESHOLD_ENABLE_LCR,
	THRESHOLD_ENABLE_UCR,
	THRESHOLD_ENABLE_BOTH,
	THRESHOLD_ENABLE_DISCRETE,
};

uint8_t fan_pump_sensor_array[] = {
	//fan board
	SENSOR_NUM_FB_1_FAN_TACH_RPM,
	SENSOR_NUM_FB_2_FAN_TACH_RPM,
	SENSOR_NUM_FB_3_FAN_TACH_RPM,
	SENSOR_NUM_FB_4_FAN_TACH_RPM,
	SENSOR_NUM_FB_5_FAN_TACH_RPM,
	SENSOR_NUM_FB_6_FAN_TACH_RPM,
	SENSOR_NUM_FB_7_FAN_TACH_RPM,
	SENSOR_NUM_FB_8_FAN_TACH_RPM,
	SENSOR_NUM_FB_9_FAN_TACH_RPM,
	SENSOR_NUM_FB_10_FAN_TACH_RPM,
	SENSOR_NUM_FB_11_FAN_TACH_RPM,
	SENSOR_NUM_FB_12_FAN_TACH_RPM,
	SENSOR_NUM_FB_13_FAN_TACH_RPM,
	SENSOR_NUM_FB_14_FAN_TACH_RPM,
	// pump
	SENSOR_NUM_PB_1_PUMP_TACH_RPM,
	SENSOR_NUM_PB_2_PUMP_TACH_RPM,
	SENSOR_NUM_PB_3_PUMP_TACH_RPM,
};

uint8_t fan_sensor_array[] = {
	SENSOR_NUM_FB_1_FAN_TACH_RPM,  SENSOR_NUM_FB_2_FAN_TACH_RPM,  SENSOR_NUM_FB_3_FAN_TACH_RPM,
	SENSOR_NUM_FB_4_FAN_TACH_RPM,  SENSOR_NUM_FB_5_FAN_TACH_RPM,  SENSOR_NUM_FB_6_FAN_TACH_RPM,
	SENSOR_NUM_FB_7_FAN_TACH_RPM,  SENSOR_NUM_FB_8_FAN_TACH_RPM,  SENSOR_NUM_FB_9_FAN_TACH_RPM,
	SENSOR_NUM_FB_10_FAN_TACH_RPM, SENSOR_NUM_FB_11_FAN_TACH_RPM, SENSOR_NUM_FB_12_FAN_TACH_RPM,
	SENSOR_NUM_FB_13_FAN_TACH_RPM, SENSOR_NUM_FB_14_FAN_TACH_RPM,
};

uint8_t pump_sensor_array[] = {
	SENSOR_NUM_PB_1_PUMP_TACH_RPM,
	SENSOR_NUM_PB_2_PUMP_TACH_RPM,
	SENSOR_NUM_PB_3_PUMP_TACH_RPM,
};

uint8_t hsc_communicate_sensor_array[] = {
	SENSOR_NUM_BPB_HSC_P48V_TEMP_C,	 SENSOR_NUM_BB_HSC_P48V_TEMP_C,
	SENSOR_NUM_PB_1_HSC_P48V_TEMP_C, SENSOR_NUM_PB_2_HSC_P48V_TEMP_C,
	SENSOR_NUM_PB_3_HSC_P48V_TEMP_C,
};

typedef struct {
	uint8_t duty;
	uint32_t standard_rpm;
} pump_threshold_mapping;

pump_threshold_mapping pump_threshold_tbl[] = {
	{ 8, 0 },     { 10, 300 },  { 20, 1800 }, { 30, 3250 },
	{ 40, 4450 }, { 50, 5450 }, { 60, 6050 }, { 70, 6450 },
};

/* return true when pump fail >= 2 */
bool pump_fail_check()
{
	uint8_t fail_num = 0;
	for (uint8_t i = 0; i < ARRAY_SIZE(pump_sensor_array); i++) {
		if (get_threshold_status(pump_sensor_array[i]))
			fail_num++;
	}

	if (fail_num >= 2)
		return true;

	return false;
}

/* return true when hex fan fail >= 2 */
bool hex_fan_fail_check()
{
	uint8_t fail_num = 0;
	for (uint8_t i = 0; i < ARRAY_SIZE(fan_sensor_array); i++) {
		if (get_threshold_status(fan_sensor_array[i]))
			fail_num++;
	}

	if (fail_num >= 2)
		return true;

	return false;
}

bool hsc_communicate_check()
{
	for (uint8_t i = 0; i < ARRAY_SIZE(hsc_communicate_sensor_array); i++) {
		if (get_threshold_status(hsc_communicate_sensor_array[i]) ==
		    THRESHOLD_STATUS_NOT_ACCESS)
			return true;
	}

	return false;
}

/* if leak or some threshold fault, return false */
bool system_failure_recovery()
{
	static uint8_t pre_status = 0; // 0: normal, 1: fault
	if (get_status_flag(STATUS_FLAG_LEAK)) {
		pre_status = 1;
		return false;
	}

	if (pump_fail_check()) {
		pre_status = 1;
		return false;
	}

	if (hex_fan_fail_check()) {
		pre_status = 1;
		return false;
	}

	uint8_t check_fault[] = {
		SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_P_KPA,  SENSOR_NUM_BPB_RACK_LEVEL_2,
		SENSOR_NUM_BPB_RPU_COOLANT_FLOW_RATE_LPM, SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_TEMP_C,
		SENSOR_NUM_BPB_RPU_COOLANT_INLET_TEMP_C,  SENSOR_NUM_MB_RPU_AIR_INLET_TEMP_C,
	};

	for (uint8_t i = 0; i < ARRAY_SIZE(check_fault); i++)
		if (get_threshold_status(check_fault[i])) {
			pre_status = 1;
			return false;
		}

	if (pre_status) {
		pre_status = 0;
		return true;
	}

	return false;
}

/* check status for rpu ready */
bool rpu_ready_recovery()
{
	if (get_status_flag(STATUS_FLAG_LEAK))
		return false;

	bool hsc_fail = false;
	if (!gpio_get(PWRGD_P48V_HSC_LF_R)) {
		if (!hsc_fail) {
			set_status_flag(STATUS_FLAG_FAILURE, GPIO_FAIL_BPB_HSC, 1);
			error_log_event(SENSOR_NUM_BPB_HSC_FAIL, IS_ABNORMAL_VAL);
			hsc_fail = true;
		}
	}

	const uint8_t rpu_recovery_table[] = {
		PUMP_FAIL_LOW_LEVEL,	  PUMP_FAIL_LOW_RPU_LEVEL,	PUMP_FAIL_TWO_PUMP_LCR,
		PUMP_FAIL_ABNORMAL_PRESS, PUMP_FAIL_ABNORMAL_FLOW_RATE, GPIO_FAIL_BPB_HSC,
	};

	for (uint8_t i = 0; i < ARRAY_SIZE(rpu_recovery_table); i++) {
		if ((get_status_flag(STATUS_FLAG_FAILURE) >> i) & 0x01)
			return false;
	}

	return true;
}

void fan_board_tach_status_handler(uint8_t sensor_num, uint8_t status)
{
	sensor_cfg *cfg = get_common_sensor_cfg_info(sensor_num);

	uint8_t pwrgd_read_back_val, read_fan_gok;
	uint8_t fault_offset = 0;
	uint8_t set_fault_bit_location, set_pwrgd_bit_location = 0;
	uint8_t set_fault_alert_value = 0;
	uint8_t pwrgd_gpio_offset = 0;

	// read back data from reg
	// EVT
	uint8_t stage = get_board_stage();
	if (stage == BOARD_STAGE_EVT) {
		// read back data from reg and set fault gpio
		if (!nct7363_read_back_data(cfg, NCT7363_GPIO1x_OUTPUT_PORT_REG_OFFSET,
					    &pwrgd_read_back_val)) {
			LOG_ERR("Read fan_board_pwrgd fail, read_back_val: %d",
				pwrgd_read_back_val);
			return;
		}

		if (!nct7363_read_back_data(cfg, NCT7363_GPIO0x_INPUT_PORT_REG_OFFSET,
					    &read_fan_gok)) {
			LOG_ERR("Read pump_board_pwrgd gpio fail, read_back_val: %d", read_fan_gok);
			return;
		}

		if ((read_fan_gok & BIT(2)) == 0) {
			LOG_DBG("fan gok is low");
			status = THRESHOLD_STATUS_NOT_ACCESS;
		}

		// fault
		if (status == THRESHOLD_STATUS_LCR) {
			LOG_DBG("fan THRESHOLD_STATUS_LCR");
			WRITE_BIT(pwrgd_read_back_val, 0, 0);
			if (!nct7363_write(cfg, FAN_BOARD_FAULT_PWM_OFFSET,
					   FAN_BOARD_FAULT_PWM_DUTY_50))
				LOG_ERR("Write fan_board_fault pwm fail");
		} else if (status == THRESHOLD_STATUS_NORMAL) {
			LOG_DBG("fan THRESHOLD_STATUS_NORMAL");
			WRITE_BIT(pwrgd_read_back_val, 0, 1);
			if (!nct7363_write(cfg, FAN_BOARD_FAULT_PWM_OFFSET,
					   FAN_BOARD_FAULT_PWM_DUTY_1))
				LOG_ERR("Write fan_board_fault pwm fail");
		} else if (status == THRESHOLD_STATUS_NOT_ACCESS) {
			LOG_DBG("fan THRESHOLD_STATUS_NOT_ACCESS");
			WRITE_BIT(pwrgd_read_back_val, 0, 0);
			if (!nct7363_write(cfg, FAN_BOARD_FAULT_PWM_OFFSET,
					   FAN_BOARD_FAULT_PWM_DUTY_100))
				LOG_ERR("Write fan_board_fault pwm fail");
		} else {
			LOG_ERR("Unexpected fan_board_tach_status");
		}

		LOG_DBG("LCR fault_write_in_val: %d", pwrgd_read_back_val);
		if (!nct7363_write(cfg, NCT7363_GPIO1x_OUTPUT_PORT_REG_OFFSET, pwrgd_read_back_val))
			LOG_ERR("Write fan_board_pwrgd gpio fail");
	} else if ((stage == BOARD_STAGE_DVT) || (stage == BOARD_STAGE_MP)) {
		pwrgd_gpio_offset = NCT7363_GPIO0x_OUTPUT_PORT_REG_OFFSET;
		// pwrgd and fault is in same offset
		if (!nct7363_read_back_data(cfg, pwrgd_gpio_offset, &pwrgd_read_back_val)) {
			LOG_ERR("DVT Read fan_board_pwrgd fail, read_back_val: %d",
				pwrgd_read_back_val);
			return;
		}

		if (!nct7363_read_back_data(cfg, NCT7363_GPIO0x_INPUT_PORT_REG_OFFSET,
					    &read_fan_gok)) {
			LOG_ERR("Read pump_board_pwrgd gpio fail, read_back_val: %d", read_fan_gok);
			return;
		}

		if ((read_fan_gok & BIT(2)) == 0) {
			LOG_ERR("fan gok is low");
			status = THRESHOLD_STATUS_NOT_ACCESS;
		}

		fault_offset = NCT7363_GPIO03_GPIO07_ALERT_LED_REG_OFFSET;
		set_fault_alert_value = DVT_FAN_BOARD_FAULT_BLINK_ENABLE;
		set_fault_bit_location = DVT_FAN_BOARD_FAULT_BIT_LOCATION;
		set_pwrgd_bit_location = DVT_FAN_BOARD_PWRGD_BIT_LOCATION;
		LOG_DBG("status: %d", status);
		// fault
		if (status == THRESHOLD_STATUS_LCR) {
			LOG_DBG("DVT fan THRESHOLD_STATUS_LCR");
			// set fault pwrgd gpio to 0
			WRITE_BIT(pwrgd_read_back_val, set_fault_bit_location, 0);
			WRITE_BIT(pwrgd_read_back_val, set_pwrgd_bit_location, 0);
			// set fault alert
			if (!nct7363_write(cfg, fault_offset, set_fault_alert_value))
				LOG_ERR("Write fan_board_fault pwm fail");
		} else if (status == THRESHOLD_STATUS_NORMAL) {
			LOG_DBG("DVT fan THRESHOLD_STATUS_NORMAL");
			// set fault 0
			WRITE_BIT(pwrgd_read_back_val, set_fault_bit_location, 0);
			// set pwrgd 1
			WRITE_BIT(pwrgd_read_back_val, set_pwrgd_bit_location, 1);
			// disable alert blink
			if (!nct7363_write(cfg, fault_offset, DVT_FAN_BOARD_FAULT_BLINK_DISABLE))
				LOG_ERR("Write fan_board_fault pwm fail");
		} else if (status == THRESHOLD_STATUS_NOT_ACCESS) {
			LOG_DBG("DVT fan THRESHOLD_STATUS_NOT_ACCESS");
			// set fault 1
			WRITE_BIT(pwrgd_read_back_val, set_fault_bit_location, 1);
			// set pwrgd 0
			WRITE_BIT(pwrgd_read_back_val, set_pwrgd_bit_location, 0);
			// disable alert blink
			if (!nct7363_write(cfg, fault_offset, DVT_FAN_BOARD_FAULT_BLINK_DISABLE))
				LOG_ERR("DVT Write fan_board_fault pwm fail");

		} else {
			LOG_DBG("Unexpected fan_board_tach_status");
		}

		LOG_DBG("DVT write in fan val : 0x%x", pwrgd_read_back_val);
		if (!nct7363_write(cfg, pwrgd_gpio_offset, pwrgd_read_back_val))
			LOG_ERR("DVT Write fan_board_pwrgd gpio fail");

	} else {
		LOG_ERR("fan_board_tach_status_handler error board_stage: %d", get_board_stage());
		return;
	}
}

void hex_fan_failure_do(uint32_t sensor_num, uint32_t status)
{
	fan_board_tach_status_handler(sensor_num, status);
	if (hex_fan_fail_check())
		set_status_flag(STATUS_FLAG_FAILURE, PUMP_FAIL_TWO_HEX_FAN_FAILURE, 1);
	else
		set_status_flag(STATUS_FLAG_FAILURE, PUMP_FAIL_TWO_HEX_FAN_FAILURE, 0);

	if (status == THRESHOLD_STATUS_LCR)
		error_log_event(sensor_num, IS_ABNORMAL_VAL);
}

/* flow_rate_ready_flag is flag to wait flow rate ready*/
static bool flow_rate_ready_flag = true;
static void flow_rate_ready()
{
	flow_rate_ready_flag = true;
}
K_WORK_DELAYABLE_DEFINE(flow_rate_ready_worker, flow_rate_ready);
static void reset_flow_rate_ready()
{
	if (!flow_rate_ready_flag)
		return;
	flow_rate_ready_flag = false;
	k_work_schedule(&flow_rate_ready_worker, K_SECONDS(10));
}

void pump_board_tach_status_handler(uint8_t sensor_num, uint8_t status)
{
	sensor_cfg *cfg = get_common_sensor_cfg_info(sensor_num);

	uint8_t read_back_val, read_pump_gok;
	uint8_t fault_offset = 0;
	uint8_t fault_read_val = 0;
	uint8_t set_pwrgd_bit_location = 0;
	uint8_t set_fault_bit_location = 0;
	uint8_t pwrgd_gpio_offset = 0;
	uint8_t stage = 0;

	stage = get_board_stage();

	if (stage == BOARD_STAGE_EVT) {
		pwrgd_gpio_offset = NCT7363_GPIO1x_OUTPUT_PORT_REG_OFFSET;
		fault_offset = NCT7363_GPIO1x_OUTPUT_PORT_REG_OFFSET;
		set_pwrgd_bit_location = EVT_PUMP_BOARD_PWRGD_BIT_LOCATION;
		set_fault_bit_location = EVT_PUMP_BOARD_FAULT_BIT_LOCATION;
		// read back data from reg to write fault gpio
		if (!nct7363_read_back_data(cfg, fault_offset, &fault_read_val)) {
			LOG_ERR("EVT Read pump_board_pwrgd gpio fail, fault_read_val: %d",
				fault_read_val);
			return;
		}
	} else if ((stage == BOARD_STAGE_DVT) || (stage == BOARD_STAGE_MP)) {
		pwrgd_gpio_offset = NCT7363_GPIO0x_OUTPUT_PORT_REG_OFFSET;
		fault_offset = NCT7363_GPIO1x_OUTPUT_PORT_REG_OFFSET;
		set_pwrgd_bit_location = DVT_PUMP_BOARD_PWRGD_BIT_LOCATION;
		set_fault_bit_location = DVT_PUMP_BOARD_FAULT_BIT_LOCATION;
		// read back data from reg to write fault gpio
		if (!nct7363_read_back_data(cfg, fault_offset, &fault_read_val)) {
			LOG_ERR("DVT Read pump_board_pwrgd gpio fail, fault_read_val: %d",
				fault_read_val);
			return;
		}
		LOG_DBG("fault_read_val: %d", fault_read_val);
	} else {
		LOG_ERR("pump_board_tach_status_handler error board_stage: %d", get_board_stage());
		return;
	}

	// read back data from reg to write fault gpio
	if (!nct7363_read_back_data(cfg, pwrgd_gpio_offset, &read_back_val)) {
		LOG_ERR("Read pump_board_pwrgd gpio fail, read_back_val: %d", read_back_val);
		return;
	}

	// read gok input value
	if (!nct7363_read_back_data(cfg, NCT7363_GPIO0x_INPUT_PORT_REG_OFFSET, &read_pump_gok)) {
		LOG_ERR("Read pump_board_pwrgd gpio fail, read_back_val: %d", read_pump_gok);
		return;
	}

	LOG_DBG("pump gok value: %ld", read_pump_gok & BIT(2));
	if (read_pump_gok & BIT(2))
		WRITE_BIT(read_back_val, set_pwrgd_bit_location, 1);
	else
		WRITE_BIT(read_back_val, set_pwrgd_bit_location, 0);

	LOG_DBG("pump led status : %d", status);
	if (status == THRESHOLD_STATUS_LCR) {
		LOG_DBG("pump THRESHOLD_STATUS_LCR");
		WRITE_BIT(fault_read_val, set_fault_bit_location, 1);
	} else if (status == THRESHOLD_STATUS_UCR) {
		LOG_DBG("pump THRESHOLD_STATUS_UCR");
		WRITE_BIT(fault_read_val, set_fault_bit_location, 1);
	} else if (status == THRESHOLD_STATUS_NORMAL) {
		LOG_DBG("pump THRESHOLD_STATUS_NORMAL");
		WRITE_BIT(fault_read_val, set_fault_bit_location, 0);
	} else if (status == THRESHOLD_STATUS_NOT_ACCESS) {
		LOG_DBG("pump THRESHOLD_STATUS_NOT_ACCESS");
		WRITE_BIT(fault_read_val, set_fault_bit_location, 1);
	} else
		LOG_DBG("Unexpected pump_board_tach_status");

	// write gok value
	if (!nct7363_write(cfg, pwrgd_gpio_offset, read_back_val))
		LOG_ERR("Write pump_board_pwrgd gpio fail");

	// write fault value
	if (!nct7363_write(cfg, fault_offset, fault_read_val))
		LOG_ERR("Write pump_board_fault gpio fail");
}

static bool pump_fan_fail_ctrl()
{
	bool ret = true;

	if (get_threshold_status(SENSOR_NUM_PB_1_FAN_1_TACH_RPM) &&
	    get_threshold_status(SENSOR_NUM_PB_1_FAN_2_TACH_RPM)) {
		if (plat_pwm_ctrl(PWM_DEVICE_E_PB_PUMB_1, 0))
			ret = false;
	}

	if (get_threshold_status(SENSOR_NUM_PB_2_FAN_1_TACH_RPM) &&
	    get_threshold_status(SENSOR_NUM_PB_2_FAN_2_TACH_RPM)) {
		if (plat_pwm_ctrl(PWM_DEVICE_E_PB_PUMB_2, 0))
			ret = false;
	}

	if (get_threshold_status(SENSOR_NUM_PB_3_FAN_1_TACH_RPM) &&
	    get_threshold_status(SENSOR_NUM_PB_3_FAN_2_TACH_RPM)) {
		if (plat_pwm_ctrl(PWM_DEVICE_E_PB_PUMB_3, 0))
			ret = false;
	}

	return ret;
}

void rpu_internal_fan_failure_do(uint32_t sensor_num, uint32_t status)
{
	pump_board_tach_status_handler(sensor_num, status);
	if (status == THRESHOLD_STATUS_LCR) {
		if (!pump_fan_fail_ctrl())
			LOG_ERR("threshold 0x%02x pump fan failure", sensor_num);
	} else {
		LOG_DBG("Unexpected threshold warning");
	}
}

void abnormal_press_do(uint32_t unused, uint32_t status)
{
	static bool is_abnormal = false;
	if (status == THRESHOLD_STATUS_UCR) {
		set_status_flag(STATUS_FLAG_FAILURE, PUMP_FAIL_ABNORMAL_PRESS, 1);
		if (!is_abnormal) {
			is_abnormal = true;
			error_log_event(SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_P_KPA, IS_ABNORMAL_VAL);
		}
	}
}

void abnormal_temp_do(uint32_t sensor_num, uint32_t status)
{
	uint8_t failure_status = (sensor_num == SENSOR_NUM_BPB_RPU_COOLANT_INLET_TEMP_C) ?
					 PUMP_FAIL_ABNORMAL_COOLANT_INLET_TEMP :
				 (sensor_num == SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_TEMP_C) ?
					 PUMP_FAIL_ABNORMAL_COOLANT_OUTLET_TEMP :
				 (sensor_num == SENSOR_NUM_MB_RPU_AIR_INLET_TEMP_C) ?
					 PUMP_FAIL_ABNORMAL_AIR_INLET_TEMP :
					 0xFF;

	bool save_log = (sensor_num == SENSOR_NUM_BPB_RPU_COOLANT_INLET_TEMP_C)	 ? true :
			(sensor_num == SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_TEMP_C) ? true :
										   false;

	if (status == THRESHOLD_STATUS_UCR) {
		if (save_log)
			error_log_event(sensor_num, IS_ABNORMAL_VAL);
		set_status_flag(STATUS_FLAG_FAILURE, failure_status, 1);
	} else if (status == THRESHOLD_STATUS_NOT_ACCESS) {
		if (sensor_num == SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_TEMP_C) {
			set_status_flag(STATUS_FLAG_FAILURE,
					HEX_FAN_FAIL_COOLANT_OUTLET_TEMP_NOT_ACCESS, 1);
		} else {
			set_status_flag(STATUS_FLAG_FAILURE, failure_status, 1);
		}
	} else if (status == THRESHOLD_STATUS_NORMAL) {
		set_status_flag(STATUS_FLAG_FAILURE, failure_status, 0);
		if (sensor_num == SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_TEMP_C)
			set_status_flag(STATUS_FLAG_FAILURE,
					HEX_FAN_FAIL_COOLANT_OUTLET_TEMP_NOT_ACCESS, 0);
	} else {
		LOG_DBG("Unexpected threshold warning");
	}
}

void level_sensor_do(uint32_t unused, uint32_t status)
{
	static bool is_abnormal = false;
	if (get_threshold_status(SENSOR_NUM_BPB_RACK_LEVEL_2)) {
		set_status_flag(STATUS_FLAG_FAILURE, PUMP_FAIL_LOW_LEVEL, 1);
		if (!is_abnormal) {
			is_abnormal = true;
			error_log_event(SENSOR_NUM_BPB_RACK_LEVEL_2, IS_ABNORMAL_VAL);
		}
		if (get_threshold_status(SENSOR_NUM_BPB_RACK_LEVEL_1))
			led_ctrl(LED_IDX_E_COOLANT, LED_TURN_OFF);
		else
			LOG_DBG("BPB_RACK_LEVEL_1 fail\n");
	} else {
		if (get_threshold_status(SENSOR_NUM_BPB_RACK_LEVEL_1)) {
			led_ctrl(LED_IDX_E_COOLANT, LED_START_BLINK);
		} else {
			led_ctrl(LED_IDX_E_COOLANT, LED_TURN_ON);
		}
	}
}

void rpu_level_sensor_do(uint32_t unused, uint32_t status)
{
	// EVT board does not have this level sensor
	if (get_board_stage() == BOARD_STAGE_EVT)
		return;

	static bool is_abnormal = false;
	if (get_threshold_status(SENSOR_NUM_BPB_RPU_LEVEL)) {
		set_status_flag(STATUS_FLAG_FAILURE, PUMP_FAIL_LOW_RPU_LEVEL, 1);
		if (!is_abnormal) {
			is_abnormal = true;
			error_log_event(SENSOR_NUM_BPB_RACK_LEVEL_2, IS_ABNORMAL_VAL);
		}
	}
}

void sensor_log(uint32_t sensor_num, uint32_t status)
{
	if (status == THRESHOLD_STATUS_LCR)
		error_log_event(sensor_num, IS_ABNORMAL_VAL);
	else if (status == THRESHOLD_STATUS_UCR)
		error_log_event(sensor_num, IS_ABNORMAL_VAL);
}

sensor_threshold threshold_tbl[] = {
	{ SENSOR_NUM_BPB_RPU_COOLANT_INLET_TEMP_C, THRESHOLD_ENABLE_UCR, 0, 55, abnormal_temp_do,
	  SENSOR_NUM_BPB_RPU_COOLANT_INLET_TEMP_C, 1 },
	{ SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_TEMP_C, THRESHOLD_ENABLE_UCR, 0, 55, abnormal_temp_do,
	  SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_TEMP_C, 1 },
	{ SENSOR_NUM_MB_RPU_AIR_INLET_TEMP_C, THRESHOLD_ENABLE_UCR, 0, 42, abnormal_temp_do,
	  SENSOR_NUM_MB_RPU_AIR_INLET_TEMP_C, 1 },
	{ SENSOR_NUM_BPB_HEX_WATER_INLET_TEMP_C, THRESHOLD_ENABLE_UCR, 0, 65, sensor_log,
	  SENSOR_NUM_BPB_HEX_WATER_INLET_TEMP_C, 1 },
	{ SENSOR_NUM_SB_HEX_AIR_INLET_1_TEMP_C, THRESHOLD_ENABLE_UCR, 0, 60, NULL, 0, 1 },
	{ SENSOR_NUM_SB_HEX_AIR_INLET_2_TEMP_C, THRESHOLD_ENABLE_UCR, 0, 60, NULL, 0, 1 },
	{ SENSOR_NUM_SB_HEX_AIR_INLET_3_TEMP_C, THRESHOLD_ENABLE_UCR, 0, 60, NULL, 0, 1 },
	{ SENSOR_NUM_SB_HEX_AIR_INLET_4_TEMP_C, THRESHOLD_ENABLE_UCR, 0, 60, NULL, 0, 1 },
	{ SENSOR_NUM_FB_1_HEX_OUTLET_TEMP_C, THRESHOLD_ENABLE_UCR, 0, 70, NULL, 0, 1 },
	{ SENSOR_NUM_FB_2_HEX_OUTLET_TEMP_C, THRESHOLD_ENABLE_UCR, 0, 70, NULL, 0, 1 },
	{ SENSOR_NUM_FB_3_HEX_OUTLET_TEMP_C, THRESHOLD_ENABLE_UCR, 0, 70, NULL, 0, 1 },
	{ SENSOR_NUM_FB_4_HEX_OUTLET_TEMP_C, THRESHOLD_ENABLE_UCR, 0, 70, NULL, 0, 1 },
	{ SENSOR_NUM_FB_5_HEX_OUTLET_TEMP_C, THRESHOLD_ENABLE_UCR, 0, 70, NULL, 0, 1 },
	{ SENSOR_NUM_FB_6_HEX_OUTLET_TEMP_C, THRESHOLD_ENABLE_UCR, 0, 70, NULL, 0, 1 },
	{ SENSOR_NUM_FB_7_HEX_OUTLET_TEMP_C, THRESHOLD_ENABLE_UCR, 0, 70, NULL, 0, 1 },
	{ SENSOR_NUM_FB_8_HEX_OUTLET_TEMP_C, THRESHOLD_ENABLE_UCR, 0, 70, NULL, 0, 1 },
	{ SENSOR_NUM_FB_9_HEX_OUTLET_TEMP_C, THRESHOLD_ENABLE_UCR, 0, 70, NULL, 0, 1 },
	{ SENSOR_NUM_FB_10_HEX_OUTLET_TEMP_C, THRESHOLD_ENABLE_UCR, 0, 70, NULL, 0, 1 },
	{ SENSOR_NUM_FB_11_HEX_OUTLET_TEMP_C, THRESHOLD_ENABLE_UCR, 0, 70, NULL, 0, 1 },
	{ SENSOR_NUM_FB_12_HEX_OUTLET_TEMP_C, THRESHOLD_ENABLE_UCR, 0, 70, NULL, 0, 1 },
	{ SENSOR_NUM_FB_13_HEX_OUTLET_TEMP_C, THRESHOLD_ENABLE_UCR, 0, 70, NULL, 0, 1 },
	{ SENSOR_NUM_FB_14_HEX_OUTLET_TEMP_C, THRESHOLD_ENABLE_UCR, 0, 70, NULL, 0, 1 },
	// pwm device
	{ SENSOR_NUM_FB_1_FAN_TACH_RPM, THRESHOLD_ENABLE_LCR, 200, 0, hex_fan_failure_do,
	  SENSOR_NUM_FB_1_FAN_TACH_RPM, 1 },
	{ SENSOR_NUM_FB_2_FAN_TACH_RPM, THRESHOLD_ENABLE_LCR, 200, 0, hex_fan_failure_do,
	  SENSOR_NUM_FB_2_FAN_TACH_RPM, 1 },
	{ SENSOR_NUM_FB_3_FAN_TACH_RPM, THRESHOLD_ENABLE_LCR, 200, 0, hex_fan_failure_do,
	  SENSOR_NUM_FB_3_FAN_TACH_RPM, 1 },
	{ SENSOR_NUM_FB_4_FAN_TACH_RPM, THRESHOLD_ENABLE_LCR, 200, 0, hex_fan_failure_do,
	  SENSOR_NUM_FB_4_FAN_TACH_RPM, 1 },
	{ SENSOR_NUM_FB_5_FAN_TACH_RPM, THRESHOLD_ENABLE_LCR, 200, 0, hex_fan_failure_do,
	  SENSOR_NUM_FB_5_FAN_TACH_RPM, 1 },
	{ SENSOR_NUM_FB_6_FAN_TACH_RPM, THRESHOLD_ENABLE_LCR, 200, 0, hex_fan_failure_do,
	  SENSOR_NUM_FB_6_FAN_TACH_RPM, 1 },
	{ SENSOR_NUM_FB_7_FAN_TACH_RPM, THRESHOLD_ENABLE_LCR, 200, 0, hex_fan_failure_do,
	  SENSOR_NUM_FB_7_FAN_TACH_RPM, 1 },
	{ SENSOR_NUM_FB_8_FAN_TACH_RPM, THRESHOLD_ENABLE_LCR, 200, 0, hex_fan_failure_do,
	  SENSOR_NUM_FB_8_FAN_TACH_RPM, 1 },
	{ SENSOR_NUM_FB_9_FAN_TACH_RPM, THRESHOLD_ENABLE_LCR, 200, 0, hex_fan_failure_do,
	  SENSOR_NUM_FB_9_FAN_TACH_RPM, 1 },
	{ SENSOR_NUM_FB_10_FAN_TACH_RPM, THRESHOLD_ENABLE_LCR, 200, 0, hex_fan_failure_do,
	  SENSOR_NUM_FB_10_FAN_TACH_RPM, 1 },
	{ SENSOR_NUM_FB_11_FAN_TACH_RPM, THRESHOLD_ENABLE_LCR, 200, 0, hex_fan_failure_do,
	  SENSOR_NUM_FB_11_FAN_TACH_RPM, 1 },
	{ SENSOR_NUM_FB_12_FAN_TACH_RPM, THRESHOLD_ENABLE_LCR, 200, 0, hex_fan_failure_do,
	  SENSOR_NUM_FB_12_FAN_TACH_RPM, 1 },
	{ SENSOR_NUM_FB_13_FAN_TACH_RPM, THRESHOLD_ENABLE_LCR, 200, 0, hex_fan_failure_do,
	  SENSOR_NUM_FB_13_FAN_TACH_RPM, 1 },
	{ SENSOR_NUM_FB_14_FAN_TACH_RPM, THRESHOLD_ENABLE_LCR, 200, 0, hex_fan_failure_do,
	  SENSOR_NUM_FB_14_FAN_TACH_RPM, 1 },
	{ SENSOR_NUM_PB_1_PUMP_TACH_RPM, THRESHOLD_ENABLE_BOTH, 0, 0, pump_failure_do,
	  THRESHOLD_ARG0_TABLE_INDEX, 6 },
	{ SENSOR_NUM_PB_2_PUMP_TACH_RPM, THRESHOLD_ENABLE_BOTH, 0, 0, pump_failure_do,
	  THRESHOLD_ARG0_TABLE_INDEX, 6 },
	{ SENSOR_NUM_PB_3_PUMP_TACH_RPM, THRESHOLD_ENABLE_BOTH, 0, 0, pump_failure_do,
	  THRESHOLD_ARG0_TABLE_INDEX, 6 },
	{ SENSOR_NUM_PB_1_FAN_1_TACH_RPM, THRESHOLD_ENABLE_LCR, 500, 0, rpu_internal_fan_failure_do,
	  SENSOR_NUM_PB_1_FAN_1_TACH_RPM, 1 },
	{ SENSOR_NUM_PB_1_FAN_2_TACH_RPM, THRESHOLD_ENABLE_LCR, 500, 0, rpu_internal_fan_failure_do,
	  SENSOR_NUM_PB_1_FAN_2_TACH_RPM, 1 },
	{ SENSOR_NUM_PB_2_FAN_1_TACH_RPM, THRESHOLD_ENABLE_LCR, 500, 0, rpu_internal_fan_failure_do,
	  SENSOR_NUM_PB_2_FAN_1_TACH_RPM, 1 },
	{ SENSOR_NUM_PB_2_FAN_2_TACH_RPM, THRESHOLD_ENABLE_LCR, 500, 0, rpu_internal_fan_failure_do,
	  SENSOR_NUM_PB_2_FAN_2_TACH_RPM, 1 },
	{ SENSOR_NUM_PB_3_FAN_1_TACH_RPM, THRESHOLD_ENABLE_LCR, 500, 0, rpu_internal_fan_failure_do,
	  SENSOR_NUM_PB_3_FAN_1_TACH_RPM, 1 },
	{ SENSOR_NUM_PB_3_FAN_2_TACH_RPM, THRESHOLD_ENABLE_LCR, 500, 0, rpu_internal_fan_failure_do,
	  SENSOR_NUM_PB_3_FAN_2_TACH_RPM, 1 },
	{ SENSOR_NUM_MB_FAN1_TACH_RPM, THRESHOLD_ENABLE_LCR, 500, 0, NULL, 0, 1 },
	{ SENSOR_NUM_MB_FAN2_TACH_RPM, THRESHOLD_ENABLE_LCR, 500, 0, NULL, 0, 1 },
	// hsc
	{ SENSOR_NUM_BPB_HSC_P48V_TEMP_C, THRESHOLD_DISABLE, 0, 0, NULL, 0, 1 },
	{ SENSOR_NUM_BB_HSC_P48V_TEMP_C, THRESHOLD_DISABLE, 0, 0, NULL, 0, 1 },
	{ SENSOR_NUM_PB_1_HSC_P48V_TEMP_C, THRESHOLD_DISABLE, 0, 0, NULL, 0, 1 },
	{ SENSOR_NUM_PB_2_HSC_P48V_TEMP_C, THRESHOLD_DISABLE, 0, 0, NULL, 0, 1 },
	{ SENSOR_NUM_PB_3_HSC_P48V_TEMP_C, THRESHOLD_DISABLE, 0, 0, NULL, 0, 1 },
	{ SENSOR_NUM_FB_1_HSC_TEMP_C, THRESHOLD_DISABLE, 0, 0, NULL, 0, 1 },
	{ SENSOR_NUM_FB_2_HSC_TEMP_C, THRESHOLD_DISABLE, 0, 0, NULL, 0, 1 },
	{ SENSOR_NUM_FB_3_HSC_TEMP_C, THRESHOLD_DISABLE, 0, 0, NULL, 0, 1 },
	{ SENSOR_NUM_FB_4_HSC_TEMP_C, THRESHOLD_DISABLE, 0, 0, NULL, 0, 1 },
	{ SENSOR_NUM_FB_5_HSC_TEMP_C, THRESHOLD_DISABLE, 0, 0, NULL, 0, 1 },
	{ SENSOR_NUM_FB_6_HSC_TEMP_C, THRESHOLD_DISABLE, 0, 0, NULL, 0, 1 },
	{ SENSOR_NUM_FB_7_HSC_TEMP_C, THRESHOLD_DISABLE, 0, 0, NULL, 0, 1 },
	{ SENSOR_NUM_FB_8_HSC_TEMP_C, THRESHOLD_DISABLE, 0, 0, NULL, 0, 1 },
	{ SENSOR_NUM_FB_9_HSC_TEMP_C, THRESHOLD_DISABLE, 0, 0, NULL, 0, 1 },
	{ SENSOR_NUM_FB_10_HSC_TEMP_C, THRESHOLD_DISABLE, 0, 0, NULL, 0, 1 },
	{ SENSOR_NUM_FB_11_HSC_TEMP_C, THRESHOLD_DISABLE, 0, 0, NULL, 0, 1 },
	{ SENSOR_NUM_FB_12_HSC_TEMP_C, THRESHOLD_DISABLE, 0, 0, NULL, 0, 1 },
	{ SENSOR_NUM_FB_13_HSC_TEMP_C, THRESHOLD_DISABLE, 0, 0, NULL, 0, 1 },
	{ SENSOR_NUM_FB_14_HSC_TEMP_C, THRESHOLD_DISABLE, 0, 0, NULL, 0, 1 },
	{ SENSOR_NUM_BPB_RPU_COOLANT_INLET_P_KPA, THRESHOLD_ENABLE_LCR, -50, 0, sensor_log,
	  SENSOR_NUM_BPB_RPU_COOLANT_INLET_P_KPA, 1 },
	{ SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_P_KPA, THRESHOLD_ENABLE_UCR, 0, 300, abnormal_press_do,
	  0, 1 },
	{ SENSOR_NUM_BPB_RPU_COOLANT_FLOW_RATE_LPM, THRESHOLD_ENABLE_LCR, 10, 0, abnormal_flow_do,
	  THRESHOLD_ARG0_TABLE_INDEX, 20 },
	{ SENSOR_NUM_BPB_RACK_LEVEL_1, THRESHOLD_ENABLE_LCR, 0.1, 0, level_sensor_do, 0, 1 },
	{ SENSOR_NUM_BPB_RACK_LEVEL_2, THRESHOLD_ENABLE_LCR, 0.1, 0, level_sensor_do, 0, 1 },
	{ SENSOR_NUM_BPB_RPU_LEVEL, THRESHOLD_ENABLE_LCR, 0.1, 0, rpu_level_sensor_do, 0, 5 },
	{ SENSOR_NUM_FAN_PRSNT, THRESHOLD_ENABLE_DISCRETE, 0, 0, fb_prsnt_handle,
	  THRESHOLD_ARG0_TABLE_INDEX, 1 },
	{ SENSOR_NUM_HEX_EXTERNAL_Y_FILTER, THRESHOLD_ENABLE_UCR, 0, 30, sensor_log,
	  SENSOR_NUM_HEX_EXTERNAL_Y_FILTER, 1 },
	{ SENSOR_NUM_BPB_RACK_PRESSURE_3_P_KPA, THRESHOLD_ENABLE_UCR, 0, 300, sensor_log,
	  SENSOR_NUM_BPB_RACK_PRESSURE_3_P_KPA, 1 },
	{ SENSOR_NUM_BPB_RACK_PRESSURE_4_P_KPA, THRESHOLD_ENABLE_UCR, 0, 300, sensor_log,
	  SENSOR_NUM_BPB_RACK_PRESSURE_4_P_KPA, 1 },
	{ SENSOR_NUM_SB_HEX_PRESSURE_1_P_KPA, THRESHOLD_ENABLE_UCR, 0, 200, sensor_log,
	  SENSOR_NUM_SB_HEX_PRESSURE_1_P_KPA, 1 },
	{ SENSOR_NUM_SB_HEX_PRESSURE_2_P_KPA, THRESHOLD_ENABLE_UCR, 0, 200, sensor_log,
	  SENSOR_NUM_SB_HEX_PRESSURE_2_P_KPA, 1 },
	{ SENSOR_NUM_SB_HEX_AIR_INLET_AVG_TEMP_C, THRESHOLD_ENABLE_BOTH, -20, 100, sensor_log,
	  SENSOR_NUM_SB_HEX_AIR_INLET_AVG_TEMP_C, 1 },

};

sensor_threshold *find_threshold_tbl_entry(uint8_t sensor_num)
{
	for (uint8_t i = 0; i < ARRAY_SIZE(threshold_tbl); i++) {
		sensor_threshold *p = threshold_tbl + i;
		if (p->sensor_num == sensor_num)
			return p;
	}

	return NULL;
}

void pump_failure_do(uint32_t thres_tbl_idx, uint32_t status)
{
	if (thres_tbl_idx >= ARRAY_SIZE(threshold_tbl))
		return;

	sensor_threshold *thres_p = &threshold_tbl[thres_tbl_idx];
	uint32_t sensor_num = thres_p->sensor_num;

	uint8_t pump_ucr = (sensor_num == SENSOR_NUM_PB_1_PUMP_TACH_RPM) ? PUMP_FAIL_PUMP1_UCR :
			   (sensor_num == SENSOR_NUM_PB_2_PUMP_TACH_RPM) ? PUMP_FAIL_PUMP2_UCR :
			   (sensor_num == SENSOR_NUM_PB_3_PUMP_TACH_RPM) ? PUMP_FAIL_PUMP3_UCR :
									   FAILURE_STATUS_MAX;

	switch (status) {
	case THRESHOLD_STATUS_LCR:
		error_log_event(sensor_num, IS_ABNORMAL_VAL);
		if (pump_fail_check())
			set_status_flag(STATUS_FLAG_FAILURE, PUMP_FAIL_TWO_PUMP_LCR, 1);
		break;
	case THRESHOLD_STATUS_UCR:
		set_status_flag(STATUS_FLAG_FAILURE, pump_ucr, 1);
		LOG_ERR("threshold 0x%02x pump ucr failure", sensor_num);
		error_log_event(sensor_num, IS_ABNORMAL_VAL);
		break;
	case THRESHOLD_STATUS_NORMAL:
		reset_flow_rate_ready();
		set_status_flag(STATUS_FLAG_FAILURE, pump_ucr, 0);
		error_log_event(sensor_num, IS_NORMAL_VAL);
		break;
	default:
		LOG_DBG("Unexpected threshold warning");
		break;
	}

	pump_board_tach_status_handler(sensor_num, status);
}

/* return true means pump 1/2/3 not access or tach too low */
static bool check_pump_tach_too_low()
{
	static uint8_t retry[3] = { 0 };

	for (uint8_t i = 0; i < ARRAY_SIZE(pump_sensor_array); i++) {
		float tmp = 0;
		if (get_sensor_reading_to_real_val(pump_sensor_array[i], &tmp) !=
		    SENSOR_READ_4BYTE_ACUR_SUCCESS)
			return true;

		if (tmp >= 500) {
			if (retry[i] < 10) {
				retry[i]++;
				return false;
			} else {
				retry[i] = 0;
				return true;
			}
		}
	}

	memset(retry, 0, 3);
	return false;
}

void abnormal_flow_do(uint32_t thres_tbl_idx, uint32_t status)
{
	if (thres_tbl_idx >= ARRAY_SIZE(threshold_tbl))
		return;

	sensor_threshold *thres_p = &threshold_tbl[thres_tbl_idx];

	if (status == THRESHOLD_STATUS_LCR) {
		if (!check_pump_tach_too_low()) {
			thres_p->last_status = THRESHOLD_STATUS_NORMAL;
			reset_flow_rate_ready();
			return;
		}
		if (!flow_rate_ready_flag) {
			thres_p->last_status = THRESHOLD_STATUS_NORMAL;
			return;
		}
		set_status_flag(STATUS_FLAG_FAILURE, PUMP_FAIL_ABNORMAL_FLOW_RATE, 1);
		error_log_event(SENSOR_NUM_BPB_RPU_COOLANT_FLOW_RATE_LPM, IS_ABNORMAL_VAL);
	} else if (status == THRESHOLD_STATUS_NOT_ACCESS) {
		set_status_flag(STATUS_FLAG_FAILURE, PUMP_FAIL_FLOW_RATE_NOT_ACCESS, 1);
	} else if (status == THRESHOLD_STATUS_NORMAL) {
		set_status_flag(STATUS_FLAG_FAILURE, PUMP_FAIL_FLOW_RATE_NOT_ACCESS, 0);
	} else {
		LOG_DBG("Unexpected threshold warning");
	}
}

#define SENSOR_NUM(type, index) SENSOR_NUM_FB_##index##_##type
#define FB_SEN_ENTRY(index)                                                                        \
	{                                                                                          \
		{                                                                                  \
			SENSOR_NUM(FAN_TACH_RPM, index), SENSOR_NUM(HSC_TEMP_C, index),            \
				SENSOR_NUM(HEX_OUTLET_TEMP_C, index)                               \
		}                                                                                  \
	}

#define FB_REINIT_SEN_COUNT 3
struct fb_sen_entry {
	uint8_t sen_num[FB_REINIT_SEN_COUNT];
} fb_sen_tbl[] = { FB_SEN_ENTRY(1),  FB_SEN_ENTRY(2),  FB_SEN_ENTRY(3),	 FB_SEN_ENTRY(4),
		   FB_SEN_ENTRY(5),  FB_SEN_ENTRY(6),  FB_SEN_ENTRY(7),	 FB_SEN_ENTRY(8),
		   FB_SEN_ENTRY(9),  FB_SEN_ENTRY(10), FB_SEN_ENTRY(11), FB_SEN_ENTRY(12),
		   FB_SEN_ENTRY(13), FB_SEN_ENTRY(14) };

static uint8_t fb_uninit(uint8_t idx)
{
	LOG_ERR("fb_uninit %d", idx + 1);

	if (idx >= HEX_FAN_NUM)
		return 1;

	for (uint8_t i = 0; i < FB_REINIT_SEN_COUNT; i++) {
		sensor_cfg *cfg = get_common_sensor_cfg_info(fb_sen_tbl[idx].sen_num[i]);
		if (!cfg)
			continue;

		switch (cfg->type) {
		case sensor_dev_adm1272: {
			adm1272_init_arg *arg_p = (adm1272_init_arg *)cfg->init_args;
			arg_p->is_init = false;
			break;
		}
		case sensor_dev_nct7363: {
			nct7363_init_arg *arg_p = (nct7363_init_arg *)cfg->init_args;
			arg_p->is_init = false;
			break;
		}
		case sensor_dev_hdc1080: {
			hdc1080_init_arg *arg_p = (hdc1080_init_arg *)cfg->init_args;
			arg_p->is_init = false;
			break;
		}
		default:
			LOG_ERR("fb_uninit unsupport sensor type %d", cfg->type);
			break;
		}
	}

	return 0;
}

static uint8_t fb_reinit(uint8_t idx)
{
	LOG_ERR("fb_reinit %d", idx + 1);

	if (idx >= HEX_FAN_NUM)
		return 1;

	for (uint8_t i = 0; i < FB_REINIT_SEN_COUNT; i++) {
		if (common_tbl_sen_reinit(fb_sen_tbl[idx].sen_num[i]))
			LOG_ERR("Reinit sensor %d fail", fb_sen_tbl[idx].sen_num[i]);
	}

	return 0;
}

static void fb_prsnt_handle(uint32_t thres_tbl_idx, uint32_t changed_status)
{
	LOG_ERR("thres_tbl_idx %d, fb_prsnt_handle %x", thres_tbl_idx, changed_status);

	if (thres_tbl_idx >= ARRAY_SIZE(threshold_tbl))
		return;

	sensor_threshold *thres_p = &threshold_tbl[thres_tbl_idx];

	for (uint8_t i = 0; i < HEX_FAN_NUM; i++) {
		if (!(changed_status & BIT(i)))
			continue;

		if (!(thres_p->last_value & BIT(i))) {
			LOG_ERR("Fan board %d is not present", i + 1);
			fb_uninit(i);
			thres_p->last_status = THRESHOLD_STATUS_NORMAL;
			continue;
		}

		LOG_ERR("Fan board %d is present", i + 1);
		fb_reinit(i);
	}
}

/*
	return 0 is hsc power ok
	fan board idx from 0 to 13
*/
uint8_t fb_hsc_status(uint8_t idx)
{
	if (idx >= HEX_FAN_NUM)
		return 1;

	uint8_t data = 0;
	sensor_cfg *cfg = get_common_sensor_cfg_info(fan_sensor_array[idx]);
	if (!nct7363_read_back_data(cfg, NCT7363_GPIO0x_INPUT_PORT_REG_OFFSET, &data))
		return 1;

	return ((data & BIT(2)) ? 0 : 1);
}

/* threshold poll relate */
void set_threshold_poll_enable_flag(bool flag)
{
	threshold_poll_enable_flag = flag;
}

bool get_threshold_poll_enable_flag()
{
	return threshold_poll_enable_flag;
}

uint32_t get_threshold_status(uint8_t sensor_num)
{
	// This is for DVT work around, it will be removed after hardware design fixed
	if ((get_board_stage() == BOARD_STAGE_DVT) && (sensor_num == SENSOR_NUM_BPB_RPU_LEVEL))
		return 0;

	for (uint8_t i = 0; i < ARRAY_SIZE(threshold_tbl); i++)
		if (threshold_tbl[i].sensor_num == sensor_num)
			return threshold_tbl[i].last_status;

	return THRESHOLD_STATUS_UNKNOWN;
}

/*
	check whether the status has changed
	If type is LCR, and UCR occurs, the status will still be normal.
 */
static bool set_threshold_status(sensor_threshold *threshold_tbl, float val)
{
	uint32_t status = THRESHOLD_STATUS_NORMAL;

	switch (threshold_tbl->type) {
	case THRESHOLD_DISABLE:
		break;
	case THRESHOLD_ENABLE_LCR:
		if (val < threshold_tbl->lcr)
			status = THRESHOLD_STATUS_LCR;
		break;
	case THRESHOLD_ENABLE_UCR:
		if (val > threshold_tbl->ucr)
			status = THRESHOLD_STATUS_UCR;
		break;
	case THRESHOLD_ENABLE_BOTH:
		if (val < threshold_tbl->lcr)
			status = THRESHOLD_STATUS_LCR;
		else if (val > threshold_tbl->ucr)
			status = THRESHOLD_STATUS_UCR;
		break;
	default:
		LOG_ERR("set status error type %d", threshold_tbl->type);
		return false;
	}

	if (threshold_tbl->last_status == status) {
		threshold_tbl->retry_count = 0;
		return false;
	}

	// retry
	threshold_tbl->retry_count++;
	if (threshold_tbl->retry_count < threshold_tbl->retry)
		return false;
	else
		threshold_tbl->retry_count = 0;

	LOG_WRN("0x%02x status change: last_status: %d, status: %d, val: %d\n",
		threshold_tbl->sensor_num, threshold_tbl->last_status, status, (int)val);

	threshold_tbl->last_status = status;
	return true;
}

void threshold_poll_init()
{
	static uint8_t is_init = 0;

	if (!is_init) {
		uint8_t stage = 0;
		stage = get_board_stage();
		LOG_INF("threshold_poll_init read board stage %d", stage);

		// check board stage and set threshold
		for (uint8_t i = 0; i < ARRAY_SIZE(threshold_tbl); i++) {
			if (stage == BOARD_STAGE_EVT) {
				// level3 sensor disable
				if (threshold_tbl[i].sensor_num == SENSOR_NUM_BPB_RPU_LEVEL)
					threshold_tbl[i].type = THRESHOLD_DISABLE;
			} else if ((stage == BOARD_STAGE_DVT) || (stage == BOARD_STAGE_MP)) {
				LOG_DBG("DVT board stage %d", stage);
			} else {
				LOG_ERR("unknown board stage %d", stage);
			}
		}

		fsc_init();
		// default set coolant led on
		led_ctrl(LED_IDX_E_COOLANT, LED_TURN_ON);
		k_msleep(2000); // wait 2s for pump run
		LOG_INF("threshold thread start!");
		is_init = 1;
		set_status_flag(STATUS_FLAG_SYSTEM_READY, 0, 1);
		set_all_rpu_ready_pin_normal();
	}
}

static void yfilter_change_threshold()
{
	float val = 0.0;
	if (get_sensor_reading_to_real_val(SENSOR_NUM_BPB_RPU_COOLANT_FLOW_RATE_LPM, &val) !=
	    SENSOR_READ_4BYTE_ACUR_SUCCESS)
		return;

	sensor_threshold *p = find_threshold_tbl_entry(SENSOR_NUM_HEX_EXTERNAL_Y_FILTER);
	if (p == NULL)
		return;
	if (val >= 60)
		p->ucr = 30;
	else
		p->ucr = 20;
}

static uint32_t get_pump_standard_rpm(uint8_t duty)
{
	if (duty < pump_threshold_tbl[0].duty) {
		return pump_threshold_tbl[0].standard_rpm;
	}

	if (duty >= pump_threshold_tbl[ARRAY_SIZE(pump_threshold_tbl) - 1].duty) {
		return pump_threshold_tbl[ARRAY_SIZE(pump_threshold_tbl) - 1].standard_rpm;
	}

	for (uint8_t i = 1; i < ARRAY_SIZE(pump_threshold_tbl); i++) {
		if (duty >= pump_threshold_tbl[i - 1].duty && duty < pump_threshold_tbl[i].duty) {
			uint32_t lower_duty = pump_threshold_tbl[i - 1].duty;
			uint32_t upper_duty = pump_threshold_tbl[i].duty;
			uint32_t lower_rpm = pump_threshold_tbl[i - 1].standard_rpm;
			uint32_t upper_rpm = pump_threshold_tbl[i].standard_rpm;

			uint32_t rpm = lower_rpm + (upper_rpm - lower_rpm) * (duty - lower_duty) /
							   (upper_duty - lower_duty);
			return rpm;
		}
	}

	return 0;
}

void pump_change_threshold(uint8_t sensor_num, uint8_t duty)
{
	// don't change when pump failure
	if (get_threshold_status(sensor_num))
		return;

	// don't change for sit test
	if (get_status_flag(STATUS_FLAG_DEBUG_MODE) & BIT(DEBUG_MODE_PUMP_THRESHOLD))
		return;

	sensor_threshold *p = find_threshold_tbl_entry(sensor_num);
	if (p == NULL)
		return;

	uint32_t standard_val = get_pump_standard_rpm(duty);
	p->lcr = standard_val * 0.75;
	p->ucr = standard_val * 1.25;
}

void check_bpb_hsc_status(void)
{
	static bool hsc_fail = false;
	if (!gpio_get(PWRGD_P48V_HSC_LF_R)) {
		if (!hsc_fail) {
			set_status_flag(STATUS_FLAG_FAILURE, GPIO_FAIL_BPB_HSC, 1);
			error_log_event(SENSOR_NUM_BPB_HSC_FAIL, IS_ABNORMAL_VAL);
			hsc_fail = true;
		}
	}
}

void plat_sensor_poll_post()
{
	int64_t current_time = k_uptime_get();
	// if current time less than 20s, do not poll threshold
	if (current_time < 48000)
		return;

	threshold_poll_init();

	if (!get_sensor_init_done_flag())
		return;

	if (!get_sensor_poll_enable_flag())
		return;

	if (!get_threshold_poll_enable_flag())
		return;

	yfilter_change_threshold();

	for (uint8_t i = 0; i < ARRAY_SIZE(threshold_tbl); i++) {
		float val = 0;

		if (threshold_tbl[i].type == THRESHOLD_ENABLE_DISCRETE) {
			/* check the discrete sensor has value changed */
			/* do not need to convert the sensor reading value */

			int reading = 0;
			uint8_t status = get_sensor_reading(sensor_config, sensor_config_count,
							    threshold_tbl[i].sensor_num, &reading,
							    GET_FROM_CACHE);

			if (status != SENSOR_READ_4BYTE_ACUR_SUCCESS) {
				LOG_ERR("0x%02x get sensor cache fail",
					threshold_tbl[i].sensor_num);
				continue;
			}

			if (threshold_tbl[i].last_value == reading)
				continue;

			LOG_ERR("threshold_tbl[i].last_value = %x, reading = %x",
				threshold_tbl[i].last_value, reading);

			/* use the last_status to carry change bits */
			threshold_tbl[i].last_status = threshold_tbl[i].last_value ^ reading;
			threshold_tbl[i].last_value = reading;

		} else {
			if (get_sensor_reading_to_real_val(threshold_tbl[i].sensor_num, &val) !=
			    SENSOR_READ_4BYTE_ACUR_SUCCESS)
				threshold_tbl[i].last_status = THRESHOLD_STATUS_NOT_ACCESS;
			/* check whether the status has changed */
			else if (!set_threshold_status(&threshold_tbl[i], val))
				continue;
		}

		if (threshold_tbl[i].fn) {
			uint32_t arg0 = threshold_tbl[i].arg0;
			if (threshold_tbl[i].arg0 == THRESHOLD_ARG0_TABLE_INDEX)
				arg0 = i;

			threshold_tbl[i].fn(arg0, threshold_tbl[i].last_status);
		}
	}

	// check bpb hsc
	check_bpb_hsc_status();

	// control fault led
	fault_led_control();

	if (!rpu_ready_recovery())
		deassert_all_rpu_ready_pin();

	return;
}

void fan_pump_pwrgd_handler(void *arug0, void *arug1, void *arug2)
{
	ARG_UNUSED(arug0);
	ARG_UNUSED(arug1);
	ARG_UNUSED(arug2);

	int fan_pump_pwrgd_handler_interval_ms = 1000; // interval 1s

	while (1) {
		if (!get_sensor_init_done_flag()) {
			k_msleep(fan_pump_pwrgd_handler_interval_ms);
			continue;
		}

		if (!get_sensor_poll_enable_flag()) {
			k_msleep(fan_pump_pwrgd_handler_interval_ms);
			continue;
		}

		for (uint8_t i = 0; i < ARRAY_SIZE(fan_pump_sensor_array); i++) {
			sensor_cfg *cfg = get_common_sensor_cfg_info(fan_pump_sensor_array[i]);

			if (cfg->cache_status == SENSOR_UNSPECIFIED_ERROR)
				continue;

			uint8_t read_gok = 0;
			// read gok data
			if (!nct7363_read_back_data(cfg, NCT7363_GPIO0x_INPUT_PORT_REG_OFFSET,
						    &read_gok))
				LOG_ERR("Read pump_board_pwrgd gpio fail, read_back_val: %d",
					read_gok);

			uint8_t pwrgd_read_back_val = 0;

			// check stage is EVT
			uint8_t stage = get_board_stage();
			if (stage == BOARD_STAGE_EVT) {
				if (!nct7363_read_back_data(cfg,
							    NCT7363_GPIO1x_OUTPUT_PORT_REG_OFFSET,
							    &pwrgd_read_back_val))
					LOG_ERR("EVT Thread read val: %d", pwrgd_read_back_val);

				if (!(read_gok & BIT(2))) {
					WRITE_BIT(pwrgd_read_back_val,
						  EVT_FAN_BOARD_PWRGD_BIT_LOCATION, 0);
					if (!nct7363_write(cfg, FAN_BOARD_FAULT_PWM_OFFSET,
							   FAN_BOARD_FAULT_PWM_DUTY_100))
						LOG_ERR("EVT Write fan_board_fault pwm fail");
				}

				if (!nct7363_write(cfg, NCT7363_GPIO1x_OUTPUT_PORT_REG_OFFSET,
						   pwrgd_read_back_val))
					LOG_ERR("EVT Write pump_board_pwrgd gpio fail");
			} else if ((stage == BOARD_STAGE_DVT) || (stage == BOARD_STAGE_MP)) {
				// DVT
				if (!nct7363_read_back_data(cfg,
							    NCT7363_GPIO0x_OUTPUT_PORT_REG_OFFSET,
							    &pwrgd_read_back_val))
					LOG_ERR("DVT Thread read val: %d", pwrgd_read_back_val);

				if (!(read_gok & BIT(2))) {
					if (i < 14) {
						if (read_gok &
						    BIT(DVT_FAN_BOARD_FAULT_BIT_LOCATION))
							WRITE_BIT(pwrgd_read_back_val,
								  DVT_FAN_BOARD_PWRGD_BIT_LOCATION,
								  0); // fan board pwrgd
					} else
						WRITE_BIT(pwrgd_read_back_val,
							  DVT_PUMP_BOARD_PWRGD_BIT_LOCATION,
							  0); // pump board pwrgd

					if (!nct7363_write(cfg, FAN_BOARD_FAULT_PWM_OFFSET,
							   FAN_BOARD_FAULT_PWM_DUTY_100))
						LOG_ERR("DVT Write fan_board_fault pwm fail");
				}

				if (!nct7363_write(cfg, NCT7363_GPIO0x_OUTPUT_PORT_REG_OFFSET,
						   pwrgd_read_back_val))
					LOG_ERR("DVT Write pump_board_pwrgd gpio fail");
			} else {
				LOG_ERR("Unknown board stage: %d", get_board_stage());
			}
		}

		k_msleep(1000);
	}
}

void fan_pump_pwrgd()
{
	k_thread_create(&fan_pump_pwrgd_thread, fan_pump_pwrgd_stack,
			K_THREAD_STACK_SIZEOF(fan_pump_pwrgd_stack), fan_pump_pwrgd_handler, NULL,
			NULL, NULL, CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&threshold_poll, "threshold_poll");
	return;
}
