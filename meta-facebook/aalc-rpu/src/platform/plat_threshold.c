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

#define THRESHOLD_POLL_STACK_SIZE 2048
#define FAN_PUMP_PWRGD_STACK_SIZE 2048

#define ENABLE_REDUNDANCY 1
#define DISABLE_REDUNDANCY 0

#define READ_ERROR -1
#define FAN_BOARD_FAULT_PWM_OFFSET 0xA2
#define FAN_BOARD_FAULT_PWM_DUTY_1 0x00
#define FAN_BOARD_FAULT_PWM_DUTY_50 0x7F
#define FAN_BOARD_FAULT_PWM_DUTY_100 0xFF

#define FAN_PUMP_TRACKING_STATUS 0xFF

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
	// pump fan
	SENSOR_NUM_PB_1_FAN_1_TACH_RPM,
	SENSOR_NUM_PB_1_FAN_2_TACH_RPM,
	SENSOR_NUM_PB_2_FAN_1_TACH_RPM,
	SENSOR_NUM_PB_2_FAN_2_TACH_RPM,
	SENSOR_NUM_PB_3_FAN_1_TACH_RPM,
	SENSOR_NUM_PB_3_FAN_2_TACH_RPM,
};

/* if leak or some threshold fault, return false */
bool pump_status_recovery()
{
	if (get_leak_status())
		return false;

	uint8_t check_fault[] = {
		SENSOR_NUM_PB_1_FAN_1_TACH_RPM,		 SENSOR_NUM_PB_1_FAN_2_TACH_RPM,
		SENSOR_NUM_PB_2_FAN_1_TACH_RPM,		 SENSOR_NUM_PB_2_FAN_2_TACH_RPM,
		SENSOR_NUM_PB_3_FAN_1_TACH_RPM,		 SENSOR_NUM_PB_3_FAN_2_TACH_RPM,
		SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_P_KPA, SENSOR_NUM_BPB_RACK_LEVEL_2
	};

	for (uint8_t i = 0; i < ARRAY_SIZE(check_fault); i++)
		if (get_threshold_status(check_fault[i]))
			return false;

	return true;
}

/* check status without pump status for rpu ready */
bool rpu_ready_recovery()
{
	uint8_t check_fault[] = {
		SENSOR_NUM_PB_1_PUMP_TACH_RPM,
		SENSOR_NUM_PB_2_PUMP_TACH_RPM,
		SENSOR_NUM_PB_3_PUMP_TACH_RPM,
		SENSOR_NUM_BPB_RPU_COOLANT_INLET_TEMP_C,
		SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_TEMP_C,
		SENSOR_NUM_BPB_RPU_COOLANT_FLOW_RATE_LPM,
	};

	for (uint8_t i = 0; i < ARRAY_SIZE(check_fault); i++)
		if (get_threshold_status(check_fault[i]))
			return false;

	return true;
}

void fan_board_tach_status_handler(uint8_t sensor_num, uint8_t status)
{
	sensor_cfg *cfg = get_common_sensor_cfg_info(sensor_num);

	uint8_t pwrgd_read_back_val, read_fan_gok;
	// read back data from reg
	pwrgd_read_back_val = nct7363_read_back_data(cfg, NCT7363_GPIO1x_OUTPUT_PORT_REG_OFFSET);
	if (pwrgd_read_back_val == READ_ERROR) {
		LOG_ERR("Read fan_board_pwrgd fail, read_back_val: %d", pwrgd_read_back_val);
		return;
	}

	read_fan_gok = nct7363_read_back_data(cfg, NCT7363_GPIO0x_INPUT_PORT_REG_OFFSET);
	if (read_fan_gok == READ_ERROR) {
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
		if (!nct7363_write(cfg, FAN_BOARD_FAULT_PWM_OFFSET, FAN_BOARD_FAULT_PWM_DUTY_50))
			LOG_ERR("Write fan_board_fault pwm fail");
	} else if (status == THRESHOLD_STATUS_NORMAL) {
		LOG_DBG("fan THRESHOLD_STATUS_NORMAL");
		WRITE_BIT(pwrgd_read_back_val, 0, 1);
		if (!nct7363_write(cfg, FAN_BOARD_FAULT_PWM_OFFSET, FAN_BOARD_FAULT_PWM_DUTY_1))
			LOG_ERR("Write fan_board_fault pwm fail");
	} else if (status == THRESHOLD_STATUS_NOT_ACCESS) {
		LOG_DBG("fan THRESHOLD_STATUS_NOT_ACCESS");
		WRITE_BIT(pwrgd_read_back_val, 0, 0);
		if (!nct7363_write(cfg, FAN_BOARD_FAULT_PWM_OFFSET, FAN_BOARD_FAULT_PWM_DUTY_100))
			LOG_ERR("Write fan_board_fault pwm fail");
	} else {
		LOG_ERR("Unexpected fan_board_tach_status");
	}

	LOG_DBG("LCR pwrgd_write_in_val: %d", pwrgd_read_back_val);
	if (!nct7363_write(cfg, NCT7363_GPIO1x_OUTPUT_PORT_REG_OFFSET, pwrgd_read_back_val))
		LOG_ERR("Write fan_board_pwrgd gpio fail");
}

void pump_board_tach_status_handler(uint8_t sensor_num, uint8_t status)
{
	sensor_cfg *cfg = get_common_sensor_cfg_info(sensor_num);

	uint8_t read_back_val, read_pump_gok;
	// read back data from reg to write fault gpio
	read_back_val = nct7363_read_back_data(cfg, NCT7363_GPIO1x_OUTPUT_PORT_REG_OFFSET);
	if (read_back_val == READ_ERROR && status != FAN_PUMP_TRACKING_STATUS) {
		LOG_ERR("Read pump_board_pwrgd gpio fail, read_back_val: %d", read_back_val);
		return;
	}

	// read gok input value
	read_pump_gok = nct7363_read_back_data(cfg, NCT7363_GPIO0x_INPUT_PORT_REG_OFFSET);
	if (read_pump_gok == READ_ERROR && status != FAN_PUMP_TRACKING_STATUS) {
		LOG_ERR("Read pump_board_pwrgd gpio fail, read_back_val: %d", read_pump_gok);
		return;
	}

	LOG_DBG("pump gok value: %ld", read_pump_gok & BIT(2));
	if (read_pump_gok & BIT(2))
		WRITE_BIT(read_back_val, 0, 1);
	else
		WRITE_BIT(read_back_val, 0, 0);

	LOG_DBG("pump led status : %d", status);
	if (status == THRESHOLD_STATUS_LCR) {
		LOG_DBG("pump THRESHOLD_STATUS_LCR");
		WRITE_BIT(read_back_val, 1, 1);
	} else if (status == THRESHOLD_STATUS_NORMAL) {
		LOG_DBG("pump THRESHOLD_STATUS_NORMAL");
		WRITE_BIT(read_back_val, 1, 0);
	} else if (status == THRESHOLD_STATUS_NOT_ACCESS) {
		LOG_DBG("pump THRESHOLD_STATUS_NOT_ACCESS");
		WRITE_BIT(read_back_val, 1, 1);
	} else
		LOG_DBG("Unexpected pump_board_tach_status");

	if (!nct7363_write(cfg, NCT7363_GPIO1x_OUTPUT_PORT_REG_OFFSET, read_back_val))
		LOG_ERR("Write pump_board_pwrgd gpio fail");
}

void pump_failure_do(uint8_t sensor_num, uint8_t status)
{
	pump_board_tach_status_handler(sensor_num, status);
	if (status == THRESHOLD_STATUS_LCR) {
		//assert pump fault status bit
		deassert_all_rpu_ready_pin();
		//auto control for Hex Fan
		error_log_event(sensor_num, IS_ABNORMAL_VAL);
		//wait 30 secs to shut down
	} else if (status == THRESHOLD_STATUS_NORMAL) {
		error_log_event(sensor_num, IS_NORMAL_VAL);
	} else
		LOG_DBG("Unexpected threshold warning");
}

void rpu_internal_fan_failure_do(uint8_t sensor_num, uint8_t status)
{
	pump_board_tach_status_handler(sensor_num, status);
	if (status == THRESHOLD_STATUS_LCR) {
		//assert internal fan fault status bit
		deassert_all_rpu_ready_pin();
		set_pwm_group(PWM_GROUP_E_PUMP, 0); //turn off pump
		//auto control for Hex Fan
	} else {
		LOG_DBG("Unexpected threshold warning");
	}
}

void aalc_leak_detect_do(uint8_t arg0, uint8_t arg1, uint8_t status)
{
	if (status == THRESHOLD_STATUS_LCR)
		aalc_leak_behavior(arg0);
	else if (status != THRESHOLD_STATUS_NORMAL)
		LOG_DBG("Unexpected threshold warning");
}

void abnormal_press_do(uint8_t unused, uint8_t status)
{
	if (status == THRESHOLD_STATUS_UCR) {
		set_pwm_group(PWM_GROUP_E_PUMP, 0); //turn off pump
		//auto control for hex fan
		deassert_all_rpu_ready_pin();
	} else {
		LOG_DBG("Unexpected threshold warning");
	}
}

void low_level_do(uint8_t unused, uint8_t status)
{
	if (status == THRESHOLD_ENABLE_LCR) {
		//assert fluid level sensor status bit
		set_pwm_group(PWM_GROUP_E_PUMP, 0); //turn off pump
		deassert_all_rpu_ready_pin();
		error_log_event(SENSOR_NUM_BPB_RACK_LEVEL_2, IS_ABNORMAL_VAL);
		led_ctrl(LED_IDX_E_COOLANT, LED_STOP_BLINK);
		led_ctrl(LED_IDX_E_COOLANT, LED_TURN_OFF);
	} else if (status == THRESHOLD_STATUS_NORMAL) {
		if (get_led_status(LED_IDX_E_COOLANT) != LED_START_BLINK)
			led_ctrl(LED_IDX_E_COOLANT, LED_START_BLINK);
	} else
		LOG_DBG("Unexpected threshold warning");
}

void high_level_do(uint8_t unused, uint8_t status)
{
	if (status == THRESHOLD_ENABLE_LCR) {
		if (get_led_status(LED_IDX_E_COOLANT) != LED_START_BLINK)
			led_ctrl(LED_IDX_E_COOLANT, LED_START_BLINK);
	} else if (status == THRESHOLD_STATUS_NORMAL) {
		led_ctrl(LED_IDX_E_COOLANT, LED_STOP_BLINK);
		led_ctrl(LED_IDX_E_COOLANT, LED_TURN_ON);
	} else
		LOG_DBG("Unexpected threshold warning");
}

void high_coolant_temp_do(uint8_t unused, uint8_t status)
{
	if (status == THRESHOLD_STATUS_UCR)
		deassert_all_rpu_ready_pin();
}

void flow_trigger_do(uint8_t unused, uint8_t status)
{
	if (status == THRESHOLD_STATUS_LCR)
		deassert_all_rpu_ready_pin();
}

sensor_threshold threshold_tbl[] = {
	{ SENSOR_NUM_BPB_RPU_COOLANT_INLET_TEMP_C, THRESHOLD_ENABLE_UCR, 0, 65,
	  high_coolant_temp_do, 0 },
	{ SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_TEMP_C, THRESHOLD_ENABLE_UCR, 0, 65,
	  high_coolant_temp_do, 0 },
	{ SENSOR_NUM_MB_RPU_AIR_INLET_TEMP_C, THRESHOLD_ENABLE_UCR, 0, 40, NULL, 0 },
	{ SENSOR_NUM_BPB_HEX_WATER_INLET_TEMP_C, THRESHOLD_ENABLE_UCR, 0, 65, NULL, 0 },
	{ SENSOR_NUM_SB_HEX_AIR_INLET_1_TEMP_C, THRESHOLD_ENABLE_UCR, 0, 60, NULL, 0 },
	{ SENSOR_NUM_SB_HEX_AIR_INLET_2_TEMP_C, THRESHOLD_ENABLE_UCR, 0, 60, NULL, 0 },
	{ SENSOR_NUM_SB_HEX_AIR_INLET_3_TEMP_C, THRESHOLD_ENABLE_UCR, 0, 60, NULL, 0 },
	{ SENSOR_NUM_SB_HEX_AIR_INLET_4_TEMP_C, THRESHOLD_ENABLE_UCR, 0, 60, NULL, 0 },
	{ SENSOR_NUM_FB_1_HEX_OUTLET_TEMP_C, THRESHOLD_ENABLE_UCR, 0, 40, NULL, 0 },
	{ SENSOR_NUM_FB_2_HEX_OUTLET_TEMP_C, THRESHOLD_ENABLE_UCR, 0, 40, NULL, 0 },
	{ SENSOR_NUM_FB_3_HEX_OUTLET_TEMP_C, THRESHOLD_ENABLE_UCR, 0, 40, NULL, 0 },
	{ SENSOR_NUM_FB_4_HEX_OUTLET_TEMP_C, THRESHOLD_ENABLE_UCR, 0, 40, NULL, 0 },
	{ SENSOR_NUM_FB_5_HEX_OUTLET_TEMP_C, THRESHOLD_ENABLE_UCR, 0, 40, NULL, 0 },
	{ SENSOR_NUM_FB_6_HEX_OUTLET_TEMP_C, THRESHOLD_ENABLE_UCR, 0, 40, NULL, 0 },
	{ SENSOR_NUM_FB_7_HEX_OUTLET_TEMP_C, THRESHOLD_ENABLE_UCR, 0, 40, NULL, 0 },
	{ SENSOR_NUM_FB_8_HEX_OUTLET_TEMP_C, THRESHOLD_ENABLE_UCR, 0, 40, NULL, 0 },
	{ SENSOR_NUM_FB_9_HEX_OUTLET_TEMP_C, THRESHOLD_ENABLE_UCR, 0, 40, NULL, 0 },
	{ SENSOR_NUM_FB_10_HEX_OUTLET_TEMP_C, THRESHOLD_ENABLE_UCR, 0, 40, NULL, 0 },
	{ SENSOR_NUM_FB_11_HEX_OUTLET_TEMP_C, THRESHOLD_ENABLE_UCR, 0, 40, NULL, 0 },
	{ SENSOR_NUM_FB_12_HEX_OUTLET_TEMP_C, THRESHOLD_ENABLE_UCR, 0, 40, NULL, 0 },
	{ SENSOR_NUM_FB_13_HEX_OUTLET_TEMP_C, THRESHOLD_ENABLE_UCR, 0, 40, NULL, 0 },
	{ SENSOR_NUM_FB_14_HEX_OUTLET_TEMP_C, THRESHOLD_ENABLE_UCR, 0, 40, NULL, 0 },
	// pwm device
	{ SENSOR_NUM_FB_1_FAN_TACH_RPM, THRESHOLD_ENABLE_LCR, 500, 0, fan_board_tach_status_handler,
	  SENSOR_NUM_FB_1_FAN_TACH_RPM },
	{ SENSOR_NUM_FB_2_FAN_TACH_RPM, THRESHOLD_ENABLE_LCR, 500, 0, fan_board_tach_status_handler,
	  SENSOR_NUM_FB_2_FAN_TACH_RPM },
	{ SENSOR_NUM_FB_3_FAN_TACH_RPM, THRESHOLD_ENABLE_LCR, 500, 0, fan_board_tach_status_handler,
	  SENSOR_NUM_FB_3_FAN_TACH_RPM },
	{ SENSOR_NUM_FB_4_FAN_TACH_RPM, THRESHOLD_ENABLE_LCR, 500, 0, fan_board_tach_status_handler,
	  SENSOR_NUM_FB_4_FAN_TACH_RPM },
	{ SENSOR_NUM_FB_5_FAN_TACH_RPM, THRESHOLD_ENABLE_LCR, 500, 0, fan_board_tach_status_handler,
	  SENSOR_NUM_FB_5_FAN_TACH_RPM },
	{ SENSOR_NUM_FB_6_FAN_TACH_RPM, THRESHOLD_ENABLE_LCR, 500, 0, fan_board_tach_status_handler,
	  SENSOR_NUM_FB_6_FAN_TACH_RPM },
	{ SENSOR_NUM_FB_7_FAN_TACH_RPM, THRESHOLD_ENABLE_LCR, 500, 0, fan_board_tach_status_handler,
	  SENSOR_NUM_FB_7_FAN_TACH_RPM },
	{ SENSOR_NUM_FB_8_FAN_TACH_RPM, THRESHOLD_ENABLE_LCR, 500, 0, fan_board_tach_status_handler,
	  SENSOR_NUM_FB_8_FAN_TACH_RPM },
	{ SENSOR_NUM_FB_9_FAN_TACH_RPM, THRESHOLD_ENABLE_LCR, 500, 0, fan_board_tach_status_handler,
	  SENSOR_NUM_FB_9_FAN_TACH_RPM },
	{ SENSOR_NUM_FB_10_FAN_TACH_RPM, THRESHOLD_ENABLE_LCR, 500, 0,
	  fan_board_tach_status_handler, SENSOR_NUM_FB_10_FAN_TACH_RPM },
	{ SENSOR_NUM_FB_11_FAN_TACH_RPM, THRESHOLD_ENABLE_LCR, 500, 0,
	  fan_board_tach_status_handler, SENSOR_NUM_FB_11_FAN_TACH_RPM },
	{ SENSOR_NUM_FB_12_FAN_TACH_RPM, THRESHOLD_ENABLE_LCR, 500, 0,
	  fan_board_tach_status_handler, SENSOR_NUM_FB_12_FAN_TACH_RPM },
	{ SENSOR_NUM_FB_13_FAN_TACH_RPM, THRESHOLD_ENABLE_LCR, 500, 0,
	  fan_board_tach_status_handler, SENSOR_NUM_FB_13_FAN_TACH_RPM },
	{ SENSOR_NUM_FB_14_FAN_TACH_RPM, THRESHOLD_ENABLE_LCR, 500, 0,
	  fan_board_tach_status_handler, SENSOR_NUM_FB_14_FAN_TACH_RPM },
	{ SENSOR_NUM_PB_1_PUMP_TACH_RPM, THRESHOLD_ENABLE_LCR, 500, 0, pump_failure_do,
	  SENSOR_NUM_PB_1_PUMP_TACH_RPM },
	{ SENSOR_NUM_PB_2_PUMP_TACH_RPM, THRESHOLD_ENABLE_LCR, 500, 0, pump_failure_do,
	  SENSOR_NUM_PB_2_PUMP_TACH_RPM },
	{ SENSOR_NUM_PB_3_PUMP_TACH_RPM, THRESHOLD_ENABLE_LCR, 500, 0, pump_failure_do,
	  SENSOR_NUM_PB_3_PUMP_TACH_RPM },
	{ SENSOR_NUM_PB_1_FAN_1_TACH_RPM, THRESHOLD_ENABLE_LCR, 500, 0, rpu_internal_fan_failure_do,
	  SENSOR_NUM_PB_1_FAN_1_TACH_RPM },
	{ SENSOR_NUM_PB_1_FAN_2_TACH_RPM, THRESHOLD_ENABLE_LCR, 500, 0, rpu_internal_fan_failure_do,
	  SENSOR_NUM_PB_1_FAN_2_TACH_RPM },
	{ SENSOR_NUM_PB_2_FAN_1_TACH_RPM, THRESHOLD_ENABLE_LCR, 500, 0, rpu_internal_fan_failure_do,
	  SENSOR_NUM_PB_2_FAN_1_TACH_RPM },
	{ SENSOR_NUM_PB_2_FAN_2_TACH_RPM, THRESHOLD_ENABLE_LCR, 500, 0, rpu_internal_fan_failure_do,
	  SENSOR_NUM_PB_2_FAN_2_TACH_RPM },
	{ SENSOR_NUM_PB_3_FAN_1_TACH_RPM, THRESHOLD_ENABLE_LCR, 500, 0, rpu_internal_fan_failure_do,
	  SENSOR_NUM_PB_3_FAN_1_TACH_RPM },
	{ SENSOR_NUM_PB_3_FAN_2_TACH_RPM, THRESHOLD_ENABLE_LCR, 500, 0, rpu_internal_fan_failure_do,
	  SENSOR_NUM_PB_3_FAN_2_TACH_RPM },
	{ SENSOR_NUM_MB_FAN1_TACH_RPM, THRESHOLD_ENABLE_LCR, 500, 0, NULL, 0 },
	{ SENSOR_NUM_MB_FAN2_TACH_RPM, THRESHOLD_ENABLE_LCR, 500, 0, NULL, 0 },
	{ SENSOR_NUM_BPB_RPU_COOLANT_INLET_P_KPA, THRESHOLD_ENABLE_LCR, -20, 0, NULL, 0 },
	{ SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_P_KPA, THRESHOLD_ENABLE_UCR, 0, 200, abnormal_press_do,
	  0 },
	{ SENSOR_NUM_BPB_RPU_COOLANT_FLOW_RATE_LPM, THRESHOLD_ENABLE_LCR, 10, 0, flow_trigger_do,
	  0 },
	{ SENSOR_NUM_BPB_RACK_LEVEL_1, THRESHOLD_ENABLE_LCR, 0.1, 0, high_level_do, 0 },
	{ SENSOR_NUM_BPB_RACK_LEVEL_2, THRESHOLD_ENABLE_LCR, 0.1, 0, low_level_do, 0 },
};

void set_threshold_poll_enable_flag(bool flag)
{
	threshold_poll_enable_flag = flag;
}

bool get_threshold_poll_enable_flag()
{
	return threshold_poll_enable_flag;
}

uint8_t get_threshold_status(uint8_t sensor_num)
{
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
	uint8_t status = THRESHOLD_STATUS_NORMAL;

	/* check device is exist */
	// to determine if tach value is 0, set status to DEVICE_NOT_EXIST status
	for (int i = 0; i < ARRAY_SIZE(fan_pump_sensor_array); i++) {
		if (threshold_tbl->sensor_num == fan_pump_sensor_array[i]) {
			if (val == 0.0) {
				threshold_tbl->last_status = THRESHOLD_STATUS_NOT_ACCESS;
				LOG_DBG("sensor 0x%x not exist", threshold_tbl->sensor_num);
				return true;
			}
		}
	}

	switch (threshold_tbl->type) {
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

	if (threshold_tbl->last_status == status)
		return false;

	threshold_tbl->last_status = status;
	return true;
}

void threshold_poll_handler(void *arug0, void *arug1, void *arug2)
{
	ARG_UNUSED(arug0);
	ARG_UNUSED(arug1);
	ARG_UNUSED(arug2);

	int threshold_poll_interval_ms = 1000; // interval 1s

	k_msleep(20000); // wait 20s for sensor ready
	ctl_all_pwm_dev(60);

	while (1) {
		if (!get_sensor_init_done_flag()) {
			k_msleep(threshold_poll_interval_ms);
			continue;
		}

		if (!get_sensor_poll_enable_flag()) {
			k_msleep(threshold_poll_interval_ms);
			continue;
		}

		if (!get_threshold_poll_enable_flag()) {
			k_msleep(threshold_poll_interval_ms);
			continue;
		}

		for (uint8_t i = 0; i < ARRAY_SIZE(threshold_tbl); i++) {
			float val = 0;
			sensor_cfg *cfg = get_common_sensor_cfg_info(threshold_tbl[i].sensor_num);
			if (cfg->cache_status != SENSOR_READ_4BYTE_ACUR_SUCCESS)
				continue;

			if (get_sensor_reading_to_real_val(threshold_tbl[i].sensor_num, &val) !=
			    SENSOR_READ_4BYTE_ACUR_SUCCESS) {
				threshold_tbl[i].last_status = THRESHOLD_STATUS_NOT_ACCESS;
				continue;
			}

			/* check whether the status has changed */
			if (!set_threshold_status(&threshold_tbl[i], val))
				continue;

			if (threshold_tbl[i].fn)
				threshold_tbl[i].fn(threshold_tbl[i].arg0,
						    threshold_tbl[i].last_status);
		}

		// control fault led
		fault_led_control();

		// pump recovery
		if (pump_status_recovery()) {
			set_pwm_group(PWM_GROUP_E_PUMP, 60);
			if (rpu_ready_recovery())
				set_all_rpu_ready_pin_normal();
		}

		k_msleep(threshold_poll_interval_ms);
	}

	return;
}

void threshold_poll_init()
{
	k_thread_create(&threshold_poll, threshold_poll_stack,
			K_THREAD_STACK_SIZEOF(threshold_poll_stack), threshold_poll_handler, NULL,
			NULL, NULL, CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&threshold_poll, "threshold_poll");
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
			// read gok data
			sensor_cfg *cfg = get_common_sensor_cfg_info(fan_pump_sensor_array[i]);
			uint8_t read_gok =
				nct7363_read_back_data(cfg, NCT7363_GPIO0x_INPUT_PORT_REG_OFFSET);
			if (read_gok == READ_ERROR)
				LOG_ERR("Read pump_board_pwrgd gpio fail, read_back_val: %d",
					read_gok);

			uint8_t pwrgd_read_back_val =
				nct7363_read_back_data(cfg, NCT7363_GPIO1x_OUTPUT_PORT_REG_OFFSET);
			if (pwrgd_read_back_val == READ_ERROR)
				LOG_ERR("Thread read val: %d", pwrgd_read_back_val);

			if (!(read_gok & BIT(2))) {
				WRITE_BIT(pwrgd_read_back_val, 0, 0);
				if (!nct7363_write(cfg, FAN_BOARD_FAULT_PWM_OFFSET,
						   FAN_BOARD_FAULT_PWM_DUTY_100))
					LOG_ERR("Write fan_board_fault pwm fail");
			}

			if (!nct7363_write(cfg, NCT7363_GPIO1x_OUTPUT_PORT_REG_OFFSET,
					   pwrgd_read_back_val))
				LOG_ERR("Write pump_board_pwrgd gpio fail");
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
