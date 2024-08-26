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

#define THRESHOLD_ARG0_TABLE_INDEX 0xFFFFFFFF

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

static void fb_prsnt_handle(uint32_t thres_tbl_idx, uint32_t changed_status);

/* if leak or some threshold fault, return false */
bool pump_status_recovery()
{
	static uint8_t pre_status = 0; // 0: normal, 1: fault
	if (get_leak_status()) {
		pre_status = 1;
		return false;
	}

	uint8_t check_fault[] = {
		SENSOR_NUM_PB_1_FAN_1_TACH_RPM,		  SENSOR_NUM_PB_1_FAN_2_TACH_RPM,
		SENSOR_NUM_PB_2_FAN_1_TACH_RPM,		  SENSOR_NUM_PB_2_FAN_2_TACH_RPM,
		SENSOR_NUM_PB_3_FAN_1_TACH_RPM,		  SENSOR_NUM_PB_3_FAN_2_TACH_RPM,
		SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_P_KPA,  SENSOR_NUM_BPB_RACK_LEVEL_2,
		SENSOR_NUM_BPB_RPU_COOLANT_FLOW_RATE_LPM,
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
	if (get_leak_status())
		return false;

	uint8_t check_fault[] = {
		SENSOR_NUM_PB_1_FAN_1_TACH_RPM,
		SENSOR_NUM_PB_1_FAN_2_TACH_RPM,
		SENSOR_NUM_PB_2_FAN_1_TACH_RPM,
		SENSOR_NUM_PB_2_FAN_2_TACH_RPM,
		SENSOR_NUM_PB_3_FAN_1_TACH_RPM,
		SENSOR_NUM_PB_3_FAN_2_TACH_RPM,
		SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_P_KPA,
		SENSOR_NUM_BPB_RACK_LEVEL_2,
		// except close pump sensor
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

void fan_board_tach_status_handler(uint32_t sensor_num, uint32_t status)
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

static bool close_pump_hsc(uint8_t sensor_num)
{
	bool ret = true;
	sensor_cfg *cfg = get_common_sensor_cfg_info(sensor_num);
	if ((cfg->pre_sensor_read_hook)) {
		if ((cfg->pre_sensor_read_hook)(cfg, cfg->pre_sensor_read_args) == false) {
			LOG_DBG("read value pre lock mutex fail !");
			return false;
		}
	}

	if (!enable_adm1272_hsc(cfg->port, cfg->target_addr, false))
		ret = false;

	if ((cfg->post_sensor_read_hook)) {
		if ((cfg->post_sensor_read_hook)(cfg, cfg->post_sensor_read_args, 0) == false) {
			LOG_DBG("read value post lock mutex fail !");
			return false;
		}
	}

	return ret;
}

static bool pump_fail_ctrl(uint8_t sensor_num)
{
	bool ret = true;
	uint8_t pwm_dev =
		(sensor_num == SENSOR_NUM_PB_1_PUMP_TACH_RPM) ? PWM_DEVICE_E_PB_PUMB_FAN_1 :
		(sensor_num == SENSOR_NUM_PB_2_PUMP_TACH_RPM) ? PWM_DEVICE_E_PB_PUMB_FAN_2 :
		(sensor_num == SENSOR_NUM_PB_3_PUMP_TACH_RPM) ? PWM_DEVICE_E_PB_PUMB_FAN_3 :
								PWM_DEVICE_E_MAX;
	uint8_t hsc_sensor =
		(sensor_num == SENSOR_NUM_PB_1_PUMP_TACH_RPM) ? SENSOR_NUM_PB_1_HSC_P48V_TEMP_C :
		(sensor_num == SENSOR_NUM_PB_2_PUMP_TACH_RPM) ? SENSOR_NUM_PB_2_HSC_P48V_TEMP_C :
		(sensor_num == SENSOR_NUM_PB_3_PUMP_TACH_RPM) ? SENSOR_NUM_PB_3_HSC_P48V_TEMP_C :
								0xFF;

	if (pwm_dev == PWM_DEVICE_E_MAX)
		return false;

	if (hsc_sensor == 0xFF)
		return false;

	if (get_threshold_status(sensor_num)) {
		if (plat_pwm_ctrl(pwm_dev, 0))
			ret = false;
		if (!close_pump_hsc(hsc_sensor))
			ret = false;
	}

	return ret;
}

void pump_failure_do(uint32_t sensor_num, uint32_t status)
{
	pump_board_tach_status_handler(sensor_num, status);
	if (status == THRESHOLD_STATUS_LCR) {
		//auto control for Hex Fan
		error_log_event(sensor_num, IS_ABNORMAL_VAL);
		//wait 30 secs to shut down
	} else if (status == THRESHOLD_STATUS_UCR) {
		if (!pump_fail_ctrl(sensor_num))
			LOG_ERR("threshold 0x%02x pump failure", sensor_num);
	} else if (status == THRESHOLD_STATUS_NORMAL) {
		error_log_event(sensor_num, IS_NORMAL_VAL);
	} else
		LOG_DBG("Unexpected threshold warning");
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
	if (status == THRESHOLD_STATUS_UCR) {
		ctl_all_pwm_dev(0);
	} else {
		LOG_DBG("Unexpected threshold warning");
	}
}

void abnormal_flow_do(uint32_t unused, uint32_t status)
{
	if (status == THRESHOLD_STATUS_LCR) {
		ctl_all_pwm_dev(0);
	} else {
		LOG_DBG("Unexpected threshold warning");
	}
}

void level_sensor_do(uint32_t unused, uint32_t status)
{
	if (get_threshold_status(SENSOR_NUM_BPB_RACK_LEVEL_2)) {
		ctl_all_pwm_dev(0);
		error_log_event(SENSOR_NUM_BPB_RACK_LEVEL_2, IS_ABNORMAL_VAL);
		if (get_threshold_status(SENSOR_NUM_BPB_RACK_LEVEL_1))
			led_ctrl(LED_IDX_E_COOLANT, LED_TURN_ON);
		else
			LOG_DBG("BPB_RACK_LEVEL_1 1fail\n");
	} else {
		if (get_threshold_status(SENSOR_NUM_BPB_RACK_LEVEL_1)) {
			led_ctrl(LED_IDX_E_COOLANT, LED_START_BLINK);
		} else {
			led_ctrl(LED_IDX_E_COOLANT, LED_TURN_OFF);
		}
	}
}

sensor_threshold threshold_tbl[] = {
	{ SENSOR_NUM_BPB_RPU_COOLANT_INLET_TEMP_C, THRESHOLD_ENABLE_UCR, 0, 65, NULL, 0 },
	{ SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_TEMP_C, THRESHOLD_ENABLE_UCR, 0, 65, NULL, 0 },
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
	{ SENSOR_NUM_PB_1_PUMP_TACH_RPM, THRESHOLD_ENABLE_BOTH, 500, 8000, pump_failure_do,
	  SENSOR_NUM_PB_1_PUMP_TACH_RPM },
	{ SENSOR_NUM_PB_2_PUMP_TACH_RPM, THRESHOLD_ENABLE_BOTH, 500, 8000, pump_failure_do,
	  SENSOR_NUM_PB_2_PUMP_TACH_RPM },
	{ SENSOR_NUM_PB_3_PUMP_TACH_RPM, THRESHOLD_ENABLE_BOTH, 500, 8000, pump_failure_do,
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
	{ SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_P_KPA, THRESHOLD_ENABLE_UCR, 0, 300, abnormal_press_do,
	  0 },
	{ SENSOR_NUM_BPB_RPU_COOLANT_FLOW_RATE_LPM, THRESHOLD_ENABLE_LCR, 10, 0, abnormal_flow_do,
	  0 },
	{ SENSOR_NUM_BPB_RACK_LEVEL_1, THRESHOLD_ENABLE_LCR, 0.1, 0, level_sensor_do, 0 },
	{ SENSOR_NUM_BPB_RACK_LEVEL_2, THRESHOLD_ENABLE_LCR, 0.1, 0, level_sensor_do, 0 },
	{ SENSOR_NUM_FAN_PRSNT, THRESHOLD_ENABLE_DISCRETE, 0, 0, fb_prsnt_handle,
	  THRESHOLD_ARG0_TABLE_INDEX, 0 },
};

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

			if (threshold_tbl[i].type == THRESHOLD_ENABLE_DISCRETE) {
				/* check the discrete sensor has value changed */
				/* do not need to convert the sensor reading value */

				int reading = 0;
				uint8_t status =
					get_sensor_reading(sensor_config, sensor_config_count,
							   threshold_tbl[i].sensor_num, &reading,
							   GET_FROM_CACHE);

				if (status != SENSOR_READ_4BYTE_ACUR_SUCCESS) {
					LOG_ERR("0x%02x get sensor cache fail",
						threshold_tbl[i].sensor_num);
					continue;
				}

				LOG_ERR("threshold_tbl[i].last_value = %x, reading = %x",
					threshold_tbl[i].last_value, reading);
				if (threshold_tbl[i].last_value == reading)
					continue;

				/* use the last_status to carry change bits */
				threshold_tbl[i].last_status =
					threshold_tbl[i].last_value ^ reading;
				threshold_tbl[i].last_value = reading;

			} else {
				if (get_sensor_reading_to_real_val(threshold_tbl[i].sensor_num,
								   &val) !=
				    SENSOR_READ_4BYTE_ACUR_SUCCESS)
					threshold_tbl[i].last_status = THRESHOLD_STATUS_NOT_ACCESS;
				else if (!set_threshold_status(&threshold_tbl[i], val))
					continue;

				/* check whether the status has changed */
				if (!set_threshold_status(&threshold_tbl[i], val))
					continue;
			}

			if (threshold_tbl[i].fn)
				threshold_tbl[i].fn(threshold_tbl[i].arg0,
						    threshold_tbl[i].last_status);
		}

		// control fault led
		fault_led_control();

		// pump recovery
		if (pump_status_recovery())
			set_pwm_group(PWM_GROUP_E_PUMP, 60);

		// rpu ready pin setting
		if (rpu_ready_recovery())
			set_all_rpu_ready_pin_normal();
		else
			deassert_all_rpu_ready_pin();

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
