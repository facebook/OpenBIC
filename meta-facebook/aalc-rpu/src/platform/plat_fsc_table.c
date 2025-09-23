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

#include "plat_fsc.h"
#include "plat_sensor_table.h"
#include "libutil.h"
#include "plat_pwm.h"
#include "plat_hwmon.h"

pid_cfg hex_fan_pid_table[] = {
	{
		.sensor_num = SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_TEMP_C,
		.setpoint = 40,
		.kp = -15,
		.ki = -0.1,
		.kd = -0.01,
		.i_limit_min = 0,
		.i_limit_max = 90,
		.pos_hyst = 0,
		.neg_hyst = 0,
	},
};

stepwise_cfg hex_fan_stepwise_table[] = {
		{
		.sensor_num = SENSOR_NUM_SB_HEX_AIR_INLET_AVG_TEMP_C,
		.step = {
			{30.0, 5},
			{31.0, 7},
			{32.0, 9},
			{33.0, 11},
			{34.0, 13},
			{35.0, 15},
			{36.0, 17},
			{40.0, 19},
		},
	},
	{
		.sensor_num = SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_TEMP_C,
		.step = {
			{34.0, 5},
			{35.0, 8},
			{36.0, 11},
			{37.0, 14},
			{38.0, 17},
			{41.0, 20},
			{41.2, 25},
			{41.5, 35},
			{42.0, 45},
		},
	},
};

pid_cfg pump_pid_table[] = {
	{
		.sensor_num = SENSOR_NUM_BPB_RPU_COOLANT_FLOW_RATE_LPM,
		.setpoint = 40,
		.kp = 0.5,
		.ki = 0.03,
		.kd = 0.01,
		.i_limit_min = -5,
		.i_limit_max = 90,
		.pos_hyst = 1,
		.neg_hyst = 1,
	},
};

stepwise_cfg pump_stepwise_auto_mode_table[] = {
	{
		.sensor_num = SENSOR_NUM_BPB_RPU_COOLANT_FLOW_RATE_LPM,
		.step = {
			{1, 100},
		},
	},
};

stepwise_cfg pump_stepwise_auto_tune_table[] = {
	{
		.sensor_num = SENSOR_NUM_BPB_RPU_COOLANT_FLOW_RATE_LPM,
		.step = {
			{1, 10},
		},
	},
};

stepwise_cfg rpu_fan_stepwise_table[] = {
	{
		.sensor_num = SENSOR_NUM_SB_HEX_AIR_INLET_AVG_TEMP_C,
		.step = {
			{25.0, 25},
			{26.0, 26},
			{27.0, 27},
			{28.0, 28},
			{29.0, 29},
			{30.0, 30},
			{31.0, 31},
			{32.0, 32},
			{33.0, 33},
			{34.0, 34},
			{35.0, 35},
			{36.0, 36},
			{37.0, 37},
			{38.0, 38},
			{39.0, 39},
			{40.0, 40},
		},
	},
};

zone_cfg zone_table[] = {
	{
		// zone 1 - hex fan
		.sw_tbl = hex_fan_stepwise_table,
		.sw_tbl_num = ARRAY_SIZE(hex_fan_stepwise_table),
		.pid_tbl = hex_fan_pid_table,
		.pid_tbl_num = ARRAY_SIZE(hex_fan_pid_table),
		.interval = 1,
		.set_duty = pwm_control,
		.set_duty_arg = PWM_GROUP_E_HEX_FAN,
		.out_limit_min = 10,
		.out_limit_max = 100,
	},
	{
		// zone 2 - pump
		.sw_tbl = pump_stepwise_auto_mode_table,
		.sw_tbl_num = ARRAY_SIZE(pump_stepwise_auto_mode_table),
		.pid_tbl = NULL,
		.pid_tbl_num = ARRAY_SIZE(pump_pid_table),
		.interval = 5,
		.set_duty = pwm_control,
		.set_duty_arg = PWM_GROUP_E_PUMP,
		.out_limit_min = 20,
		.out_limit_max = 100,
	},
	{
		// zone 3 - rpu fan
		.sw_tbl = rpu_fan_stepwise_table,
		.sw_tbl_num = ARRAY_SIZE(rpu_fan_stepwise_table),
		.interval = 1,
		.set_duty = pwm_control,
		.set_duty_arg = PWM_GROUP_E_RPU_FAN,
		.out_limit_min = 20,
		.out_limit_max = 100,
	},
};

uint32_t zone_table_size = ARRAY_SIZE(zone_table);
