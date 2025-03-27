#include "plat_fsc.h"
#include "plat_sensor_table.h"
#include "libutil.h"
#include "plat_pwm.h"
#include "plat_hwmon.h"

pid_cfg hex_fan_pid_table[] = {
	{
		.sensor_num = SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_TEMP_C,
		.setpoint = 40,
		.kp = -3,
		.ki = -0.05,
		.kd = -0.01,
		.i_limit_min = -5,
		.i_limit_max = 90,
		.pos_hyst = 1,
		.neg_hyst = 1,
	},
};

stepwise_cfg hex_fan_stepwise_table[] = {
	{
		.sensor_num = SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_TEMP_C,
		.step = {
			{25, 10},
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
		.neg_hyst = 0,
	},
};

stepwise_cfg pump_stepwise_auto_mode_table[] = {
	{
		.sensor_num = SENSOR_NUM_COOLING_CAPACITY,
		.step = {
			{1, 100},
		},
	},
};

stepwise_cfg pump_stepwise_auto_tune_table[] = {
	{
		.sensor_num = SENSOR_NUM_COOLING_CAPACITY,
		.step = {
			{1, 10},
		},
	},
};

stepwise_cfg rpu_fan_stepwise_table[] = {
	{
		.sensor_num = SENSOR_NUM_SB_HEX_AIR_INLET_AVG_TEMP_C,
		.step = {
			{25, 25},
			{26, 26},
			{27, 27},
			{28, 28},
			{29, 29},
			{30, 30},
			{31, 31},
			{32, 32},
			{33, 33},
			{34, 34},
			{35, 35},
			{36, 36},
			{37, 37},
			{38, 38},
			{39, 39},
			{40, 40},
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