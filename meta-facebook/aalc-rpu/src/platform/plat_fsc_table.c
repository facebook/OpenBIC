#include "plat_fsc.h"
#include "plat_sensor_table.h"
#include "libutil.h"
#include "plat_pwm.h"
#include "plat_hwmon.h"

pid_cfg hex_fan_pid_table[] = {
	{
		.sensor_num = SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_TEMP_C,
		.setpoint = 40,
		.kp = -7,
		.ki = -0.1,
		.kd = -3,
		.i_limit_min = 0,
		.i_limit_max = 0,
		.pos_hyst = 1,
		.neg_hyst = 1,
	},
};

stepwise_cfg hex_fan_stepwise_table[] = {
	{
		.sensor_num = SENSOR_NUM_SB_HEX_AIR_INLET_AVG_TEMP_C,
		.step = {
			{25, 20},
			{26, 20},
			{27, 20},
			{28, 20},
			{29, 20},
			{30, 20},
			{31, 21},
			{32, 22},
			{33, 23},
			{34, 24},
			{35, 25},
			{36, 26},
			{37, 27},
			{38, 28},
			{39, 29},
			{40, 30},
		},
	},
	{
		.sensor_num = SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_TEMP_C,
		.step = {
			{25, 15},
			{26, 16},
			{27, 17},
			{28, 18},
			{29, 19},
			{30, 20},
			{31, 21},
			{32, 22},
			{33, 23},
			{34, 24},
			{35, 25},
			{36, 26},
			{37, 27},
			{38, 28},
			{39, 29},
			{40, 30},
		},
	},
};

/*
pid_cfg pump_pid_table[] = {
	{
		.sensor_num = SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_TEMP_C,
		.setpoint_type = SETPOINT_FLOW_RATE_LPM,
		.kp = -7,
		.ki = -0.1,
		.kd = -3,
		.i_limit_min = 0,
		.i_limit_max = 0,
		.pos_hyst = 1,
		.neg_hyst = 1,
	},
};

stepwise_cfg pump_stepwise_table[] = {
	{
		.sensor_num = SENSOR_NUM_SB_HEX_AIR_INLET_AVG_TEMP_C,
		.step = {
			{25, 30},
			{26, 32},
			{27, 34},
			{28, 36},
			{29, 38},
			{30, 40},
			{31, 42},
			{32, 44},
			{33, 46},
			{34, 48},
			{35, 50},
			{36, 52},
			{37, 54},
			{38, 56},
			{39, 58},
			{40, 60},
		},
	},
	{
		.sensor_num = SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_TEMP_C,
		.step = {
			{30, 20},
			{31, 21},
			{32, 22},
			{33, 23},
			{34, 24},
			{35, 25},
			{36, 26},
			{37, 27},
			{38, 28},
			{39, 29},
			{40, 30},
			{41, 31},
			{42, 32},
			{43, 33},
			{44, 34},
			{45, 35},
		},
	},
};*/

stepwise_cfg pump_stepwise_table[] = {
	{
		.sensor_num = SENSOR_NUM_COOLING_CAPACITY,
		.step = {
			{30, 45},
			{45, 55},
			{60, 65},
			{61, 100},
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
		.out_limit_min = 20,
		.out_limit_max = 100,
	},
	{
		// zone 2 - pump
		.sw_tbl = pump_stepwise_table,
		.sw_tbl_num = ARRAY_SIZE(pump_stepwise_table),
		/*
		.pid_tbl = pump_pid_table,
		.pid_tbl_num = ARRAY_SIZE(pump_pid_table),*/
		.interval = 1,
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