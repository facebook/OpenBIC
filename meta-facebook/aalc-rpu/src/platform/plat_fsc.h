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

#ifndef PLAT_FSC_H
#define PLAT_FSC_H
#include "stdio.h"
#include "stdint.h"
#include "stdbool.h"

#define FSC_TEMP_INVALID 0x8000
#define FSC_RPM_INVALID 0xFFFF

#define FSC_ENABLE 1
#define FSC_DISABLE 0

#define SENSOR_STEPWISE_STEPS_MAX 16

enum FSC_ERROR {
	FSC_ERROR_NONE = 0, // OK status
	FSC_ERROR_UNKNOW,
	FSC_ERROR_OUT_OF_RANGE,
	FSC_ERROR_NOT_FOUND_ZONE_TABLE,
	FSC_ERROR_NOT_FOUND_STEPWISE_TABLE,
	FSC_ERROR_NOT_FOUND_PID_TABLE,
	FSC_ERROR_NULL_ARG,
};

enum FSC_TYPE {
	FSC_TYPE_DISABLE = 0,
	FSC_TYPE_STEPWISE,
	FSC_TYPE_PID,
	FSC_TYPE_BOTH,
	FSC_TYPE_DEFAULT,
};

enum FSC_MODE_E {
	FSC_MODE_UNKNOW = 0,
	FSC_MODE_AUTO_MODE, // include auto tune
	FSC_MODE_MANUAL_MODE,
	FSC_MODE_SEMI_MODE,
};

/* stepwise */
typedef struct {
	float temp;
	uint8_t duty;
} stepwise_dict;

typedef struct {
	uint8_t sensor_num;
	stepwise_dict step[SENSOR_STEPWISE_STEPS_MAX];
	uint8_t pos_hyst; // positive_hysteresis
	uint8_t neg_hyst; // negative_hysteresis

	// calculate use
	float last_temp;
} stepwise_cfg;

/* pid */
typedef struct {
	uint8_t sensor_num;
	float setpoint;
	/*uint8_t setpoint_type;*/
	float kp;
	float ki;
	float kd;
	int16_t i_limit_min; // RPM
	int16_t i_limit_max; // RPM
	uint8_t pos_hyst; // positive_hysteresis
	uint8_t neg_hyst; // negative_hysteresis

	// calculate use
	float integral;
	float last_error; //for kd
	float last_temp;
} pid_cfg;

/* zone control */
typedef struct {
	uint8_t sensor_num;
	uint8_t type; // stepwise or pid
} fsc_type_mapping;

typedef struct {
	stepwise_cfg *sw_tbl;
	uint8_t sw_tbl_num;

	pid_cfg *pid_tbl;
	uint8_t pid_tbl_num;

	uint8_t out_limit_min; // Duty
	uint8_t out_limit_max; // Duty
	uint16_t slew_neg; // RPM
	uint16_t slew_pos; // RPM
	uint8_t interval; // sec

	uint8_t (*set_duty)(uint8_t, uint8_t); // set duty function
	uint8_t set_duty_arg; //  set_duty arg

	// calculate use
	uint16_t fsc_poll_count;
	uint8_t last_duty;
	uint8_t is_init;
} zone_cfg;

uint8_t get_fsc_enable_flag(void);
void set_fsc_enable_flag(uint8_t flag);
void set_fsc_tbl_enable(uint8_t flag);
void fsc_init(void);
void controlFSC(uint8_t action);
uint8_t fsc_debug_set(uint8_t enable);
uint8_t fsc_debug_get(void);
float get_fsc_setpoint(uint8_t idx);
void set_fsc_setpoint(uint8_t idx, float val);
void change_lpm_setpoint(uint8_t onoff);
void change_temp_setpoint(uint8_t onoff);

#endif