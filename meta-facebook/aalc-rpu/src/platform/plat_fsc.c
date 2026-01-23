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
#include <stdlib.h>
#include <libutil.h>
#include "sensor.h"
#include "plat_sensor_table.h"
#include "plat_status.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(plat_fsc);

static bool enable_debug_print = false;
#define FSC_PRINTF(fmt, ...)                                                                       \
	do {                                                                                       \
		if (enable_debug_print) {                                                          \
			printf(fmt, ##__VA_ARGS__);                                                \
		}                                                                                  \
	} while (0)

struct k_thread fsc_thread;
K_KERNEL_STACK_MEMBER(fsc_thread_stack, 2048);

static uint8_t fsc_poll_flag = 1;
static bool fsc_tbl_enable = true;
extern zone_cfg zone_table[];
extern uint32_t zone_table_size;
extern pid_cfg pump_pid_table[];
extern stepwise_cfg pump_stepwise_auto_mode_table[];
extern stepwise_cfg pump_stepwise_auto_tune_table[];

static float setpoint[SETPOINT_FLAG_MAX] = { 0, 40.0 };

uint8_t fsc_debug_set(uint8_t enable)
{
	enable_debug_print = enable;
	return FSC_ERROR_NONE;
}

uint8_t fsc_debug_get(void)
{
	return enable_debug_print;
}

uint8_t get_fsc_enable_flag(void)
{
	return fsc_poll_flag;
}

void set_fsc_enable_flag(uint8_t flag)
{
	fsc_poll_flag = flag;
}

uint8_t get_fsc_tbl_enable(void)
{
	return fsc_tbl_enable;
}
void set_fsc_tbl_enable(uint8_t flag)
{
	fsc_tbl_enable = flag;
}

float get_fsc_setpoint(uint8_t idx)
{
	if (idx >= SETPOINT_FLAG_MAX) {
		return 0xFF;
	}
	return setpoint[idx];
}
void set_fsc_setpoint(uint8_t idx, float val)
{
	if (idx < SETPOINT_FLAG_MAX) {
		setpoint[idx] = val;
	}
}

/* get the maximum duty of all stepwise sensor */
static uint8_t calculateStepwise(zone_cfg *zone_p, uint8_t *duty)
{
	CHECK_NULL_ARG_WITH_RETURN(zone_p, FSC_ERROR_NULL_ARG);
	CHECK_NULL_ARG_WITH_RETURN(duty, FSC_ERROR_NULL_ARG);

	if (!zone_p->sw_tbl || !zone_p->sw_tbl_num)
		return FSC_ERROR_NULL_ARG;

	FSC_PRINTF("\t------- calculateStepwise\n");
	uint8_t max_duty = 0;
	uint8_t ambient_duty = 0;

	for (int i = 0; i < zone_p->sw_tbl_num; i++) {
		stepwise_cfg *p = zone_p->sw_tbl + i;
		if (!p)
			continue;

		float tmp = 0.0;
		if (get_sensor_reading_to_real_val(p->sensor_num, &tmp) !=
		    SENSOR_READ_4BYTE_ACUR_SUCCESS) {
			// if the sensor reading fail, assume that is 100.0 degree
			tmp = 100.0;
		}

		float temp = tmp;
		FSC_PRINTF("\t\t----- sensor_num %x, temp = %f\n", p->sensor_num, temp);

		// hysteresis
		if (p->pos_hyst || p->neg_hyst) {
			if (p->last_temp == FSC_TEMP_INVALID) // first time
				p->last_temp = temp;
			else if ((temp - p->last_temp) > p->pos_hyst)
				p->last_temp = temp;
			else if ((p->last_temp - temp) > p->neg_hyst)
				p->last_temp = temp;
		} else {
			p->last_temp = temp;
		}

		// find duty by temp
		uint8_t tmp_duty = 100;
		for (int j = 0; j < ARRAY_SIZE(p->step); j++) {
			// 0 is a step that does not exist
			if (p->step[j].temp == 0) {
				if (j > 0) {
					tmp_duty = p->step[j - 1].duty;
				}
				break;
			}

			FSC_PRINTF("\t\t\ttemp %f, duty %d\n", p->step[j].temp, p->step[j].duty);
			if (p->last_temp <= p->step[j].temp) {
				tmp_duty = p->step[j].duty;
				break;
			}
		}

		// the ambient sensor should be a duty base, do not compare with the maximum duty
		if (p->sensor_num == SENSOR_NUM_SB_HEX_AIR_INLET_AVG_TEMP_C)
			ambient_duty = tmp_duty;
		else
			max_duty = MAX(max_duty, tmp_duty);

		FSC_PRINTF(
			"\t\tsensor_num %x, last_temp = %f, tmp_duty = %d, max_duty = %d, amb_duty = %d\n",
			p->sensor_num, p->last_temp, tmp_duty, max_duty, ambient_duty);
	}

	*duty = ambient_duty + max_duty;
	FSC_PRINTF("\tcalculateStepwise duty = %d\n", *duty);
	return FSC_ERROR_NONE;
}

static uint8_t calculatePID(zone_cfg *zone_p, uint8_t *duty)
{
	CHECK_NULL_ARG_WITH_RETURN(zone_p, FSC_ERROR_NULL_ARG);
	CHECK_NULL_ARG_WITH_RETURN(duty, FSC_ERROR_NULL_ARG);

	FSC_PRINTF("\t------- calculatePID\n");

	uint8_t max_duty = 0;
	for (int i = 0; i < zone_p->pid_tbl_num; i++) {
		pid_cfg *p = zone_p->pid_tbl + i;
		if (!p)
			continue;

		float tmp = 0.0;
		if (get_sensor_reading_to_real_val(p->sensor_num, &tmp) !=
		    SENSOR_READ_4BYTE_ACUR_SUCCESS) {
			tmp = 100.0;
		}

		// pump group will truncate decimals
		float temp =
			(zone_p->pid_tbl->sensor_num == SENSOR_NUM_BPB_RPU_COOLANT_FLOW_RATE_LPM) ?
				(int16_t)tmp :
				tmp;

		FSC_PRINTF("\t\t----- sensor_num %x, temp = %f, p->setpoint %f\n", p->sensor_num,
			   temp, p->setpoint);

		// hysteresis
		if (p->pos_hyst || p->neg_hyst) {
			if (p->last_temp == FSC_TEMP_INVALID) // first time
				p->last_temp = temp;
			else if ((temp - p->last_temp) > p->pos_hyst)
				p->last_temp = temp;
			else if ((p->last_temp - temp) > p->neg_hyst)
				p->last_temp = temp;
		} else {
			p->last_temp = temp;
		}

		// p term
		float error = p->setpoint - temp;
		float pterm = p->kp * error;
		FSC_PRINTF("\t\t\tp->kp = %f, error = %f, pterm = %f\n", p->kp, error, pterm);

		// i term
		float iterm = p->integral;
		iterm += p->ki * error;

		FSC_PRINTF("\t\t\tp->ki = %f, p->integral = %f, iterm = %f\n", p->ki, p->integral,
			   iterm);
		iterm = CLAMP(iterm, p->i_limit_min, p->i_limit_max);
		FSC_PRINTF("\t\t\tclamped iterm = %f\n", iterm);

		// d term
		float dterm = p->kd * (error - p->last_error);
		FSC_PRINTF("\t\t\tp->kd = %f, p->last_error = %f, dterm = %f\n", p->kd,
			   p->last_error, dterm);

		// calculate duty
		uint8_t tmp_duty = (uint8_t)(pterm + iterm + dterm);
		FSC_PRINTF("\t\ttmp_duty = %d, %f\n", tmp_duty, pterm + iterm + dterm);

		p->integral = iterm;
		p->last_error = error;

		/* compare with the maximum duty */
		max_duty = MAX(max_duty, tmp_duty);
	}

	FSC_PRINTF("\tcalculatePID duty = %d\n", max_duty);
	*duty = max_duty;
	return FSC_ERROR_NONE;
}

uint8_t get_fsc_duty_cache(uint8_t zone, uint8_t *cache)
{
	CHECK_NULL_ARG_WITH_RETURN(cache, FSC_ERROR_NULL_ARG);
	if (zone >= zone_table_size)
		return FSC_ERROR_OUT_OF_RANGE;

	*cache = zone_table[zone].last_duty;
	return FSC_ERROR_NONE;
}

bool get_fsc_poll_count(uint8_t zone, uint8_t *count)
{
	CHECK_NULL_ARG_WITH_RETURN(count, false);
	if (zone >= zone_table_size)
		return false;

	*count = zone_table[zone].fsc_poll_count;
	return true;
}

/* set the zone_cfg stored data to default */
static void zone_init(void)
{
	for (uint8_t i = 0; i < zone_table_size; i++) {
		zone_cfg *zone_p = zone_table + i;

		zone_p->is_init = false;

		// init stepwise last temp
		for (uint8_t j = 0; j < zone_p->sw_tbl_num; j++) {
			stepwise_cfg *p = zone_p->sw_tbl + j;
			if (p)
				p->last_temp = FSC_TEMP_INVALID;
		}

		// init pid
		for (uint8_t k = 0; k < zone_p->pid_tbl_num; k++) {
			pid_cfg *p = zone_p->pid_tbl + k;
			if (p) {
				p->integral = 0.0;
				p->last_error = 0.0;
				p->last_temp = FSC_TEMP_INVALID;
			}
		}

		// init zone config
		zone_p->fsc_poll_count = 0;
		zone_p->last_duty = 0;
	}
}

void change_lpm_setpoint(uint8_t onoff)
{
	if (onoff) {
		zone_table[1].pid_tbl = pump_pid_table;
		zone_table[1].pid_tbl->setpoint = get_fsc_setpoint(SETPOINT_FLAG_LPM) + 1.0;
		zone_table[1].sw_tbl = pump_stepwise_auto_tune_table;
	} else {
		if (zone_table[1].pid_tbl) {
			zone_table[1].pid_tbl->integral = 0.0;
			zone_table[1].pid_tbl->last_error = 0.0;
			zone_table[1].pid_tbl->last_temp = FSC_TEMP_INVALID;
		}
		zone_table[1].pid_tbl = NULL;
		zone_table[1].sw_tbl = pump_stepwise_auto_mode_table;
	}
}

void change_temp_setpoint(uint8_t onoff)
{
	zone_table[0].pid_tbl->integral = 0.0;
	zone_table[0].pid_tbl->last_error = 0.0;
	zone_table[0].pid_tbl->last_temp = FSC_TEMP_INVALID;
	zone_table[0].pid_tbl->setpoint =
		onoff ? get_fsc_setpoint(SETPOINT_FLAG_OUTLET_TEMP) : 40.0;
}

/**
 * @brief Function to control the FSC thread.
 *
 * @param action The specified action, can be FSC_DISABLE (turn off) or FSC_ENABLE (turn on).
 */
void controlFSC(uint8_t action)
{
	fsc_poll_flag = (action == FSC_DISABLE) ? 0 : 1;
	if (action == FSC_ENABLE)
		zone_init();
}

static void fsc_thread_handler(void *arug0, void *arug1, void *arug2)
{
	CHECK_NULL_ARG(arug0);
	CHECK_NULL_ARG(arug1);
	ARG_UNUSED(arug2);

	zone_cfg *fsc_zone_table = (zone_cfg *)arug0;
	uint32_t fsc_zone_table_size = POINTER_TO_UINT(arug1);

	LOG_INF("fsc_thread_handler fsc_zone_table %p, fsc_zone_table_size %d", fsc_zone_table,
		fsc_zone_table_size);

	const int fsc_poll_interval_ms = 1000;

	while (1) {
		k_msleep(fsc_poll_interval_ms);

		if (!fsc_poll_flag)
			continue;

		for (uint8_t i = 0; i < fsc_zone_table_size; i++) {
			uint16_t duty = 0;
			uint8_t tmp_duty = 0;

			FSC_PRINTF("---------- fsc zone %d\n", i);
			zone_cfg *zone_p = fsc_zone_table + i;
			if (zone_p == NULL)
				continue;

			// poll interval
			(zone_p->fsc_poll_count)++;
			if (zone_p->fsc_poll_count >= zone_p->interval)
				zone_p->fsc_poll_count = 0;
			else
				continue;

			if (zone_p->sw_tbl) {
				calculateStepwise(zone_p, &tmp_duty);
				duty += tmp_duty;
			}

			tmp_duty = 0;
			if (zone_p->pid_tbl) {
				calculatePID(zone_p, &tmp_duty);
				duty += tmp_duty;
			}

			FSC_PRINTF("sw + pid duty %d\n", duty);

			if (zone_p->is_init) {
				if (zone_p->slew_neg) {
					uint8_t min_out = zone_p->last_duty - zone_p->slew_neg;
					if (duty < min_out)
						duty = min_out;
				}

				if (zone_p->slew_pos) {
					uint8_t max_out = zone_p->last_duty + zone_p->slew_pos;
					if (duty > max_out)
						duty = max_out;
				}
			}

			FSC_PRINTF("slew duty %d\n", duty);
			duty = CLAMP(duty, zone_p->out_limit_min, zone_p->out_limit_max);
			FSC_PRINTF("clamped duty %d\n", duty);

			zone_p->last_duty = duty;
			zone_p->is_init = true;

			// set_duty
			if (zone_p->set_duty)
				zone_p->set_duty(zone_p->set_duty_arg,
						 (fsc_tbl_enable ? duty : 70));
			else
				LOG_ERR("FSC zone %d set duty function is NULL", i);
		}
	}
}

void fsc_init(void)
{
	zone_init();
	controlFSC(FSC_ENABLE);

	LOG_INF("fsc_init zone_table %p, zone_table_size %d", zone_table, zone_table_size);

	k_thread_create(&fsc_thread, fsc_thread_stack, K_THREAD_STACK_SIZEOF(fsc_thread_stack),
			fsc_thread_handler, zone_table, UINT_TO_POINTER(zone_table_size), NULL,
			CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&fsc_thread, "fsc_thread");
}