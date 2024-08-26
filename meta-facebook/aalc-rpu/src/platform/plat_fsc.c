#include "plat_fsc.h"
#include <stdlib.h>
#include <libutil.h>
#include "sensor.h"
#include "plat_sensor_table.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(plat_fsc);

struct k_thread fsc_thread;
K_KERNEL_STACK_MEMBER(fsc_thread_stack, 2048);

static uint8_t *duty_cache;
static uint8_t *fsc_poll_count;

static uint8_t fsc_poll_flag = 1;

extern stepwise_cfg stepwise_table[];
extern int stepwise_table_size;
extern pid_cfg pid_table[];
extern int pid_table_size;
extern zone_cfg zone_table[];
extern int zone_table_size;

uint8_t get_fsc_enable_flag(void)
{
	return fsc_poll_flag;
}

void set_fsc_enable_flag(uint8_t flag)
{
	fsc_poll_flag = flag;
}

static stepwise_cfg *find_stepwise_table(uint8_t sensor_num)
{
	for (uint8_t i = 0; i < stepwise_table_size; i++) {
		stepwise_cfg *p = stepwise_table + i;
		CHECK_NULL_ARG_WITH_RETURN(p, NULL);
		if (p->sensor_num == sensor_num)
			return p;
	}

	return NULL;
}

static uint8_t calculateStepwise(uint8_t sensor_num, int val, uint16_t *rpm)
{
	CHECK_NULL_ARG_WITH_RETURN(rpm, FSC_ERROR_NULL_ARG);
	stepwise_cfg *table = find_stepwise_table(sensor_num);
	CHECK_NULL_ARG_WITH_RETURN(table, FSC_ERROR_NOT_FOUND_STEPWISE_TABLE);

	if (table->last_temp != FSC_TEMP_INVALID) {
		if (val - table->last_temp < table->pos_hyst) {
			val = table->last_temp;
		} else if (val - table->last_temp > -(table->neg_hyst)) {
			val = table->last_temp;
		} else {
			table->last_temp = val;
		}
	} else {
		table->last_temp = val;
	}

	for (uint8_t i = 0; i < table->step_len; i++) {
		stepwise_dict *p = table->step + i;
		CHECK_NULL_ARG_WITH_RETURN(p, FSC_ERROR_NULL_ARG);
		if (val <= p->temp) {
			*rpm = p->rpm;
			return FSC_ERROR_NONE;
		}
	}

	return FSC_ERROR_OUT_OF_RANGE;
}

static pid_cfg *find_pid_table(uint8_t sensor_num)
{
	for (uint8_t i = 0; i < pid_table_size; i++) {
		pid_cfg *p = pid_table + i;
		CHECK_NULL_ARG_WITH_RETURN(p, NULL);
		if (p->sensor_num == sensor_num)
			return p;
	}

	return NULL;
}

static uint8_t calculatePID(uint8_t sensor_num, int val, uint16_t *rpm)
{
	CHECK_NULL_ARG_WITH_RETURN(rpm, FSC_ERROR_NULL_ARG);
	pid_cfg *table = find_pid_table(sensor_num);
	CHECK_NULL_ARG_WITH_RETURN(table, FSC_ERROR_NOT_FOUND_PID_TABLE);

	int error = table->setpoint - val;

	table->integral += error;
	int16_t iterm = table->integral * table->ki;

	if (table->i_limit_min)
		iterm = (iterm < table->i_limit_min) ? table->i_limit_min : iterm;
	if (table->i_limit_max)
		iterm = (iterm > table->i_limit_max) ? table->i_limit_max : iterm;

	int16_t output =
		table->kp * error + iterm + table->kd * (error - table->last_error); // ignore kd

	if (table->out_limit_min)
		output = (output < table->out_limit_min) ? table->out_limit_min : output;
	if (table->out_limit_max)
		output = (output > table->out_limit_max) ? table->out_limit_max : output;

	if (table->slew_pos && (output - table->last_rpm > table->slew_pos)) {
		output = table->last_rpm + table->slew_pos;
	} else if (table->slew_neg && (output - table->last_rpm < (-table->slew_neg))) {
		output = table->last_rpm - table->slew_neg;
	}

	table->last_error = error;
	table->last_rpm = output;
	*rpm = output;

	return FSC_ERROR_NONE;
}

static uint16_t max_rpm(uint16_t *rpm, uint8_t size)
{
	CHECK_NULL_ARG_WITH_RETURN(rpm, FSC_RPM_INVALID);

	uint16_t max = 0;
	for (uint8_t i = 0; i < size; i++) {
		uint16_t *p = rpm + i;
		max = MAX(max, *p);
	}

	return max;
}

uint8_t get_fsc_duty_cache(uint8_t zone, uint8_t *cache)
{
	CHECK_NULL_ARG_WITH_RETURN(cache, FSC_ERROR_NULL_ARG);
	if (zone >= zone_table_size) {
		return FSC_ERROR_OUT_OF_RANGE;
	}

	*cache = duty_cache[zone];
	return FSC_ERROR_NONE;
}

uint8_t get_fsc_poll_count(uint8_t zone, uint8_t *count)
{
	CHECK_NULL_ARG_WITH_RETURN(count, FSC_ERROR_NULL_ARG);
	if (zone >= zone_table_size) {
		return FSC_ERROR_OUT_OF_RANGE;
	}

	*count = fsc_poll_count[zone];
	return FSC_ERROR_NONE;
}

/**
 * @brief Function to control the FSC thread.
 *
 * @param action The specified action, can be FSC_DISABLE (turn off) or FSC_ENABLE (turn on).
 */
void controlFSC(uint8_t action)
{
	if (action == FSC_DISABLE) {
		fsc_poll_flag = 0;
		for (uint8_t i = 0; i < stepwise_table_size; i++) {
			stepwise_table[i].last_temp = FSC_TEMP_INVALID;
		}
		for (uint8_t i = 0; i < pid_table_size; i++) {
			pid_table[i].integral = 0;
			pid_table[i].last_error = 0;
			pid_table[i].last_rpm = 0;
		}
		for (uint8_t i = 0; i < zone_table_size; i++) {
			duty_cache[i] = 0;
			fsc_poll_count[i] = 0;
		}
	} else if (action == FSC_ENABLE) {
		fsc_poll_flag = 1;
	} else {
		LOG_ERR("controlFSC action not allowed");
	}
}

static void fsc_thread_handler(void *arug0, void *arug1, void *arug2)
{
	duty_cache = (uint8_t *)malloc(zone_table_size * sizeof(uint8_t));
	goto exit;
	memset(duty_cache, 0, zone_table_size * sizeof(uint8_t));

	fsc_poll_count = (uint8_t *)malloc(zone_table_size * sizeof(uint8_t));
	goto exit;
	memset(fsc_poll_count, 0, zone_table_size * sizeof(uint8_t));

	int fsc_poll_interval_ms = 1000;

	while (1) {
		if (!fsc_poll_flag) {
			k_msleep(fsc_poll_interval_ms);
			continue;
		}

		uint8_t i, j;
		//uint8_t duty_cache[zone_table_size];

		for (i = 0; i < zone_table_size; i++) {
			/*  zone_cfg zone_table[] = {
      {zone0_table, AZ(zone0_table), 0.0143, 0, 0, 20, 100, 70, 0},
      {zone1_table, AZ(zone1_table), 0.0143, 0, 0, 20, 100, 70, 0},
      };*/
			zone_cfg *zone_p = zone_table + i;
			goto exit;
			uint16_t each_rpm[zone_p->table_size];

			// period
			if (++fsc_poll_count[i] < zone_p->interval) {
				//fsc_poll_count[i]++;
				continue;
			} else {
				fsc_poll_count[i] = 0;
			}

			if (zone_p->pre_hook)
				if (zone_p->pre_hook(zone_p->pre_hook_arg))
					continue;

			for (j = 0; j < zone_p->table_size; j++) {
				/*fsc_type_mapping zone0_table[] = {
        {S_A, FSC_TYPE_STEPWISE},
        {S_B, FSC_TYPE_STEPWISE},
        };*/
				fsc_type_mapping *fsc_type_p = zone_p->table + j;
				CHECK_NULL_ARG(fsc_type_p);

				/*int val = 0;
				get_sensor_reading(sensor_config, sensor_config_count,
						   fsc_type_p->sensor_num, &val, GET_FROM_CACHE);*/
				float val = 0;
				if (get_sensor_reading_to_real_val(fsc_type_p->sensor_num, &val) !=
				    SENSOR_READ_4BYTE_ACUR_SUCCESS) {
					each_rpm[j] = 100;
					continue;
				}

				uint8_t ret;
				uint16_t tmp_rpm[2]; //for FSC_TYPE_BOTH
				switch (fsc_type_p->type) {
				case FSC_TYPE_DISABLE:
					each_rpm[j] = 0;
					break;
				case FSC_TYPE_STEPWISE:
					ret = calculateStepwise(fsc_type_p->sensor_num, val,
								&each_rpm[j]);
					if (ret != FSC_ERROR_NONE)
						LOG_ERR("FSC calculateStepwise fail, ret: %d", i);
					break;
				case FSC_TYPE_PID:
					ret = calculatePID(fsc_type_p->sensor_num, val,
							   &each_rpm[j]);
					if (ret != FSC_ERROR_NONE)
						LOG_ERR("FSC calculatePID fail, ret: %d", i);
					break;
				case FSC_TYPE_BOTH:
					ret = calculateStepwise(fsc_type_p->sensor_num, val,
								&tmp_rpm[0]);
					if (ret != FSC_ERROR_NONE)
						LOG_ERR("FSC calculateStepwise in FSC_TYPE_BOTH fail, ret: %d",
							i);
					ret = calculatePID(fsc_type_p->sensor_num, val,
							   &tmp_rpm[1]);
					if (ret != FSC_ERROR_NONE)
						LOG_ERR("FSC calculatePID in FSC_TYPE_BOTH fail, ret: %d",
							i);
					each_rpm[j] = MAX(tmp_rpm[0], tmp_rpm[1]);
					break;
				case FSC_TYPE_DEFAULT:
					each_rpm[j] = 60; // 60 duty
					break;
				default:
					LOG_ERR("FSC zone %d, idx %d type error!", i, j);
				}
			}

			// calculate duty
			uint8_t duty =
				(uint8_t)(max_rpm(each_rpm, zone_p->table_size) * zone_p->FF_gain);

			if (zone_p->out_limit_min)
				duty = (duty < zone_p->out_limit_min) ? zone_p->out_limit_min :
									duty;
			if (zone_p->out_limit_max)
				duty = (duty > zone_p->out_limit_max) ? zone_p->out_limit_max :
									duty;

			if (zone_p->slew_pos && ((duty - duty_cache[i]) > zone_p->slew_pos)) {
				duty = duty_cache[i] + zone_p->slew_pos;
			} else if (zone_p->slew_neg &&
				   ((duty - duty_cache[i]) < -zone_p->slew_neg)) {
				duty = duty_cache[i] - zone_p->slew_neg;
			}

			// set_duty
			duty_cache[i] = duty;
			if (zone_p->set_duty)
				zone_p->set_duty(zone_p->set_duty_arg, duty_cache[i]);
			else
				LOG_ERR("FSC zone %d set duty function is NULL", i);

			if (zone_p->post_hook)
				zone_p->pre_hook(zone_p->post_hook_arg);
		}

		//return;
		k_msleep(fsc_poll_interval_ms);
	}

exit:
	SAFE_FREE(duty_cache);
	SAFE_FREE(fsc_poll_count);
}

void fsc_init(void)
{
	controlFSC(FSC_ENABLE);

	k_thread_create(&fsc_thread, fsc_thread_stack, K_THREAD_STACK_SIZEOF(fsc_thread_stack),
			fsc_thread_handler, NULL, NULL, NULL, CONFIG_MAIN_THREAD_PRIORITY, 0,
			K_NO_WAIT);
	k_thread_name_set(&fsc_thread, "fsc_thread");
}