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

#include <zephyr.h>
#include <sys/printk.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <shell/shell.h>

#include "sensor.h"
#include "plat_gpio.h"
#include "plat_sensor_table.h"

#define MAX_E1S_NUMBER 16
#define E1S_POWER_CLOCK_PERIOD 2
#define E1S_POWER_AVG_PERIOD 250

struct stress_e1s_pwr_s {
	uint8_t e1s_idx;
	uint32_t time_count_ms;
	struct k_timer timer;
	struct k_work work;
	struct k_work stop_work;
	const struct shell *shell;
};

uint8_t e1s_power_sensor_table[MAX_E1S_NUMBER] = {
	SENSOR_NUM_POUT_E1S_0,	SENSOR_NUM_POUT_E1S_1,	SENSOR_NUM_POUT_E1S_2,
	SENSOR_NUM_POUT_E1S_3,	SENSOR_NUM_POUT_E1S_4,	SENSOR_NUM_POUT_E1S_5,
	SENSOR_NUM_POUT_E1S_6,	SENSOR_NUM_POUT_E1S_7,	SENSOR_NUM_POUT_E1S_8,
	SENSOR_NUM_POUT_E1S_9,	SENSOR_NUM_POUT_E1S_10, SENSOR_NUM_POUT_E1S_11,
	SENSOR_NUM_POUT_E1S_12, SENSOR_NUM_POUT_E1S_13, SENSOR_NUM_POUT_E1S_14,
	SENSOR_NUM_POUT_E1S_15,
};

uint8_t e1s_present_pin_table[MAX_E1S_NUMBER] = {
	PRSNT_SSD0_R_N,	 PRSNT_SSD1_R_N,  PRSNT_SSD2_R_N,  PRSNT_SSD3_R_N,
	PRSNT_SSD4_R_N,	 PRSNT_SSD5_R_N,  PRSNT_SSD6_R_N,  PRSNT_SSD7_R_N,
	PRSNT_SSD8_R_N,	 PRSNT_SSD9_R_N,  PRSNT_SSD10_R_N, PRSNT_SSD11_R_N,
	PRSNT_SSD12_R_N, PRSNT_SSD13_R_N, PRSNT_SSD14_R_N, PRSNT_SSD15_R_N,
};

void e1s_pwr_work_handler(struct k_work *work)
{
	static uint16_t period_count = E1S_POWER_AVG_PERIOD;
	static float avg = 0.0;
	int reading = 0;

	struct stress_e1s_pwr_s *data = CONTAINER_OF(work, struct stress_e1s_pwr_s, work);

	if (period_count > 0) {
		if (!get_sensor_reading(e1s_power_sensor_table[data->e1s_idx], &reading,
					GET_FROM_SENSOR))
			shell_warn(data->shell, "[%s] Reading E1S dev%d power failed \n", __func__,
				   data->e1s_idx);
		period_count--;
		sensor_val *sval = (sensor_val *)(&reading);
		avg += (sval->integer) + (sval->fraction * 0.001);
	} else if (period_count == 0) {
		period_count = E1S_POWER_AVG_PERIOD;
		avg /= E1S_POWER_AVG_PERIOD;
		shell_print(data->shell, "E1S dev%d power avg = %.2f Watts\n", data->e1s_idx, avg);
		avg = 0.0;
	}
}

void e1s_pwr_stop_work_handler(struct k_work *work)
{
	struct stress_e1s_pwr_s *data = CONTAINER_OF(work, struct stress_e1s_pwr_s, stop_work);

	if (data) {
		shell_print(data->shell, "==== Stop stress E1S dev%d power consumption ==== \n",
			    data->e1s_idx);
		free(data);
		data = NULL;
	}
}

static void e1s_pwr_timer_stop_handler(struct k_timer *timer)
{
	struct stress_e1s_pwr_s *data = CONTAINER_OF(timer, struct stress_e1s_pwr_s, timer);
	k_work_submit(&data->stop_work);
}

static void e1s_pwr_timer_handler(struct k_timer *timer)
{
	struct stress_e1s_pwr_s *data = CONTAINER_OF(timer, struct stress_e1s_pwr_s, timer);

	if (data->time_count_ms > 0) {
		data->time_count_ms--;
		k_work_submit(&data->work);
	} else {
		k_timer_stop(timer);
	}
}

int cmd_stress_e1s_pwr(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_warn(shell, "Usage: test e1s power [device index] [duration (minute)]");
		return -1;
	}

	uint8_t e1s_idx = strtoul(argv[1], NULL, 10);
	uint16_t duration = strtoul(argv[2], NULL, 10);

	if (e1s_idx >= MAX_E1S_NUMBER) {
		shell_warn(shell, "Device index out of range \n");
		return -1;
	}

	if (gpio_get(e1s_present_pin_table[e1s_idx])) {
		shell_warn(shell, "E1S dev%d not present \n", e1s_idx);
		return -1;
	}
	struct stress_e1s_pwr_s *data = malloc(sizeof(struct stress_e1s_pwr_s));

	if (!data) {
		shell_warn(shell, "Malloc fail\n");
		return -1;
	}

	data->e1s_idx = e1s_idx;
	data->time_count_ms = (duration * 60 * 1000) / E1S_POWER_CLOCK_PERIOD;
	data->shell = shell;

	k_timer_init(&data->timer, e1s_pwr_timer_handler, e1s_pwr_timer_stop_handler);

	k_work_init(&data->work, e1s_pwr_work_handler);
	k_work_init(&data->stop_work, e1s_pwr_stop_work_handler);

	shell_print(shell, "==== Start stress E1S dev%d power consumption %d minutes =====\n",
		    e1s_idx, duration);

	k_timer_start(&data->timer, K_NO_WAIT, K_MSEC(E1S_POWER_CLOCK_PERIOD));

	return 0;
}
