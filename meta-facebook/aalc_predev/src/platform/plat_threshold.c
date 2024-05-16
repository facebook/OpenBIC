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
#include <logging/log.h>

#define THRESHOLD_POLL_STACK_SIZE 2048

struct k_thread threshold_poll;
K_KERNEL_STACK_MEMBER(threshold_poll_stack, THRESHOLD_POLL_STACK_SIZE);

LOG_MODULE_REGISTER(plat_threshold);

static bool threshold_poll_enable_flag = true;

typedef struct {
	uint8_t sensor_num;
	float lcr;
	float ucr;
} sensor_threshold;

sensor_threshold threshold_tbl[] = {
	{ SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_TEMP_C, -20, 100 },
	{ SENSOR_NUM_PB_1_PUMP_TACH_RPM, 0, 8000 },
};

void set_threshold_poll_enable_flag(bool flag)
{
	threshold_poll_enable_flag = flag;
}

bool get_threshold_poll_enable_flag()
{
	return threshold_poll_enable_flag;
}

void threshold_poll_handler(void *arug0, void *arug1, void *arug2)
{
	int threshold_poll_interval_ms = 1000; // interval 1s

	while (1) {
		if (!get_sensor_init_done_flag()) {
			k_msleep(threshold_poll_interval_ms);
			continue;
		}

		if (!get_sensor_poll_enable_flag()) {
			k_msleep(threshold_poll_interval_ms);
			continue;
		}

		if (!threshold_poll_enable_flag) {
			k_msleep(threshold_poll_interval_ms);
			continue;
		}

		for (uint8_t i = 0; i < ARRAY_SIZE(threshold_tbl); i++) {
			float val = get_sensor_reading_to_real_val(threshold_tbl[i].sensor_num);
			if (val < threshold_tbl[i].lcr)
				printf("0x%02x lcr %f/%f\n", threshold_tbl[i].sensor_num, val,
				       threshold_tbl[i].lcr);
			if (val > threshold_tbl[i].ucr)
				printf("0x%02x ucr %f/%f\n", threshold_tbl[i].sensor_num, val,
				       threshold_tbl[i].ucr);
			k_yield();
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