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
#include <stdio.h>
#include "plat_led.h"
#include "plat_gpio.h"
#include <logging/log.h>
#include "plat_pldm_sensor.h"
#include "pldm_sensor.h"

LOG_MODULE_REGISTER(plat_led);

K_WORK_DELAYABLE_DEFINE(heartbeat_led_work, heartbeat_led_handler);

bool led_set_flag = false;

void set_led_flag(bool flag_value)
{
	led_set_flag = flag_value;
}

bool get_led_flag(void)
{
	return led_set_flag;
}

void heartbeat_led_handler(struct k_work *work)
{
	/* Lit heartbeat LED */
	static bool led_status = true;
	led_status = !led_status;
	LOG_DBG("heartbeat LED %s ", led_status ? "OFF" : "ON");
	gpio_set(LED_MMC_HEARTBEAT_R, led_status);
	uint16_t overall_interval = 500;

	uint32_t sensor_ids[] = {
		SENSOR_NUM_ASIC_MEDHA0_SENSOR0_TEMP_C, SENSOR_NUM_ASIC_MEDHA0_SENSOR1_TEMP_C,
		SENSOR_NUM_ASIC_OWL_W_TEMP_C,	       SENSOR_NUM_ASIC_OWL_E_TEMP_C,
		SENSOR_NUM_ASIC_MEDHA1_SENSOR0_TEMP_C, SENSOR_NUM_ASIC_MEDHA1_SENSOR1_TEMP_C,
		SENSOR_NUM_ASIC_HAMSA_CRM_TEMP_C,      SENSOR_NUM_ASIC_HAMSA_LS_TEMP_C
	};

	size_t num_sensors = sizeof(sensor_ids) / sizeof(sensor_ids[0]);

	for (size_t i = 0; i < num_sensors; i++) {
		uint32_t sensor_id = sensor_ids[i];
		int cache_reading = 0;
		uint8_t sensor_operational_state = PLDM_SENSOR_STATUSUNKOWN;

		uint8_t status = pldm_sensor_get_reading_from_cache(sensor_id, &cache_reading,
								    &sensor_operational_state);
		LOG_DBG("Sensor 0x%x: status = 0x%x, operational state = 0x%x", sensor_id, status,
			sensor_operational_state);

		// If the sensor reading fails, consider it critical and set the overall interval to 50 ms
		if (status != PLDM_SUCCESS) {
			overall_interval = 50;
			LOG_DBG("Sensor 0x%x read failed, setting overall_interval to 50ms",
				sensor_id);
			continue;
		}

		uint16_t candidate_interval = 500;
		if (cache_reading <= 70000) { //70 degree C
			if (led_set_flag) {
				// event occurred, set the interval to 2s per sec
				candidate_interval = 1000;
			} else {
				// normal, set the interval to 1s per sec
				candidate_interval = 500;
			}
		} else if (cache_reading <= 100000) { //100 degree C
			candidate_interval = 250; // 0.5 time per sec
		} else {
			candidate_interval = 50;
		}

		// Choose the most critical (lowest) interval among all sensors
		if (candidate_interval < overall_interval) {
			overall_interval = candidate_interval;
		}

		LOG_DBG("Sensor 0x%x reading: %d, candidate interval: %d ms", sensor_id,
			cache_reading, candidate_interval);
	}

	LOG_DBG("Overall heartbeat interval: %d ms", overall_interval);
	k_work_schedule(&heartbeat_led_work, K_MSEC(overall_interval));
}

void plat_led_init(void)
{
	k_work_schedule(&heartbeat_led_work, K_NO_WAIT);
	LOG_INF("Heartbeat LED timer started");
}
