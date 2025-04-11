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
#include "pldm_sensor.h"
#include "pldm_monitor.h"
#include "plat_pldm_sensor.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(plat_led);

K_TIMER_DEFINE(heartbeat_led_timer, lit_heartbeat_led, NULL);

void lit_heartbeat_led(struct k_timer *timer)
{
	// Toggle heartbeat LED status
	static bool led_status = true;
	led_status = !led_status;
	LOG_DBG("heartbeat LED %s", led_status ? "OFF" : "ON");
	gpio_set(LED_NPCM_HEARTBEAT_R, led_status);

	uint16_t overall_interval = 500;

	uint32_t sensor_ids[] = { ASIC_DIE_ATH_SENSOR_0_TEMP_C, ASIC_DIE_ATH_SENSOR_1_TEMP_C,
				  ASIC_DIE_N_OWL_TEMP_C, ASIC_DIE_S_OWL_TEMP_C };
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
			candidate_interval = 500;
		} else if (cache_reading <= 100000) { //100 degree C
			candidate_interval = 250;
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
	k_timer_start(&heartbeat_led_timer, K_MSEC(overall_interval), K_NO_WAIT);
}

void plat_led_init(void)
{
	k_timer_start(&heartbeat_led_timer, K_NO_WAIT, K_NO_WAIT);
	LOG_INF("Heartbeat LED timer started");
}
