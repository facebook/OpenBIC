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

#ifndef PLDM_SENSOR_H
#define PLDM_SENSOR_H

#include <stdlib.h>
#include "sensor.h"
#include "pdr.h"

#define PLDM_SENSOR_POLL_STACK_SIZE 3056

#define PLDM_SENSOR_POLL_TIME_DEFAULT_MS 1000

typedef struct pldm_sensor_info {
	PDR_numeric_sensor pdr_numeric_sensor;
	uint32_t update_time;
	sensor_cfg pldm_sensor_cfg;
} pldm_sensor_info;

typedef struct pldm_sensor_thread {
	int thread_id;
	char *thread_name;
} pldm_sensor_thread;

void pldm_sensor_monitor_init();
void pldm_sensor_poll_thread_init();
void pldm_sensor_polling_handler(void *arug0, void *arug1, void *arug2);
void pldm_sensor_get_reading(sensor_cfg *pldm_sensor_cfg, uint32_t *update_time,
			     int pldm_sensor_count, int thread_id, int sensor_num);
uint8_t pldm_sensor_get_reading_from_cache(uint16_t sensor_id, int *reading,
					   uint8_t *sensor_operational_state);
bool pldm_sensor_is_interval_ready(pldm_sensor_info *pldm_sensor_list);
pldm_sensor_thread *plat_pldm_sensor_load_thread();
pldm_sensor_info *plat_pldm_sensor_load(int thread_id);
int plat_pldm_sensor_get_sensor_count(int thread_id);
int pldm_sensor_polling_pre_check(pldm_sensor_info *pldm_snr_list, int sensor_num);
int pldm_polling_sensor_reading(pldm_sensor_info *pldm_snr_list, int pldm_sensor_count,
				int thread_id, int sensor_num);

#endif
