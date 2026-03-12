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

#ifndef PLAT_THERMAL_H
#define PLAT_THERMAL_H

#include <stdint.h>
#include "plat_user_setting.h"

#define TEMP_STATUS_H_LIMIT BIT(4)
#define TEMP_STATUS_L_LIMIT BIT(3)
#define TEMP_STATUS_OPEN BIT(2)
#define TEMP_LIMIT_STATUS (TEMP_STATUS_H_LIMIT | TEMP_STATUS_L_LIMIT)
#define H_LIMIT_STATUS 0x35
#define L_LIMIT_STATUS 0x36

void init_thermal_polling(void);
uint8_t get_thermal_status_val_for_log(uint8_t sensor_num);
uint8_t get_thermal_limit_status_val_for_log(uint8_t sensor_num);
bool plat_clear_temp_status(uint8_t rail);
bool plat_get_temp_status(uint8_t rail, uint8_t *temp_status);

typedef struct temp_mapping_sensor {
	uint8_t index;
	uint8_t sensor_id;
	uint8_t *sensor_name;
} temp_mapping_sensor;

extern temp_mapping_sensor temp_index_table[TEMP_INDEX_MAX];

#endif