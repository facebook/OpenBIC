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

enum IRIS_TEMP_INDEX_E {
	TEMP_STATUS_INDEX_ASIC_MEDHA0_SENSOR0,
	TEMP_STATUS_INDEX_ASIC_MEDHA0_SENSOR1,
	TEMP_STATUS_INDEX_ASIC_OWL_W,
	TEMP_STATUS_INDEX_ASIC_OWL_E,
	TEMP_STATUS_INDEX_ASIC_MEDHA1_SENSOR0,
	TEMP_STATUS_INDEX_ASIC_MEDHA1_SENSOR1,
	TEMP_STATUS_INDEX_ASIC_HAMSA_CRM,
	TEMP_STATUS_INDEX_ASIC_HAMSA_LS,
	TEMP_STATUS_INDEX_MAX,
};

#define TEMP_STATUS_H_LIMIT BIT(4)
#define TEMP_STATUS_L_LIMIT BIT(3)
#define TEMP_STATUS_OPEN BIT(2)
#define TEMP_LIMIT_STATUS (TEMP_STATUS_H_LIMIT | TEMP_STATUS_L_LIMIT)
#define H_LIMIT_STATUS 0x35
#define L_LIMIT_STATUS 0x36

void init_thermal_polling(void);
uint8_t get_thermal_status_val_for_log(uint8_t sensor_num);
uint8_t get_thermal_limit_status_val_for_log(uint8_t sensor_num);

#endif