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

#ifndef PLAT_DEV
#define PLAT_DEV

#include <stdint.h>
#include "sensor.h"

#define NVME_TEMP_OFFSET 0x00
#define NVME_VOLTAGE_RAIL_1_OFFSET 0x75
#define NVME_VOLTAGE_RAIL_2_OFFSET 0x77

bool pal_sensor_drive_init(uint8_t card_id, sensor_cfg *cfg, uint8_t *init_status);
bool pal_sensor_drive_read(uint8_t card_id, sensor_cfg *cfg, int *reading, uint8_t *sensor_status);
uint8_t pal_nvme_init(uint8_t card_id, sensor_cfg *cfg);
uint8_t pal_nvme_read(uint8_t card_id, sensor_cfg *cfg, int *reading);
uint8_t pal_ina233_init(uint8_t card_id, sensor_cfg *cfg);
uint8_t pal_ina233_read(uint8_t card_id, sensor_cfg *cfg, int *reading);

#endif
