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

bool pal_sensor_drive_init(sensor_cfg *cfg, uint8_t *init_status);
bool pal_sensor_drive_read(sensor_cfg *cfg, int *reading, uint8_t *sensor_status);
uint8_t pal_tmp75_init(sensor_cfg *cfg);
uint8_t pal_tmp75_read(sensor_cfg *cfg, int *reading);
uint8_t pal_emc1412_init(sensor_cfg *cfg);
uint8_t pal_emc1412_read(sensor_cfg *cfg, int *reading);
uint8_t pal_nvme_init(sensor_cfg *cfg);
uint8_t pal_nvme_read(sensor_cfg *cfg, int *reading);
uint8_t pal_ina233_init(sensor_cfg *cfg);
uint8_t pal_ina233_read(sensor_cfg *cfg, int *reading);
uint8_t pal_ltc2991_init(sensor_cfg *cfg);
uint8_t pal_ltc2991_read(sensor_cfg *cfg, int *reading);
uint8_t pal_xdpe12284c_init(sensor_cfg *cfg);
uint8_t pal_xdpe12284c_read(sensor_cfg *cfg, int *reading);
bool cxl_single_ioexp_init(uint8_t ioexp_name);
int cxl_ioexp_init(uint8_t cxl_channel);

#endif
