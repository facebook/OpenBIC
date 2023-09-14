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

#ifndef NVME_H
#define NVME_H

#include <stdint.h>

#define NVME_TEMP_OFFSET 0x00
#define NVME_CORE_VOLTAGE_1_OFFSET 0x71
#define NVME_CORE_VOLTAGE_2_OFFSET 0x73
#define NVME_VOLTAGE_RAIL_1_OFFSET 0x75
#define NVME_VOLTAGE_RAIL_2_OFFSET 0x77

int read_nvme_info(uint8_t bus, uint8_t addr, uint8_t offset, uint8_t read_len, uint8_t *data);
uint8_t nvme_init(sensor_cfg *cfg);

#endif
