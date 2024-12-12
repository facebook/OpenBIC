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
#ifndef ISL69259_H
#define ISL69259_H

#include "stdint.h"
#include "sensor.h"

#define TWO_COMPLEMENT_NEGATIVE_BIT BIT(15)
#define ADJUST_IOUT_RANGE 2

struct isl69259_config {
	uint8_t *buff;
	uint32_t len;
};

bool isl69260_get_vout_max(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt);
bool isl69260_get_vout_min(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt);
bool isl69260_set_vout_max(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt);
bool isl69260_set_vout_min(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt);
bool isl69259_fwupdate(uint8_t bus, uint8_t addr, uint8_t *img_buff, uint32_t img_size);
bool isl69259_get_raa_hex_mode(uint8_t bus, uint8_t addr, uint8_t *mode);
bool isl69259_get_raa_crc(uint8_t bus, uint8_t addr, uint8_t mode, uint32_t *crc);
bool get_raa_remaining_wr(uint8_t bus, uint8_t addr, uint8_t mode, uint16_t *remain);
bool isl69260_get_vout_command(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt);
bool isl69260_set_vout_command(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt);

#endif
