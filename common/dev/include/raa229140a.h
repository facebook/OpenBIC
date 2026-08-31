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
#ifndef RAA229140A_H
#define RAA229140A_H

#include "stdint.h"
#include "sensor.h"

bool raa229140a_fwupdate(uint8_t bus, uint8_t addr, uint8_t *img_buff, uint32_t img_size);
bool raa229140a_get_crc(uint8_t bus, uint8_t addr, uint32_t *crc);
int raa229140a_dma_read(uint8_t bus, uint8_t addr, uint8_t *reg, uint8_t *data);
int raa229140a_dma_write(uint8_t bus, uint8_t addr, uint8_t *reg, uint8_t *data);
bool raa229140a_get_vout_command(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt);
bool raa229140a_set_vout_command(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt);
bool raa229140a_get_iout_oc_warn_limit(sensor_cfg *cfg, uint16_t *value);
bool raa229140a_set_iout_oc_warn_limit(sensor_cfg *cfg, uint16_t value);
bool raa229140a_get_vr_status(sensor_cfg *cfg, uint8_t rail, uint8_t vr_status_rail,
			      uint16_t *vr_status);
bool raa229140a_clear_vr_status(sensor_cfg *cfg, uint8_t rail);
bool raa229140a_get_vout_offset(sensor_cfg *cfg, uint16_t *offset_return);
int raa229140a_get_hex_mode(uint8_t bus, uint8_t addr, uint8_t *mode);
int raa229140a_get_remaining_wr(uint8_t bus, uint8_t addr, uint8_t *remain);

#endif
