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
#ifndef MP2971_H
#define MP2971_H

#include "stdint.h"
#include "sensor.h"

bool mp2971_fwupdate(uint8_t bus, uint8_t addr, uint8_t *img_buff, uint32_t img_size);
bool mp2971_get_vout_max(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt);
bool mp2971_get_vout_min(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt);
bool mp2971_set_vout_max(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt);
bool mp2971_set_vout_min(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt);
bool mp2971_get_checksum(uint8_t bus, uint8_t addr, uint32_t *checksum);
bool mp2971_get_vout_command(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt);
bool mp2971_set_vout_command(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt);
bool mp2971_get_vr_status(sensor_cfg *cfg, uint8_t rail, uint8_t vr_status_rail,
			  uint16_t *vr_status);
bool mp2971_clear_vr_status(sensor_cfg *cfg, uint8_t rail);
bool mp2971_get_vout_offset(sensor_cfg *cfg, uint8_t rail, uint16_t *vout_offset);
bool mp2971_get_total_ocp(sensor_cfg *cfg, uint8_t rail, uint16_t *total_ocp);
bool mp2971_set_total_ocp(sensor_cfg *cfg, uint8_t rail, uint16_t total_ocp);
bool mp2971_get_ovp_1(sensor_cfg *cfg, uint8_t rail, uint16_t *ovp_1_mv);
bool mp2971_set_ovp_1(sensor_cfg *cfg, uint8_t rail, uint16_t *ovp_1_mv);
bool mp2971_get_ovp_2(sensor_cfg *cfg, uint8_t rail, uint16_t *ovp_1_mv);
bool mp2971_get_uvp(sensor_cfg *cfg, uint8_t rail, uint16_t *uvp_mv);
bool mp2971_set_ovp2_action_mode(sensor_cfg *cfg, uint8_t rail, uint8_t *mode);
bool mp2971_get_ovp2_action_mode(sensor_cfg *cfg, uint8_t rail, uint8_t *mode);
bool mp2971_set_thres_div_en(sensor_cfg *cfg, uint8_t rail, uint16_t *enable);
bool mp2971_set_uvp_threshold(sensor_cfg *cfg, uint8_t rail, uint16_t *write_uvp_threshold);

#endif
