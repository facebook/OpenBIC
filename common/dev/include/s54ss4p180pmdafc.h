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
#ifndef S54SS4P180PMDAFC_H
#define S54SS4P180PMDAFC_H

#include "stdint.h"
#include "sensor.h"

bool s54ss4p180pmdafc_get_vr_status(sensor_cfg *cfg, uint8_t rail, uint8_t vr_status_rail,
				    uint16_t *vr_status);
bool s54ss4p180pmdafc_clear_vr_status(sensor_cfg *cfg, uint8_t rail);

#endif
