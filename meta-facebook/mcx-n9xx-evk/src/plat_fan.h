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
 *
 * Required by common/service/ipmi/oem_handler.c's include chain. This
 * board has no fans - pal_set_fan_duty() (src/plat_stubs.c) always
 * fails, honestly reporting "not supported" rather than pretending to
 * control real hardware.
 */

#ifndef PLAT_FAN_H
#define PLAT_FAN_H

#include <stdint.h>

#define MAX_FAN_DUTY_VALUE 100
#define MAX_FAN_PWM_INDEX_COUNT 0
#define INDEX_ALL_PWM 0xFF

enum { FAN_AUTO_MODE = 0, FAN_MANUAL_MODE = 1 };

int pal_set_fan_duty(uint8_t pwm_id, uint8_t duty, uint8_t slot_index);
int pal_get_fan_ctrl_mode(uint8_t *mode);
void pal_set_fan_ctrl_mode(uint8_t mode);

#endif
