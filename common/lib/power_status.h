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

#ifndef POWER_STATUS_H
#define POWER_STATUS_H

#include <stdbool.h>
#include <stdint.h>

void set_DC_status(uint8_t gpio_num);
bool get_DC_status();
void set_DC_on_delayed_status();
void set_DC_on_delayed_status_with_value(bool status);
bool get_DC_on_delayed_status();
void set_DC_off_delayed_status();
bool get_DC_off_delayed_status();
void set_post_status(uint8_t gpio_num);
void set_post_complete(bool status);
bool get_post_status();
void set_CPU_power_status(uint8_t gpio_num);
bool CPU_power_good();
void set_post_thread();
void set_vr_monitor_status(bool value);
bool get_vr_monitor_status();

#endif
