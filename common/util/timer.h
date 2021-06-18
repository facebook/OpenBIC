/*
 * Copyright (c) Facebook, Inc. and its affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef TIMER_H
#define TIMER_H

#define time_1000_ms 1000
#define time_500_ms  500
#define time_100_ms  100

uint32_t util_get_us_tick(uint32_t time);
uint32_t util_get_ms_tick(uint32_t time);
uint32_t util_get_s_tick(uint32_t time);
void util_init_timer(void);

#endif
