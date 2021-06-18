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

#include "cmsis_os2.h"
#include <stdio.h>

static uint32_t sys_tick_freq;

uint32_t util_get_us_tick(uint32_t time) {
  return time * sys_tick_freq / 1000000; 
}

uint32_t util_get_ms_tick(uint32_t time) {
  return time * sys_tick_freq / 1000;
}

uint32_t util_get_s_tick(uint32_t time) {
  return time * sys_tick_freq;
}

void util_init_timer(void) {
  sys_tick_freq = osKernelGetSysTimerFreq(); // get sys tick per second
  return;
}
