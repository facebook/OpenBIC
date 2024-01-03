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

#ifndef PLAT_CPU_H
#define PLAT_CPU_H

#define CPU_TIME_UNIT 100000000

#define MONITOR_CPU_STACK_SIZE 600
#define MONITOR_CPU_TIME_MS 1000
#define MONITOR_SMIOUT_STACK_SIZE 400
#define MONITOR_SMIOUT_TIME_MS 1000

#define SMIOUT_INDEX 9
#define SMIOUT_TIMEOUT 90

#define RDPKG_IDX_PKG_THERMAL_STATUS 0x14
#define THERMAL_STATUS_DEASSERT 0
#define THERMAL_STATUS_ASSERT 1

void monitor_cpu_handler();
void start_monitor_cpu_thread();
void monitor_smiout_handler();
void start_monitor_smi_thread();

#endif
