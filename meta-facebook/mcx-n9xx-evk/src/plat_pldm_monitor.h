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

#ifndef PLAT_PLDM_MONITOR_H
#define PLAT_PLDM_MONITOR_H

#include <stdint.h>

/* Build the PDR repository and probe the die-temp sensor. Call once at
 * boot, after the sensor subsystem is up. Safe to call even if the
 * die_temp devicetree node is absent (sensor then reports UNAVAILABLE).
 */
void plat_pldm_monitor_init(void);

/* Local (no MCTP) read of the SoC die temperature in milli-degrees C,
 * for the "plat pldm2 temp" shell command. Returns 0 / -errno. */
int plat_pldm_monitor_read_die_temp_mdegc(int32_t *out_mdegc);

/* Number of records currently in the board PDR repository. */
uint32_t plat_pldm_monitor_pdr_count(void);

#endif /* PLAT_PLDM_MONITOR_H */
