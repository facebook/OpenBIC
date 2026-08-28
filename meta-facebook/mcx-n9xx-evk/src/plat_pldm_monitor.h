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

#include <stdbool.h>
#include <stdint.h>

/* Build the PDR repository and probe the backing hardware (die-temp
 * sensor, SW2 button, LED effecter). Call once at boot, after the
 * sensor subsystem is up. Safe to call if any backing node is absent.
 */
void plat_pldm_monitor_init(void);

/* Local (no MCTP) accessors for the `plat pldm2 ...` shell commands -
 * they read/drive the same board state the MCTP-facing handlers use. */

/* Die temperature in milli-degrees C. Returns 0 / -errno. */
int plat_pldm_monitor_read_die_temp_mdegc(int32_t *out_mdegc);

/* SW2 as a DSP0249 Presence state: PLDM_STATE_SET_PRESENT / _NOT_PRESENT. */
uint8_t plat_pldm_monitor_sw2_state(void);

/* Drive the LED effecter. Returns 0 / -errno. */
int plat_pldm_monitor_set_led(bool on);

/* Current LED effecter state (OEM device-status set: 1=off, 2=on). */
uint8_t plat_pldm_monitor_led_state(void);

/* Number of records in the board PDR repository. */
uint32_t plat_pldm_monitor_pdr_count(void);

#endif /* PLAT_PLDM_MONITOR_H */
