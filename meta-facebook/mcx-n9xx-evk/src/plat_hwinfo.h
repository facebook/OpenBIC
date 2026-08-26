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

#ifndef PLAT_HWINFO_H
#define PLAT_HWINFO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Reads and logs the chip's unique device ID and the cause of the last
 * reset - stand-ins for the device-identity data a real BIC reports
 * over IPMI (Get Device ID, Get Device GUID). Clears the reset-cause
 * latch after reading, per hwinfo API convention.
 */
void plat_hwinfo_init(void);

/* Copies the 16-byte device ID read by plat_hwinfo_init() into out
 * (out_len must be >= 16). Returns false if hwinfo_get_device_id()
 * never succeeded at boot. Used as this board's real, unique value for
 * MCTP Control's Get Endpoint UUID (see plat_mctp.c) - a genuine
 * per-chip silicon ID rather than a fabricated placeholder.
 */
bool plat_get_device_id(uint8_t *out, size_t out_len);

#endif
