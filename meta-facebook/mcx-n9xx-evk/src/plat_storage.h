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

#ifndef PLAT_STORAGE_H
#define PLAT_STORAGE_H

#include <stdint.h>

/* Mounts NVS on the "storage" partition (on the on-board W25Q64 QSPI
 * flash), reads and increments a persistent boot counter, and logs the
 * new value. Real BICs use this same mechanism for config/calibration
 * data that must survive a reset.
 */
void plat_storage_init(void);

/* Current boot counter value (as last read/written by plat_storage_init). */
uint32_t plat_storage_boot_count(void);

#endif
