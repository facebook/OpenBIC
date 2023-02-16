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

#ifndef PT5161L_H
#define PT5161L_H

#include "hal_i2c.h"

#define PT5161L_MAIN_MICRO_FW_INFO (96 * 1024 - 128)
#define PT5161L_MM_FW_VERSION_MAJOR 0
#define PT5161L_MM_FW_VERSION_MINOR 1
#define PT5161L_MM_FW_VERSION_BUILD 2
#define PT5161L_MAIN_SRAM_DMEM_OFFSET (64 * 1024)
#define PT5161L_TG_RD_LOC_IND_SRAM 0x16
#define PT5161L_TEMP_OFFSET 0x42c
#define PT5161L_MAIN_MICRO_INDIRECT 0xd99

#define PT5161L_MUTEX_LOCK_MS 1000

bool get_retimer_fw_version(I2C_MSG *msg, uint8_t *version);

#endif
