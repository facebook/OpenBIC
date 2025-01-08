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

#ifndef BCM85658_H
#define BCM85658_H

#include "hal_i2c.h"

#define BCM85658_MUTEX_LOCK_MS 1000
#define IS_BCM85658_TEMP_VAILD(r) (((r) >> 11) & 0x1)
#define BCM85658_TEMP_SENSOR_DATA_GET(r) ((r) & 0x7ff)

#define INTEL_SMBUS_RD32_4ADDR_D 0x09
#define SMBUS_WR_BLOCK_CMD 0xA7
#define SMBUS_PROCESS_BLOCK_CMD 0xA9
#define BCM85658_TEMP_OFFSET 0x981c
#define BCM85658_TEMP_OFFSET_32B 0x6000981c
#define BCM85658_VERSION_OFFSET 0x600004a0

bool bcm85658_get_fw_version(I2C_MSG *msg, uint8_t *version);
uint8_t pcie_retimer_fw_update(I2C_MSG *msg, uint32_t offset, uint16_t msg_len, uint8_t *msg_buf,
			       uint8_t flag);

#endif
