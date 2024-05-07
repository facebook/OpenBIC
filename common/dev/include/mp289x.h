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
#ifndef MP289X_H
#define MP289X_H

#include "stdint.h"

enum {
	MODE_USR,
	MODE_MULTI_CFG_12,
	MODE_MULTI_CFG_34,
	MODE_MULTI_CFG_56,
	MODE_MULTI_CFG_ALL,
};

bool mp289x_fwupdate(uint8_t bus, uint8_t addr, uint8_t *img_buff, uint32_t img_size);
bool mp289x_crc_get(uint8_t bus, uint8_t addr, uint8_t mode, uint16_t *crc);

#endif
