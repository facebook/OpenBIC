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

#ifndef PLAT_UTIL_H
#define PLAT_UTIL_H

#include <stdio.h>
#include <stdint.h>

bool plat_i2c_read(uint8_t bus, uint8_t addr, uint8_t offset, uint8_t *data, uint8_t len);
bool plat_i2c_write(uint8_t bus, uint8_t addr, uint8_t offset, uint8_t *data, uint8_t len);

#endif