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

#ifndef HAL_JTAG_H
#define HAL_JTAG_H

#include <drivers/jtag.h>

void jtag_set_tap(uint8_t data, uint8_t bitlength);
void jtag_shift_data(uint16_t Wbit, uint8_t *Wdate, uint16_t Rbit, uint8_t *Rdate, uint8_t lastidx);

#endif
