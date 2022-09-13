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

#include <stdint.h>

#define SSD_TURN_OFF 0x00
#define SSD_TURN_ON 0x01
#define SSD_START_BLINK 0x02
#define SSD_STOP_BLINK 0x03

void SSDLEDSet(uint8_t idx, uint8_t behaviour);
uint8_t SSDLEDCtrl(uint8_t idx, uint8_t ctrl);
uint8_t GetAmberLEDStat(uint8_t idx);
void SSDLEDInit(void);