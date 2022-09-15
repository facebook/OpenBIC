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

#ifndef PLAT_SYS_H
#define PLAT_SYS_H

#include "stdint.h"
#include <stdbool.h>

enum BUTTON_ID {
	BASEBOARD_SLED_BUTTON = 0x00,
	SLOT1_SLOT_BUTTON = 0x01,
	SLOT3_SLOT_BUTTON = 0x03,
};

enum SLOT_12V_STATUS {
	SLOT_12V_OFF = 0x00,
	SLOT_12V_ON = 0x01,
};

void control_slot_12V_power(uint8_t slot_id, uint8_t control_mode);
void submit_button_event(uint8_t button_id, uint8_t target_slot, uint8_t event_type);

#endif
