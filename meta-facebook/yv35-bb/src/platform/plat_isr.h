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

#ifndef PLAT_FUNC_H
#define PLAT_FUNC_H

#include <stdint.h>

#define MAX_PRESS_SLED_BTN_TIME_S 4
#define MAX_RE_ENABLE_USB_POWER_TIME_MS 250 // ms
#define MAX_PRESS_SLOT_BTN_TIME_S 4
#define MAX_BUTTON_12V_CYCLE_INTERVAL_TIME_MS 2000
#define ADD_BUTTON_SEL_DELAY_MS 500

#define SYSTEM_PRESENT 0x0
#define SYSTEM_ABSENT 0x1
#define CABLE_PRESENT 0x0
#define CABLE_ABSENT 0x1

void ISR_PWROK_SLOT1();
void ISR_PWROK_SLOT3();
void ISR_SLED_CYCLE();
void ISR_SLOT1_PRESENT();
void ISR_SLOT3_PRESENT();
void ISR_USB_POWER_LOST();
void ISR_SLOT1_BUTTON();
void ISR_SLOT3_BUTTON();
void set_BIC_slot_isolator(uint8_t pwr_state_gpio_num, uint8_t isolator_gpio_num);
void set_sled_cycle();

#endif
