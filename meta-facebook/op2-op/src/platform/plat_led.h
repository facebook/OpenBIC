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

#define AMBER_LED_BLINK_INTERVAL_MS 500
#define UNKNOWN_LED_GPIO 0xFF

enum AMBER_LED_CONTROL_OPTION {
	CTRL_LED_TURN_OFF = 0x00,
	CTRL_LED_TURN_ON,
	CTRL_LED_START_BLINK,
	CTRL_LED_STOP_BLINK,
	CTRL_LED_UNKNOWN = 0xFF,
};

enum AMBER_LED_STATUS {
	LED_STATUS_OFF = 0x00,
	LED_STATUS_ON,
	LED_STATUS_BLINK,
	LED_STATUS_UNKNOWN = 0xFF,
};

uint8_t get_e1s_led_gpio(uint8_t device_id);
struct k_timer *get_e1s_led_timer(uint8_t device_id);
void stop_blink_timer(uint8_t device_id);
void set_e1s_amber_led(uint8_t device_id, uint8_t ctrl_option);
uint8_t get_e1s_amber_led_status(uint8_t device_id);
int control_e1s_amber_led(uint8_t device_id, uint8_t ctrl_option);
