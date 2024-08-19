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

#define LED_TURN_OFF 0x00
#define LED_TURN_ON 0x01
#define LED_START_BLINK 0x02
#define LED_STOP_BLINK 0x03

enum LED_IDX_E {
	LED_IDX_E_POWER = 0,
	LED_IDX_E_FAULT,
	LED_IDX_E_LEAK,
	LED_IDX_E_COOLANT,
	LED_IDX_E_MAX,
};

// FM_LED_FP_4_EN //coolant led
// FM_LED_FP_3_EN //leak led
// FM_LED_FP_2_EN //fault led
// FM_LED_FP_1_EN //pwr led

enum LED_FAULT_THRESHOLD_E {
	LED_FAULT_PUMP_1 = 0,
	LED_FAULT_PUMP_2 = 1,
	LED_FAULT_PUMP_3 = 2,
	LED_FAULT_PB_1_FAN_1 = 3,
	LED_FAULT_PB_1_FAN_2 = 4,
	LED_FAULT_PB_2_FAN_1 = 5,
	LED_FAULT_PB_2_FAN_2 = 6,
	LED_FAULT_PB_3_FAN_1 = 7,
	LED_FAULT_PB_3_FAN_2 = 8,
	LED_FAULT_HEX_FAN_1 = 9,
	LED_FAULT_HEX_FAN_2 = 10,
	LED_FAULT_HEX_FAN_3 = 11,
	LED_FAULT_HEX_FAN_4 = 12,
	LED_FAULT_HEX_FAN_5 = 13,
	LED_FAULT_HEX_FAN_6 = 14,
	LED_FAULT_HEX_FAN_7 = 15,
	LED_FAULT_HEX_FAN_8 = 16,
	LED_FAULT_HEX_FAN_9 = 17,
	LED_FAULT_HEX_FAN_10 = 18,
	LED_FAULT_HEX_FAN_11 = 19,
	LED_FAULT_HEX_FAN_12 = 20,
	LED_FAULT_HEX_FAN_13 = 21,
	LED_FAULT_HEX_FAN_14 = 22,
	LED_FAULT_HIGH_PRESS = 23,
	LED_FAULT_LOW_LEVEL = 24,
	LED_FAULT_HIGH_AIR_TEMP = 25,
	LED_FAULT_HIGH_COOLANT_INLET_TEMP = 26,
	LED_FAULT_HIGH_COOLANT_OUTLET_TEMP = 27,
	LED_FAULT_FLOW_TRIGGER = 28,
	LED_FAULT_THRESHOLD_E_MAX,
};

void led_set(uint8_t idx, uint8_t behaviour);
void led_ctrl(uint8_t idx, uint8_t ctrl);
uint8_t get_led_pin(uint8_t idx);
uint8_t get_led_status(uint8_t idx);
//void SSDLEDInit(void);
bool fault_led_control(void);