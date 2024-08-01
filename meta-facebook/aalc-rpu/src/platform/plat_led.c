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

#include <stdio.h>

#include "plat_gpio.h"
#include "plat_modbus.h"
#include "plat_util.h"
#include "plat_led.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(plat_led);

static uint8_t pre_state[LED_IDX_E_MAX];
static uint8_t LEDStatus[LED_IDX_E_MAX];

#define LED_BLINK_INIT(DEV)                                                                        \
	void LED_blink_Handler_##DEV(struct k_timer *timer)                                        \
	{                                                                                          \
		led_set(LED_IDX_E_##DEV, LED_START_BLINK);                                         \
	}                                                                                          \
	void LED_stop_blink_Handler_##DEV(struct k_timer *timer)                                   \
	{                                                                                          \
		led_set(LED_IDX_E_##DEV, LED_TURN_OFF);                                            \
	}                                                                                          \
	K_TIMER_DEFINE(led_blink_##DEV, LED_blink_Handler_##DEV, LED_stop_blink_Handler_##DEV);

LED_BLINK_INIT(POWER); // LED_IDX_E_POWER
LED_BLINK_INIT(FAULT); // LED_IDX_E_FAULT
LED_BLINK_INIT(LEAK); // LED_IDX_E_LEAK
LED_BLINK_INIT(COOLANT); // LED_IDX_E_COOLANT

uint8_t get_led_pin(uint8_t idx)
{
	const uint8_t pin = (idx == LED_IDX_E_POWER)   ? FM_LED_FP_1_EN :
			    (idx == LED_IDX_E_FAULT)   ? FM_LED_FP_2_EN :
			    (idx == LED_IDX_E_LEAK)    ? FM_LED_FP_3_EN :
			    (idx == LED_IDX_E_COOLANT) ? FM_LED_FP_4_EN :
							 0xFF;
	return pin;
}

// FM_LED_FP_4_EN //coolant led
// FM_LED_FP_3_EN //leak led
// FM_LED_FP_2_EN //fault led
// FM_LED_FP_1_EN //pwr led

struct k_timer *idx_to_led_timer(uint8_t idx)
{
	return (idx == LED_IDX_E_POWER)	  ? &led_blink_POWER :
	       (idx == LED_IDX_E_FAULT)	  ? &led_blink_FAULT :
	       (idx == LED_IDX_E_LEAK)	  ? &led_blink_LEAK :
	       (idx == LED_IDX_E_COOLANT) ? &led_blink_COOLANT :
					    NULL;
}

void led_set(uint8_t idx, uint8_t behaviour)
{
	uint8_t pin = get_led_pin(idx);
	switch (behaviour) {
	case LED_TURN_ON:
		gpio_set(pin, GPIO_HIGH);
		break;
	case LED_TURN_OFF:
		gpio_set(pin, GPIO_LOW);
		break;
	case LED_START_BLINK:
		gpio_set(pin, !gpio_get(pin));
		break;
	default:
		LOG_ERR("Error LED  %d behaviour %d!", idx, behaviour);
		break;
	}
}

void stop_blink_timer(uint8_t idx) // STOP_BLINK_CLOCK
{
	struct k_timer *p = idx_to_led_timer(idx);
	k_timer_stop(p);
}

void led_ctrl(uint8_t idx, uint8_t ctrl)
{
	if (idx >= LED_IDX_E_MAX || ctrl > 0x03) {
		LOG_ERR("Error LED  %d control %d!", idx, ctrl);
		return;
	}

	// void SSD_LEDThread(void)
	LEDStatus[idx] = ctrl;

	stop_blink_timer(idx);

	switch (ctrl) {
	case LED_TURN_ON:
	case LED_TURN_OFF:
		pre_state[idx] = ctrl;
		led_set(idx, ctrl);
		break;
	case LED_START_BLINK:
		k_timer_start(idx_to_led_timer(idx), K_NO_WAIT, K_MSEC(1000));
		break;
	case LED_STOP_BLINK:
		stop_blink_timer(idx);
		break;
	default:
		LOG_ERR("Error control for LED %d!", idx);
	}

	return;
}

uint8_t get_led_status(uint8_t idx)
{
	if (idx >= LED_IDX_E_MAX)
		return 0xFF;

	return LEDStatus[idx];
}

// void SSDLEDInit(void) // void SSD_LEDThread(void)
// {
// 	uint8_t idx;
// 	for (idx = M2_IDX_E_A; idx < M2_IDX_E_MAX; idx++) {
// 		pre_state[idx] =
// 			gpio_get(get_led_pin(idx)); // Initial LED state -> 1: LED on 0: LED off
// 	}
// }
