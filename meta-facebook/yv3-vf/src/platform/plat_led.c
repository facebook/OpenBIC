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

#include "plat_led.h"
#include "plat_gpio.h"
#include "plat_util.h"
#include "plat_m2.h"

#include "plat_led.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(plat_led);

static uint8_t pre_state[M2_IDX_E_MAX];
static uint8_t AmberLEDStatus[M2_IDX_E_MAX];

#define SSD_LED_BLINK_INIT(DEV)                                                                    \
	void SSD_LED_blink_Handler_##DEV(struct k_timer *timer)                                    \
	{                                                                                          \
		SSDLEDSet(M2_IDX_E_##DEV, SSD_START_BLINK);                                        \
	}                                                                                          \
	void SSD_LED_stop_blink_Handler_##DEV(struct k_timer *timer)                               \
	{                                                                                          \
		SSDLEDSet(M2_IDX_E_##DEV, pre_state[M2_IDX_E_##DEV]);                              \
	}                                                                                          \
	K_TIMER_DEFINE(ssdled_blink_##DEV, SSD_LED_blink_Handler_##DEV,                            \
		       SSD_LED_stop_blink_Handler_##DEV);

SSD_LED_BLINK_INIT(A); // SSD 0
SSD_LED_BLINK_INIT(B); // SSD 1
SSD_LED_BLINK_INIT(C); // SSD 2
SSD_LED_BLINK_INIT(D); // SSD 3

uint8_t GetSSDLEDPin(uint8_t idx)
{
	const uint8_t pin = (idx == M2_IDX_E_A) ? LED_BIC_E1S_0 :
			    (idx == M2_IDX_E_B) ? LED_BIC_E1S_1 :
			    (idx == M2_IDX_E_C) ? LED_BIC_E1S_2 :
			    (idx == M2_IDX_E_D) ? LED_BIC_E1S_3 :
						  0xFF;
	return pin;
}

struct k_timer *idx_to_ssdled_timer(uint8_t idx)
{
	return (idx == M2_IDX_E_A) ? &ssdled_blink_A :
	       (idx == M2_IDX_E_B) ? &ssdled_blink_B :
	       (idx == M2_IDX_E_C) ? &ssdled_blink_C :
	       (idx == M2_IDX_E_D) ? &ssdled_blink_D :
				     NULL;
}

void SSDLEDSet(uint8_t idx, uint8_t behaviour)
{
	uint8_t pin = GetSSDLEDPin(idx);
	switch (behaviour) {
	case SSD_TURN_ON:
		gpio_set(pin, GPIO_HIGH);
		break;
	case SSD_TURN_OFF:
		gpio_set(pin, GPIO_LOW);
		break;
	case SSD_START_BLINK:
		gpio_set(pin, !gpio_get(pin));
		break;
	default:
		LOG_ERR("Error LED  %d behaviour %d!", idx, behaviour);
		break;
	}
}

void stop_blink_timer(uint8_t idx) // STOP_BLINK_CLOCK
{
	struct k_timer *p = idx_to_ssdled_timer(idx);
	k_timer_stop(p);
}

uint8_t SSDLEDCtrl(uint8_t idx, uint8_t ctrl)
{
	if (idx >= M2_IDX_E_MAX)
		return 1;
	if (ctrl > 0x03) {
		LOG_ERR("Error LED  %d control %d!", idx, ctrl);
		return 1;
	}

	// void SSD_LEDThread(void)
	AmberLEDStatus[idx] = ctrl;

	stop_blink_timer(idx);

	switch (ctrl) {
	case SSD_TURN_ON:
	case SSD_TURN_OFF:
		pre_state[idx] = ctrl;
		SSDLEDSet(idx, ctrl);
		break;
	case SSD_START_BLINK:
		k_timer_start(idx_to_ssdled_timer(idx), K_MSEC(500), K_MSEC(500));
		break;
	case SSD_STOP_BLINK:
		stop_blink_timer(idx);
		break;
	default:
		LOG_ERR("Error control for SSD %d!", idx);
	}

	return 0;
}

uint8_t GetAmberLEDStat(uint8_t idx)
{
	if (idx >= M2_IDX_E_MAX)
		return 0xFF;

	return AmberLEDStatus[idx];
}

void SSDLEDInit(void) // void SSD_LEDThread(void)
{
	uint8_t idx;
	for (idx = M2_IDX_E_A; idx < M2_IDX_E_MAX; idx++) {
		pre_state[idx] =
			gpio_get(GetSSDLEDPin(idx)); // Initial LED state -> 1: LED on 0: LED off
	}
}
