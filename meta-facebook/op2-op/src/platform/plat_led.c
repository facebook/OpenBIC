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

#include <logging/log.h>
#include "libutil.h"
#include "plat_gpio.h"
#include "plat_class.h"
#include "plat_power_seq.h"
#include "plat_led.h"

LOG_MODULE_REGISTER(plat_led);

static uint8_t e1s_amber_led_pre_status[MAX_E1S_IDX] = { LED_STATUS_OFF, LED_STATUS_OFF,
							 LED_STATUS_OFF, LED_STATUS_OFF,
							 LED_STATUS_OFF };
static uint8_t e1s_amber_led_status[MAX_E1S_IDX] = { LED_STATUS_OFF, LED_STATUS_OFF, LED_STATUS_OFF,
						     LED_STATUS_OFF, LED_STATUS_OFF };
static uint8_t els_amber_led_gpio_opa[MAX_E1S_IDX] = { OPA_LED_E1S_0_ATTN_R, OPA_LED_E1S_1_ATTN_R,
						       OPA_LED_E1S_2_ATTN_R };
static uint8_t els_amber_led_gpio_opb[MAX_E1S_IDX] = { OPB_LED_E1S_0_ATTN_R, OPB_LED_E1S_1_ATTN_R,
						       OPB_LED_E1S_2_ATTN_R, OPB_LED_E1S_3_ATTN_R,
						       OPB_LED_E1S_4_ATTN_R };

#define E1S_LED_BLINK_INIT(device)                                                                 \
	void blink_handler_##device(struct k_timer *timer)                                         \
	{                                                                                          \
		CHECK_NULL_ARG(timer);                                                             \
		set_e1s_amber_led(E1S_##device, CTRL_LED_START_BLINK);                             \
	}                                                                                          \
	void stop_blink_handler_##device(struct k_timer *timer)                                    \
	{                                                                                          \
		CHECK_NULL_ARG(timer);                                                             \
		set_e1s_amber_led(E1S_##device, e1s_amber_led_pre_status[E1S_##device]);           \
	}                                                                                          \
	K_TIMER_DEFINE(e1s_led_blink_timer_##device, blink_handler_##device,                       \
		       stop_blink_handler_##device);

E1S_LED_BLINK_INIT(0);
E1S_LED_BLINK_INIT(1);
E1S_LED_BLINK_INIT(2);
E1S_LED_BLINK_INIT(3);
E1S_LED_BLINK_INIT(4);

uint8_t get_e1s_led_gpio(uint8_t device_id)
{
	if (get_card_type() == CARD_TYPE_OPA) {
		return els_amber_led_gpio_opa[device_id];
	} else {
		return els_amber_led_gpio_opb[device_id];
	}
}

struct k_timer *get_e1s_led_timer(uint8_t device_id)
{
	switch (device_id) {
	case E1S_0:
		return &e1s_led_blink_timer_0;
	case E1S_1:
		return &e1s_led_blink_timer_1;
	case E1S_2:
		return &e1s_led_blink_timer_2;
	case E1S_3:
		return &e1s_led_blink_timer_3;
	case E1S_4:
		return &e1s_led_blink_timer_4;
	default:
		return NULL;
	}
}

void stop_blink_timer(uint8_t device_id)
{
	struct k_timer *timer = get_e1s_led_timer(device_id);

	CHECK_NULL_ARG(timer);

	k_timer_stop(timer);
}

void set_e1s_amber_led(uint8_t device_id, uint8_t ctrl_option)
{
	uint8_t led_gpio = get_e1s_led_gpio(device_id);
	if (led_gpio == UNKNOWN_LED_GPIO) {
		return;
	}

	switch (ctrl_option) {
	case CTRL_LED_TURN_OFF:
		gpio_set(led_gpio, GPIO_LOW);
		break;
	case CTRL_LED_TURN_ON:
		gpio_set(led_gpio, GPIO_HIGH);
		break;
	case CTRL_LED_START_BLINK:
		gpio_set(led_gpio, !gpio_get(led_gpio));
		break;
	default:
		LOG_ERR("Failed to control LED%d due to unknown control command %d", device_id,
			ctrl_option);
		break;
	}

	return;
}

uint8_t get_e1s_amber_led_status(uint8_t device_id)
{
	if (device_id >= MAX_E1S_IDX) {
		return LED_STATUS_UNKNOWN;
	}

	return e1s_amber_led_status[device_id];
}

int control_e1s_amber_led(uint8_t device_id, uint8_t ctrl_option)
{
	uint8_t card_type = get_card_type();
	uint8_t max_device_num = 0;
	int ret = 0;

	if (card_type == CARD_TYPE_OPA) {
		max_device_num = OPA_MAX_E1S_IDX;
	} else {
		max_device_num = MAX_E1S_IDX;
	}

	if (device_id >= max_device_num) {
		return -1;
	}

	stop_blink_timer(device_id);

	switch (ctrl_option) {
	case CTRL_LED_TURN_OFF:
	case CTRL_LED_TURN_ON:
		e1s_amber_led_pre_status[device_id] = ctrl_option;
		e1s_amber_led_status[device_id] = ctrl_option;
		set_e1s_amber_led(device_id, ctrl_option);
		break;
	case CTRL_LED_START_BLINK:
		e1s_amber_led_status[device_id] = ctrl_option;
		k_timer_start(get_e1s_led_timer(device_id), K_MSEC(AMBER_LED_BLINK_INTERVAL_MS),
			      K_MSEC(AMBER_LED_BLINK_INTERVAL_MS));
		break;
	case CTRL_LED_STOP_BLINK:
		e1s_amber_led_status[device_id] = e1s_amber_led_pre_status[device_id];
		stop_blink_timer(device_id);
		break;
	default:
		LOG_ERR("Failed to control LED%d due to unknown control command %d", device_id,
			ctrl_option);
		ret = -1;
		break;
	}

	return ret;
}
