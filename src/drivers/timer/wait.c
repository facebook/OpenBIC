/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdint.h>
#include "hal_def.h"
#include "objects.h"
#include "timer_aspeed.h"

#if CONFIG_DEVICE_TIMER
#include "board_device.h"
ast_timer_t delay_timer = {.device = &DELAY_TIMER_DEVICE};
#endif

typedef osStatus_t (*wait_t)(uint32_t);
static wait_t p_wait_func;

int aspeed_wait_ms(uint32_t ms)
{
	return p_wait_func(ms);
}

static int _aspeed_wait_ms(uint32_t ms)
{
#if CONFIG_DEVICE_TIMER
	return aspeed_timer_wait_ms(&delay_timer, ms);
#else
	uint32_t tick_per_1ms;

	tick_per_1ms = SystemCoreClock / 1000 / 8;
	for (int i = 0; i < ms; i++)
		for (int j = 0; j < tick_per_1ms; j++)
		;
	return 0;
#endif
}

void aspeed_wait_init_timer(uint8_t is_os_start)
{
	if (is_os_start) {
		p_wait_func = osDelay;
	}
	else {
#if CONFIG_DEVICE_TIMER
		aspeed_timer_init(&delay_timer);
#endif		
		p_wait_func = _aspeed_wait_ms;
	}
}

int aspeed_wait_us(uint32_t us)
{
#if CONFIG_DEVICE_TIMER
	return aspeed_timer_wait_us(&delay_timer, us);
#else
	uint32_t i, j;
	uint32_t tick_per_1us;

	tick_per_1us = SystemCoreClock / 1000000 / 8;
	for (i = 0; i < us; i++)
		for (j = 0; j < tick_per_1us; j++)
		;

	return 0;
#endif
}

