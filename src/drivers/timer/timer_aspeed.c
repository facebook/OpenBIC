/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdint.h>
#include <stdio.h>
#include "common.h"
#include "device.h"
#include "objects.h"
#include "clk_aspeed.h"
#include "log.h"

#define DEV_ID_TO_TMC_INDEX(x)	(x - ASPEED_DEV_TMC0)

#define TMC_STATUS_REG			0x0
#define TMC_RELOAD_REG			0x4
#define TMC_CTRL_REG			0x30
#define TMC_CTRL_EN(tmr)		BIT(tmr * 4)
#define TMC_CTRL_MASK(tmr) 		(0xf << tmr)

#define TMC_INT_STATUS_REG		0x34
#define TMC_INT_STATUS(tmr)		BIT(tmr)

#define TMC_CTRL_CLEAR_REG		0x3c

int aspeed_timer_init(ast_timer_t *obj)
{
	uint32_t index = DEV_ID_TO_TMC_INDEX(obj->device->dev_id);
	uint32_t pclk;

	/* disable all functions by default */
	writel(TMC_CTRL_MASK(index), obj->device->base + TMC_CTRL_CLEAR_REG);

	/* get system pclk and register the tick/1us count */
	pclk = aspeed_clk_get_pclk();
	obj->tick_per_1us = pclk / 1000000;
	log_info("tick per 1us = %d\n", obj->tick_per_1us);
	if (obj->tick_per_1us == 0) {
		log_error("unable to get 1us period\n");
		return -1;
	}

	return 0;
}

int aspeed_timer_wait_us(ast_timer_t *obj, uint32_t us)
{
	uint32_t index = DEV_ID_TO_TMC_INDEX(obj->device->dev_id);
	uint32_t base = obj->device->base + (index * 0x10);
	uint32_t mask = TMC_CTRL_EN(index);
	
	writel(obj->tick_per_1us * us, base + TMC_RELOAD_REG);
	writel(TMC_CTRL_EN(index), obj->device->base + TMC_CTRL_REG);
	
	while(0 == (readl(obj->device->base + TMC_INT_STATUS_REG) & TMC_INT_STATUS(index)));
	writel(TMC_INT_STATUS(index), obj->device->base + TMC_INT_STATUS_REG);
	writel(mask, obj->device->base + TMC_CTRL_CLEAR_REG);

	return 0;
}

int aspeed_timer_wait_ms(ast_timer_t *obj, uint32_t ms)
{
	return aspeed_timer_wait_us(obj, 1000 * ms);
}