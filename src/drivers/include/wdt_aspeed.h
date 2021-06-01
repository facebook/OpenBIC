/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _WDT_ASPEED_H_
#define _WDT_ASPEED_H_
#include "common.h"
#include "memory_map.h"

#define WDT_RESTART_KEY 0x4755

#define WDT_COUNTER_STATUS_REG 0x0
#define WDT_COUNTER_RELOAD_REG 0x4
#define WDT_COUNTER_RESTART_REG 0x8
#define WDT_CONTROL_REG 0xc
#define WDT_TIMOUT_STATUE_REG 0x10
#define WDT_CLEAR_TIMOUT_STATUE_REG 0x14
#define WDT_RESET_WIDTH_REG 0x18
#define WDT_RESET_MASK_REG 0x1C


static inline void wdt_init()
{
	writel(CONFIG_WDT_TIMEOUT_MS * 1000, WDT0_BASE + WDT_COUNTER_RELOAD_REG);
	writel(CONFIG_WDT_RESET_MASK, WDT0_BASE + WDT_RESET_MASK_REG);
}

static inline void wdt_reload()
{
	writel(WDT_RESTART_KEY, WDT0_BASE + WDT_COUNTER_RESTART_REG);
}

static inline void wdt_enable()
{
	writel(0x3, WDT0_BASE + WDT_CONTROL_REG);
}

#endif