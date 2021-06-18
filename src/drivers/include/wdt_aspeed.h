/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _WDT_ASPEED_H_
#define _WDT_ASPEED_H_
#include "common.h"
#include "memory_map.h"

#define WDT_COUNTER_STATUS_REG 0x0

#define WDT_COUNTER_RELOAD_REG 0x4

#define WDT_COUNTER_RESTART_REG 0x8
#define WDT_RESTART_KEY 0x4755

#define WDT_CONTROL_REG 0xc
#define WDT_PRE_TIMEOUT_INT_MASK GENMASK(31, 10)
#define WDT_RESET_SYS_MODE GENMASK(6, 5)
#define WDT_RESET_BY_SOC_ENABLE BIT(4)
#define WDT_EXT_SIGNAL_ENABLE BIT(3)
#define WDT_PRE_TIMEOUT_INT_ENABLE BIT(2)
#define WDT_RESET_SYS_ENABLE BIT(1)
#define WDT_ENABLE BIT(0)

#define WDT_TIMOUT_STATUE_REG 0x10
#define WDT_CLEAR_TIMOUT_STATUE_REG 0x14
#define WDT_RESET_WIDTH_REG 0x18
#define WDT_RESET_MASK_REG 0x1C

static inline void wdt_set_timeout(uint32_t ms)
{
	writel(ms * 1000, WDT0_BASE + WDT_COUNTER_RELOAD_REG);
}

static inline void wdt_init()
{
	writel(CONFIG_WDT_TIMEOUT_MS * 1000, WDT0_BASE + WDT_COUNTER_RELOAD_REG);
	writel(CONFIG_WDT_RESET_MASK, WDT0_BASE + WDT_RESET_MASK_REG);
	writel(WDT_RESTART_KEY, WDT0_BASE + WDT_COUNTER_RESTART_REG);
}

static inline void wdt_reload()
{
	writel(WDT_RESTART_KEY, WDT0_BASE + WDT_COUNTER_RESTART_REG);
}

static inline void wdt_enable()
{
	writel(readl(WDT0_BASE + WDT_CONTROL_REG) | WDT_RESET_SYS_ENABLE |
				   WDT_ENABLE,
		   WDT0_BASE + WDT_CONTROL_REG);
}

#endif