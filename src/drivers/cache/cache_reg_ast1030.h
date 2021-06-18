/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
//#define CPU_BASE_ADDR_REG		0xa14
//#define CPU_IMEM_ADDR_LIM_REG	0xa18
//#define CPU_DMEM_ADDR_LIM_REG	0xa1c

/**
 * cache area control: each bit controls 32KB cache area
 * 	1: cacheable
 * 	0: no-cache
 * 
 * 	bit[0]: 1st 32KB from 0x0000_0000 to 0x0000_7fff
 *	bit[1]: 2nd 32KB from 0x0000_8000 to 0x0000_ffff
 *	...
 *	bit[22]: 23th 32KB from 0x000a_8000 to 0x000a_ffff
 *	bit[23]: 24th 32KB from 0x000b_0000 to 0x000b_ffff
 */
#define CACHE_AREA_CTRL_REG		0xa50
#define CACHE_AREA_SIZE_LOG2	15
#define CACHE_AREA_SIZE			(1 << CACHE_AREA_SIZE_LOG2)

#define CACHE_INVALID_REG		0xa54
#define DCACHE_INVALID(addr)	(BIT(31) | ((addr & GENMASK(10, 0)) << 16))
#define ICACHE_INVALID(addr)	(BIT(15) | ((addr & GENMASK(10, 0)) << 0))

#define CACHE_FUNC_CTRL_REG		0xa58
#define ICACHE_CLEAN			BIT(2)
#define DCACHE_CLEAN			BIT(1)
#define CACHE_EANABLE			BIT(0)
