/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
//#define CPU_BASE_ADDR_REG		0xa04
//#define CPU_IMEM_ADDR_LIM_REG	0xa08
//#define CPU_DMEM_ADDR_LIM_REG	0xa0c

/**
 * cache area control: each bit controls 16MB cache area
 * 	1: cacheable
 * 	0: no-cache
 * 
 * 	bit[0]: 1st 16MB from 0x0000_0000 to 0x00ff_ffff
 *	bit[1]: 2nd 16MB from 0x0100_0000 to 0x01ff_ffff
 *	...
 *	bit[30]: 31th 16MB from 0x1e00_0000 to 0x1eff_ffff
 *	bit[31]: 32th 16MB from 0x1f00_0000 to 0x1fff_ffff
 */
#define CACHE_AREA_CTRL_REG		0xa40
#define CACHE_AREA_SIZE_LOG2	24
#define CACHE_AREA_SIZE			(1 << CACHE_AREA_SIZE_LOG2)

#define CACHE_INVALID_REG		0xa44
#define DCACHE_INVALID(addr)	(BIT(31) | ((addr & GENMASK(10, 0)) << 16))
#define ICACHE_INVALID(addr)	(BIT(15) | ((addr & GENMASK(10, 0)) << 0))

#define CACHE_FUNC_CTRL_REG		0xa48
#define ICACHE_CLEAN			BIT(2)
#define DCACHE_CLEAN			BIT(1)
#define CACHE_EANABLE			BIT(0)