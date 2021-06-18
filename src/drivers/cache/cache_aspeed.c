/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "common.h"
#include "hal_def.h"
#include "cmsis_os.h"
#include "memory_map.h"
#if defined (CONFIG_AST1030_SERIES) && (CONFIG_AST1030_SERIES == 1)
#include "cache_reg_ast1030.h"
#elif defined (CONFIG_AST2600_SERIES) && (CONFIG_AST2600_SERIES == 1)
#include "cache_reg_ast2600.h"
#else
#error "unsupported SOC"
#endif
#include "cache_aspeed.h"

/* cache size = 32B * 128 = 4KB */
#define CACHE_LINE_SIZE_LOG2	5
#define CACHE_LINE_SIZE			(1 << CACHE_LINE_SIZE_LOG2)
#define N_CACHE_LINE			128
#define CACHE_ALIGNED_ADDR(addr)	((addr >> CACHE_LINE_SIZE_LOG2) << CACHE_LINE_SIZE_LOG2)

/* prefetch buffer */
#define PREFETCH_BUF_SIZE		CACHE_LINE_SIZE

extern uint32_t __ram_start;
extern uint32_t __ram_end;
extern uint32_t __ram_nc_start;
extern uint32_t __ram_nc_end;
typedef struct aspeed_cache_priv_s {
	struct {
		uint32_t start;
		uint32_t end;
	} c;

	struct {
		uint32_t start;
		uint32_t end;
	} nc;

	uint32_t enable;
} aspeed_cache_priv_t;

static aspeed_cache_priv_t aspeed_cache_priv;
static NON_CACHED_BSS_ALIGN16 uint32_t aspeed_cache_nc_var;

static hal_status_t aspeed_cache_disable(void)
{
	portENTER_CRITICAL();
	__ISB();
	writel(0, SCU_BASE + CACHE_AREA_CTRL_REG);
	writel(0, SCU_BASE + CACHE_FUNC_CTRL_REG);
	portEXIT_CRITICAL();
	return HAL_OK;
}

hal_status_t aspeed_cache_init(uint32_t enable)
{
	uint32_t start_bit, end_bit, config;
	aspeed_cache_priv_t *priv = &aspeed_cache_priv;

	if (!enable) {
		priv->enable = 0;
		return aspeed_cache_disable();
	}

	priv->c.start = (uint32_t)&__ram_start;
	priv->c.end = (uint32_t)&__ram_end;
	priv->nc.start = (uint32_t)&__ram_nc_start;
	priv->nc.end = (uint32_t)&__ram_nc_end;
	priv->enable = 1;

	writel(0, SCU_BASE + CACHE_FUNC_CTRL_REG);
	start_bit = (uint32_t)&__ram_start >> CACHE_AREA_SIZE_LOG2;
	end_bit = (uint32_t)(&__ram_end - 1) >> CACHE_AREA_SIZE_LOG2;
	config = GENMASK(end_bit, start_bit);
	writel(config, SCU_BASE + CACHE_AREA_CTRL_REG);
	writel(1, SCU_BASE + CACHE_FUNC_CTRL_REG);

	return HAL_OK;
}

/**
 * @brief get aligned address and the number of cachline to be invalied
 * @param [IN] addr - start address to be invalidated
 * @param [IN] size - size in byte
 * @param [OUT] p_aligned_addr - pointer to the cacheline aligned address variable
 * @return number of cacheline to be invalidated
 * 
 *  * addr
 *   |--------size-------------|
 * |-----|-----|-----|-----|-----|
 *  \                             \
 *   head                          tail
 * 
 * example 1:
 * addr = 0x100 (cacheline aligned), size = 64
 * then head = 0x100, number of cache line to be invalidated = 64 / 32 = 2
 * which means range [0x100, 0x140) will be invalidated
 * 
 * example 2:
 * addr = 0x104 (cacheline unaligned), size = 64
 * then head = 0x100, number of cache line to be invalidated = 1 + 64 / 32 = 3
 * which means range [0x100, 0x160) will be invalidated
*/
static uint32_t get_n_cacheline(uint32_t addr, uint32_t size, uint32_t *p_head)
{
	uint32_t n = 0;
	uint32_t tail;

	/* head */
	*p_head = CACHE_ALIGNED_ADDR(addr); 

	/* roundup the tail address */
	tail = addr + size + (CACHE_LINE_SIZE - 1);
	tail = CACHE_ALIGNED_ADDR(tail);

	n = (tail - *p_head) >> CACHE_LINE_SIZE_LOG2;

	return n;
}

hal_status_t aspeed_cache_invalid_data(uint32_t addr, uint32_t size)
{
	uint32_t aligned_addr, i, n;
	aspeed_cache_priv_t *priv = &aspeed_cache_priv;

	if (((uint32_t)addr < priv->c.start) || ((uint32_t)addr >= priv->c.end))
		return HAL_ERROR;

	n = get_n_cacheline(addr, size, &aligned_addr);

	for (i = 0; i < n; i++) {
		writel(DCACHE_INVALID(aligned_addr), SCU_BASE + CACHE_INVALID_REG);
		writel(0, SCU_BASE + CACHE_INVALID_REG);
		aligned_addr += CACHE_LINE_SIZE;
	}
	__DSB();
	
	/* issue a non-cached data access to flush the prefetch buffer */
	readl((uint32_t)&aspeed_cache_nc_var);

	return HAL_OK;
}

hal_status_t aspeed_cache_invalid_instruct(uint32_t addr, uint32_t size)
{
	uint32_t aligned_addr, i, n;
	aspeed_cache_priv_t *priv = &aspeed_cache_priv;

	if (((uint32_t)addr < priv->c.start) || ((uint32_t)addr >= priv->c.end))
		return HAL_ERROR;

	n = get_n_cacheline(addr, size, &aligned_addr);

	for (i = 0; i < n; i++) {
		writel(ICACHE_INVALID(aligned_addr), SCU_BASE + CACHE_INVALID_REG);
		writel(0, SCU_BASE + CACHE_INVALID_REG);
		aligned_addr += CACHE_LINE_SIZE;
	}
	__DSB();

	return HAL_OK;
}

hal_status_t aspeed_cache_clear_data(void)
{
	uint32_t value;

	portENTER_CRITICAL();
	value = readl(SCU_BASE + CACHE_FUNC_CTRL_REG);
	value &= ~DCACHE_CLEAN;
	writel(value, SCU_BASE + CACHE_FUNC_CTRL_REG);
	__DSB();
	value |= DCACHE_CLEAN;
	writel(value, SCU_BASE + CACHE_FUNC_CTRL_REG);
	__DSB();
	portEXIT_CRITICAL();

	return HAL_OK;
}

hal_status_t aspeed_cache_clear_instruct(void)
{
	uint32_t value;

	portENTER_CRITICAL();
	value = readl(SCU_BASE + CACHE_FUNC_CTRL_REG);
	value &= ~ICACHE_CLEAN;
	writel(value, SCU_BASE + CACHE_FUNC_CTRL_REG);
	__ISB();
	value |= ICACHE_CLEAN;
	writel(value, SCU_BASE + CACHE_FUNC_CTRL_REG);
	__ISB();
	portEXIT_CRITICAL();

	return HAL_OK;
}

hal_status_t aspeed_cache_flush_prefetch(uint32_t *addr)
{
#ifdef CONFIG_AST2600_SERIES
	volatile uint32_t __attribute__((__unused__)) flush_data;
	uint32_t flush_addr;
	aspeed_cache_priv_t *priv = &aspeed_cache_priv;

	if (!addr)
		return HAL_ERROR;

	/* return error if addr is not in non-cached region */
	if (((uint32_t)addr < priv->nc.start) || ((uint32_t)addr >= priv->nc.end))
		return HAL_ERROR;

	flush_addr = (uint32_t)addr + PREFETCH_BUF_SIZE;
	if (flush_addr >= priv->nc.end)
		flush_addr = (uint32_t)addr - PREFETCH_BUF_SIZE;

	flush_data = readl(flush_addr);
#endif
	return HAL_OK;
}

hal_status_t aspeed_cache_dma_mem_read(uint32_t *addr, uint32_t n, uint32_t *out)
{
	hal_status_t ret;
	int i;

	ret = aspeed_cache_flush_prefetch(addr);
	if (HAL_OK != ret)
		return ret;

	for (i = 0; i < n; i++) {
		*out++ = readl((uint32_t)addr + (i << 2));
	}

	return HAL_OK;
}