/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdint.h>
#include "hal_def.h"

#define NON_CACHED_BSS	__attribute__((section(".nocache.bss")))
#define NON_CACHED_BSS_ALIGN16	__attribute__((aligned(16), section(".nocache.bss")))
#define NON_CACHED_DATA	__attribute__((section(".nocache.data")))

/**
 * @brief init cache functions
 * @param [IN] enable - 0:disable cache, 1:enable cache
 * @return HAL_OK if successfully set
 * 
 * The cached and non-cached regions are defined by the linker script.
 * Only provide API to enable/disable cache functions
*/
hal_status_t aspeed_cache_init(uint32_t enable);

/**
 * @brief invalid data cache
 * @param [IN] addr - start address of the data to be invalidated
 * @param [IN] size - size in byte
 * @return HAL_OK if successfully set
*/
hal_status_t aspeed_cache_invalid_data(uint32_t addr, uint32_t size);

/**
 * @brief invalid instruction cache
 * @param [IN] addr - start address of the instruction to be invalidated
 * @param [IN] size - size in byte
 * @return HAL_OK if successfully set
*/
hal_status_t aspeed_cache_invalid_instruct(uint32_t addr, uint32_t size);

/**
 * @brief clear data cache
 * @return HAL_OK if successfully clear
*/
hal_status_t aspeed_cache_clear_data(void);

/**
 * @brief clear instruction cache
 * @return HAL_OK if successfully clear
*/
hal_status_t aspeed_cache_clear_instruct(void);

/**
 * @brief flush the internal prefetch buffer
 * @param [IN] addr - start address to be read
 * @return HAL_OK if successfully flush. HAL_ERROR is addr is not valid
 * 
 * This function is to fix HW bug in ast2600, and is functionless in ast1030 and 
 * newer chip series.
 * 
 * There is an invisible 32 byte prefetch buffer in the cache controller. It 
 * queues 32 byte of data even if the data address is in non-cached region. By 
 * issuing a dummy read on (addr + 32), we can ensure the data to be read 
 * is flushed out of the buffer, and will be renewed at the next load.
*/
hal_status_t aspeed_cache_flush_prefetch(uint32_t *addr);

/**
 * @brief read dma/uncached memory
 * @param [IN] addr - start address of the dma data
 * @param [IN] n - number of uint32_t to be read
 * @param [OU] out - pointer to the output buffer
 * @return HAL_OK if successfully read. HAL_ERROR is addr is not valid
*/
hal_status_t aspeed_cache_dma_mem_read(uint32_t *addr, uint32_t n, uint32_t *out);