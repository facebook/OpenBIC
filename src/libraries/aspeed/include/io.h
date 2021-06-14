/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @brief	read 32-bit data
*/
#define readl(addr)			*(volatile uint32_t *)(addr)
#define readb(addr)			*(volatile uint8_t *)(addr)

/**
 * @brief	write 32-bit data
*/
#ifndef CONFIG_PRINT_REG_W
#define writel(value, addr)	(*(volatile uint32_t *)(addr) = value)
#else
#define writel(value, addr)                                                    \
	do {                                                                       \
		(*(volatile uint32_t *)(addr) = value);                                \
		printf("[%08x] %08x\n", addr, value);                                  \
	} while (0)
#endif

/**
 * @brief	write 32-bit data
*/
#ifndef CONFIG_PRINT_REG_W
#define writeb(value, addr)	(*(volatile uint8_t *)(addr) = (uint8_t)value)
#else
#define writeb(value, addr)                                                    \
	do {                                                                       \
		(*(volatile uint8_t *)(addr) = (uint8_t)value);                        \
		printf("[%08x] %08x\n", addr, value);                                  \
	} while (0)
#endif

/**
 * @brief	read 32-bit data from port 
*/
void readsl(void *addr, const void *data, int len);

/**
 * @brief	write 32-bit data to port 
*/
void writesl(void *addr, const void *data, int len);

#define setbits(addr, set)              writel(readl(addr) | (set), addr)
#define clrbits(addr, clear)    		writel(readl(addr) & (~(clear)), addr)
#define clrsetbits(addr, clear, set)	writel((readl(addr) & (~clear)) | (set), addr)

#define memcpy_fromio(a, b, c)		memcpy((a), (void *)(b), (c))