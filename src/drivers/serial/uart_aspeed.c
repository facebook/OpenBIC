/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include "common.h"
#include "util.h"
#include "hal_def.h"
#include "uart_reg_aspeed.h"
#include "uart_aspeed.h"
#include "clk_aspeed.h"
#include "log.h"

#define DEV_ID_TO_UART_INDEX(x)	(x - ASPEED_DEV_UART0)

/**
 * 	todo: add baud calculation
*/
hal_status_t aspeed_uart_init(struct serial_s *obj)
{
	aspeed_device_t *p_uart = obj->device;
	uint32_t base = p_uart->base;
	uint32_t clk, dll, dlh;

	if (NULL == &obj->cfg) {
		log_error("UART configuration data not exist\n");
		return HAL_ERROR;
	}

	if (p_uart->init) {
		log_warn("UART%d is occupied\n", DEV_ID_TO_UART_INDEX(p_uart->dev_id));
		return HAL_BUSY;
	}
	p_uart->init = 1;

	/* divisor latch disable */
	writel(0x3, base + UART_LCR);

	/* clear fifo */
	writel(0xc7, base + UART_FCR);

	/* divisor latch enable */
	writel(0x83, base + UART_LCR);

	clk = aspeed_clk_get_uart_clk(obj->device);
	clk = clk / (16 * obj->cfg.baud);
	dll = clk & 0xf;
	dlh = clk >> 8;

#if CONFIG_AST1030_SERIES
	/* divisor = 13, baud rate = 115200 bps */
	writel(dll, base + UART_DLL);
	writel(dlh, base + UART_DLH);
#elif CONFIG_AST2600_SERIES
	(void)dll;
	(void)dlh;
	writel(0x1, base + UART_DLL);
	writel(0x0, base + UART_DLH);
#else
	#error "Series configure erro"
#endif

	/* divisor latch disable */
	writel(0x03, base + UART_LCR);
	
	writel(0x0d, base + UART_THR);
	writel(0x0a, base + UART_THR);

	return HAL_OK;
}

int aspeed_uart_putc(struct serial_s *obj, uint8_t c)
{
	uint32_t base = obj->device->base;
	volatile uint32_t status = 0;	

	/* Wait for Ready */
	do {
		status = readl(base + UART_LSR);
	} while ((status & UART_LSR_THRE) != UART_LSR_THRE);

	/* Write Character */
	writel(c, base + UART_THR);	
	

	return c;
}

int aspeed_uart_getc(struct serial_s *obj)
{
	uint32_t base = obj->device->base;
	volatile uint32_t status = 0;	

	/* Wait for Ready */
	do {
		status = readl(base + UART_LSR);
	} while ((status & UART_LSR_DR) != UART_LSR_DR);	

	return readl(base + UART_RBR);
}

int aspeed_uart_gets(struct serial_s *obj, char *str, int n)
{
	int c, i = 0;
	int echo = 1;

	do {
		c = aspeed_uart_getc(obj);
		if (c == '\b') {
			/* backspace */
			if (--i < 0) {
				i = 0;
				continue;
			} else {
				aspeed_uart_putc(obj, c);
				continue;
			}
		} else if (c == 0x1b) {
			echo = 0;
		}
		if (echo) {
			aspeed_uart_putc(obj, c);
		}

		str[i++] = (char)c;
		if (i == n)
			return 0;
		else if ((echo == 0) && (i == 3))
			return i;
	} while (c != '\r');
	if (echo)
		aspeed_uart_putc(obj, '\n');
	return i;
}
