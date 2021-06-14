/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include "common.h"
#include "util.h"
#include "hal_def.h"
#include "cache_aspeed.h"
#include "uart_reg_aspeed.h"
#include "uart_aspeed.h"
#include "clk_aspeed.h"
#include "log.h"

#define DEV_ID_TO_UART_INDEX(x)	(x - ASPEED_DEV_UART0)

enum udma_buffer_size_code {
	UDMA_BUFSZ_1KB,
	UDMA_BUFSZ_4KB,
	UDMA_BUFSZ_16KB,
	UDMA_BUFSZ_64KB,

	/* supported only for VUART */
	UDMA_BUFSZ_128KB,
	UDMA_BUFSZ_256KB,
	UDMA_BUFSZ_512KB,
	UDMA_BUFSZ_1024KB,
	UDMA_BUFSZ_2048KB,
	UDMA_BUFSZ_4096KB,
	UDMA_BUFSZ_8192KB,
	UDMA_BUFSZ_16384KB,
};

#define UDMA_MAX_CHANNEL	14
#define UDMA_TX_RBSZ		0x400
#define UDMA_RX_RBSZ		0x400
static uint8_t udma_tx_rb[UDMA_MAX_CHANNEL][UDMA_TX_RBSZ] NON_CACHED_BSS_ALIGN16;
static uint8_t udma_rx_rb[UDMA_MAX_CHANNEL][UDMA_RX_RBSZ] NON_CACHED_BSS_ALIGN16;

static void cfg_udma(struct serial_s *obj)
{
	aspeed_device_t *p_uart = obj->device;
	aspeed_uart_priv_t *priv = (aspeed_uart_priv_t *)p_uart->private;
	uint32_t reg;

	priv->dma.tx_rb = udma_tx_rb[priv->dma.ch];
	priv->dma.tx_rb_addr = TO_PHY_ADDR(priv->dma.tx_rb);

	reg = readl(UDMA_BASE + UDMA_CHX_TX_CTRL(priv->dma.ch)) |
	      ((UDMA_BUFSZ_1KB << UDMA_TX_CTRL_BUFSZ_SHIFT) & UDMA_TX_CTRL_BUFSZ_MASK);
	writel(reg, UDMA_BASE + UDMA_CHX_TX_CTRL(priv->dma.ch));

	writel(priv->dma.tx_rb_addr, UDMA_BASE + UDMA_CHX_TX_BUF_BASE(priv->dma.ch));

	reg = readl(UDMA_BASE + UDMA_TX_DMA_EN) |
		  (0x1 << priv->dma.ch);
	writel(reg, UDMA_BASE + UDMA_TX_DMA_EN);

	priv->dma.rx_rb = udma_rx_rb[priv->dma.ch];
	priv->dma.rx_rb_addr = TO_PHY_ADDR(priv->dma.rx_rb);

	reg = readl(UDMA_BASE + UDMA_CHX_RX_CTRL(priv->dma.ch)) |
	      ((UDMA_BUFSZ_1KB << UDMA_RX_CTRL_BUFSZ_SHIFT) & UDMA_RX_CTRL_BUFSZ_MASK);
	writel(reg, UDMA_BASE + UDMA_CHX_RX_CTRL(priv->dma.ch));

	writel(priv->dma.rx_rb_addr, UDMA_BASE + UDMA_CHX_RX_BUF_BASE(priv->dma.ch));

	reg = readl(UDMA_BASE + UDMA_RX_DMA_EN) |
		  (0x1 << priv->dma.ch);
	writel(reg, UDMA_BASE + UDMA_RX_DMA_EN);
}

static void cfg_vuart(struct serial_s *obj)
{
	aspeed_device_t *p_uart = obj->device;
	aspeed_uart_priv_t *priv = (aspeed_uart_priv_t *)p_uart->private;
	uint32_t base = p_uart->base;
	uint32_t reg;

	writel((priv->virt.port >> 0), base + VUART_ADDRL);
	writel((priv->virt.port >> 8), base + VUART_ADDRH);

	reg = readl(base + VUART_GCRB);
	reg &= ~VUART_GCRB_HOST_SIRQ_MASK;
	reg |= ((priv->virt.sirq << VUART_GCRB_HOST_SIRQ_SHIFT) & VUART_GCRB_HOST_SIRQ_MASK);
	writel(reg, base + VUART_GCRB);

	reg = readl(base + VUART_GCRA) |
	      VUART_GCRA_DISABLE_HOST_TX_DISCARD |
	      VUART_GCRA_VUART_EN;

	if (priv->virt.sirq_pol)
		reg |= VUART_GCRA_SIRQ_POLARITY;
	else
		reg &= ~VUART_GCRA_SIRQ_POLARITY;

	writel(reg, base + VUART_GCRA);
}

static void cfg_baud(struct serial_s *obj)
{
	aspeed_device_t *p_uart = obj->device;
	uint32_t base = p_uart->base;
	uint32_t clk, dll, dlh;

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
}

static void aspeed_udma_init(void)
{
	static bool udma_init = false;
	uint32_t i, reg;

	if (udma_init)
	    return;

	writel(0x0, UDMA_BASE + UDMA_TX_DMA_EN);
	writel(0x0, UDMA_BASE + UDMA_RX_DMA_EN);

	/*
	 * For legacy design.
	 *  - TX ringbuffer size: 1KB
	 *  - RX ringbuffer size: 1KB
	 */
	reg = ((UDMA_BUFSZ_1KB << UDMA_MISC_TX_BUFSZ_SHIFT) & UDMA_MISC_TX_BUFSZ_MASK) |
	      ((UDMA_BUFSZ_1KB << UDMA_MISC_RX_BUFSZ_SHIFT) & UDMA_MISC_RX_BUFSZ_MASK);
	writel(reg, UDMA_BASE + UDMA_MISC);

	for (i = 0; i < UDMA_MAX_CHANNEL; ++i) {
	    writel(0, UDMA_BASE + UDMA_CHX_TX_WR_PTR(i));
	    writel(0, UDMA_BASE + UDMA_CHX_RX_RD_PTR(i));
	}

	writel(0xffffffff, UDMA_BASE + UDMA_TX_DMA_RST);
	writel(0x0, UDMA_BASE + UDMA_TX_DMA_RST);

	writel(0xffffffff, UDMA_BASE + UDMA_RX_DMA_RST);
	writel(0x0, UDMA_BASE + UDMA_RX_DMA_RST);

	writel(0x200, UDMA_BASE + UDMA_TIMEOUT_TIMER);

	udma_init = true;
}

/*
 *	todo: add baud calculation
 */
hal_status_t aspeed_uart_init(struct serial_s *obj)
{
	aspeed_device_t *p_uart = obj->device;
	aspeed_uart_priv_t *priv = (aspeed_uart_priv_t *)p_uart->private;

	aspeed_udma_init();

	if (NULL == &obj->cfg) {
		log_error("UART configuration data not exist\n");
		return HAL_ERROR;
	}

	if (p_uart->init) {
		log_warn("UART%d is occupied\n", DEV_ID_TO_UART_INDEX(p_uart->dev_id));
		return HAL_BUSY;
	}

	if (priv->virt_mode)
		cfg_vuart(obj);
	else
		cfg_baud(obj);

	if (priv->dma_mode)
		cfg_udma(obj);

	p_uart->init = 1;

	return HAL_OK;
}

int aspeed_uart_putc(struct serial_s *obj, uint8_t c)
{
	aspeed_device_t *p_uart = obj->device;
	aspeed_uart_priv_t *priv = (aspeed_uart_priv_t *)p_uart->private;
	uint32_t base = p_uart->base;
	volatile uint32_t status = 0;
	uint32_t rptr, wptr;

	if (priv->dma_mode) {
		while (1) {
			rptr = readl(UDMA_BASE + UDMA_CHX_TX_RD_PTR(priv->dma.ch));
			wptr = readl(UDMA_BASE + UDMA_CHX_TX_WR_PTR(priv->dma.ch));
			if (((wptr + 1) % UDMA_TX_RBSZ) == rptr) {
				osDelay(1000);
				continue;
			}

			priv->dma.tx_rb[wptr] = c;
			writel((wptr + 1) % UDMA_TX_RBSZ, UDMA_BASE + UDMA_CHX_TX_WR_PTR(priv->dma.ch));
			break;
		}
	}
	else {
		/* Wait for Ready */
		do {
			status = readl(base + UART_LSR);
		} while ((status & UART_LSR_THRE) != UART_LSR_THRE);

		/* Write Character */
		writel(c, base + UART_THR);
	}

	return c;
}

int aspeed_uart_getc(struct serial_s *obj)
{
	uint8_t c;
	aspeed_device_t *p_uart = obj->device;
	aspeed_uart_priv_t *priv = (aspeed_uart_priv_t *)p_uart->private;
	uint32_t base = p_uart->base;
	volatile uint32_t status = 0;
	uint32_t rptr, wptr;

	if (priv->dma_mode) {
		while (1) {
			rptr = readl(UDMA_BASE + UDMA_CHX_RX_RD_PTR(priv->dma.ch));
			wptr = readl(UDMA_BASE + UDMA_CHX_RX_WR_PTR(priv->dma.ch));
			if (rptr == wptr) {
				osDelay(1000);
				continue;
			}

			c = priv->dma.rx_rb[rptr];
			writel((rptr + 1) % UDMA_RX_RBSZ, UDMA_BASE + UDMA_CHX_RX_RD_PTR(priv->dma.ch));
			break;
		}
	}
	else {
		/* Wait for Ready */
		do {
			status = readl(base + UART_LSR);
		} while ((status & UART_LSR_DR) != UART_LSR_DR);

		c = readl(base + UART_RBR);
	}

	return c;
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
