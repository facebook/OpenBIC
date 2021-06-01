/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef	__UART_REG_H__
#define	__UART_REG_H__

/* DLAB = 0 */
#define UART_THR				0x00
#define UART_RBR				0x00
#define UART_IER				0x04
/* DLAB = 1 */
#define UART_DLL				0x00
#define UART_DLH				0x04

#define UART_IIR				0x08	/* for read */
#define UART_FCR				0x08	/* for write */
#define UART_LCR				0x0C
#define UART_MCR				0x10
#define UART_LSR				0x14
#define UART_MSR				0x18
#define UART_SCR				0x1C

/* VUART */
#define VUART_GCRA				0x20
#define VUART_GCRB				0x24
#define VUART_ADDRL				0x28
#define VUART_ADDRH				0x2C

/* UDMA */
#define UDMA_TX_DMA_EN				0x00
#define UDMA_RX_DMA_EN				0x04
#define UDMA_MISC				0x08
#define UDMA_TIMEOUT_TIMER			0x0c
#define UDMA_TX_DMA_RST				0x20
#define UDMA_RX_DMA_RST				0x24

#define UDMA_CHX_OFF(x)				((x) * 0x20)
#define UDMA_CHX_TX_RD_PTR(x)			(0x40 + UDMA_CHX_OFF(x))
#define UDMA_CHX_TX_WR_PTR(x)			(0x44 + UDMA_CHX_OFF(x))
#define UDMA_CHX_TX_BUF_BASE(x)			(0x48 + UDMA_CHX_OFF(x))
#define UDMA_CHX_TX_CTRL(x)			(0x4c + UDMA_CHX_OFF(x))
#define UDMA_CHX_RX_RD_PTR(x)			(0x50 + UDMA_CHX_OFF(x))
#define UDMA_CHX_RX_WR_PTR(x)			(0x54 + UDMA_CHX_OFF(x))
#define UDMA_CHX_RX_BUF_BASE(x)			(0x58 + UDMA_CHX_OFF(x))
#define UDMA_CHX_RX_CTRL(x)			(0x5c + UDMA_CHX_OFF(x))

/* bitfields for UART_LSR */
#define UART_LSR_THRE				BIT(5)
#define UART_LSR_DR				BIT(0)

/* bitfields for VUART_GCRA */
#define VUART_GCRA_DISABLE_HOST_TX_DISCARD	BIT(5)
#define VUART_GCRA_SIRQ_POLARITY		BIT(1)
#define VUART_GCRA_VUART_EN			BIT(0)

/* bitfields for VUART_GCRB */
#define VUART_GCRB_HOST_SIRQ_MASK		GENMASK(7, 4)
#define VUART_GCRB_HOST_SIRQ_SHIFT		4

/* bitfields for UDMA_MISC */
#define UDMA_MISC_RX_BUFSZ_MASK			GENMASK(3, 2)
#define UDMA_MISC_RX_BUFSZ_SHIFT		2
#define UDMA_MISC_TX_BUFSZ_MASK			GENMASK(1, 0)
#define UDMA_MISC_TX_BUFSZ_SHIFT		0

/* bitfields for UDMA_CHX_TX_CTRL */
#define UDMA_TX_CTRL_TMOUT_DISABLE		BIT(4)
#define UDMA_TX_CTRL_BUFSZ_MASK			GENMASK(3, 0)
#define UDMA_TX_CTRL_BUFSZ_SHIFT		0

/* bitfields for UDMA_CHX_RX_CTRL */
#define UDMA_RX_CTRL_TMOUT_DISABLE		BIT(4)
#define UDMA_RX_CTRL_BUFSZ_MASK			GENMASK(3, 0)
#define UDMA_RX_CTRL_BUFSZ_SHIFT		0

#endif	/* end of "#ifndef __UART_REG_H__" */
