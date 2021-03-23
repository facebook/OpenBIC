/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef	__UART_REG_H__
#define	__UART_REG_H__

/* DLAB = 0 */
#define UART_THR			0x00
#define UART_RBR			0x00
#define UART_IER			0x04
/* DLAB = 1 */
#define UART_DLL			0x00
#define UART_DLH			0x04

#define UART_IIR			0x08	/* for read */
#define UART_FCR			0x08	/* for write */
#define UART_LCR			0x0C
#define UART_MCR			0x10
#define UART_LSR			0x14
#define UART_MSR			0x18
#define UART_SCR			0x1C

/* bitfields for UART_LSR */
#define UART_LSR_THRE			BIT(5)
#define UART_LSR_DR			BIT(0)

#endif	/* end of "#ifndef __UART_REG_H__" */
