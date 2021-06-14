/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _UART_API_H_
#define _UART_API_H_
#include "device_id.h"
#include "hal_def.h"
#include "objects.h"

typedef struct aspeed_uart_priv_s {

	bool dma_mode;
	bool virt_mode;

	struct {
		uint32_t port;
		uint32_t sirq;
		uint32_t sirq_pol;
	} virt;

	struct {
		uint32_t ch;
		uint8_t *tx_rb;
		uint32_t tx_rb_addr;
		uint8_t *rx_rb;
		uint32_t rx_rb_addr;
	} dma;
} aspeed_uart_priv_t;

hal_status_t aspeed_uart_init(struct serial_s *obj);
int aspeed_uart_putc(struct serial_s *obj, uint8_t c);
int aspeed_uart_getc(struct serial_s *obj);
int aspeed_uart_gets(struct serial_s *obj, char *str, int n);

#endif /* end of "_UART_API_H_" */
