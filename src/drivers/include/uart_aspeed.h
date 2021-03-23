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

hal_status_t aspeed_uart_init(struct serial_s *obj);
int aspeed_uart_putc(struct serial_s *obj, uint8_t c);
int aspeed_uart_getc(struct serial_s *obj);
int aspeed_uart_gets(struct serial_s *obj, char *str, int n);

#endif /* end of "_UART_API_H_" */