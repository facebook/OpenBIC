/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <errno.h>
#include "cmsis_os.h"
#include "board_device.h"
#include "objects.h"
#include "uart_aspeed.h"
#include "log.h"

extern aspeed_device_t vuart0;

osThreadId_t vuart_tid;
osThreadAttr_t vuart_tattr;

static void vuart_task(void *arg)
{
	uint8_t c;
	struct serial_s vuart = { .device = &vuart0 };

	aspeed_uart_init(&vuart);

	while (1) {
		c = aspeed_uart_getc(&vuart);
		aspeed_uart_putc(&vuart, c);
		log_info("loopback c=%c\n", c);
		osDelay(1000);
	}
}

void demo_vuart_init(void)
{
	vuart_tattr.name = "demo_vuart";
	vuart_tattr.priority = osPriorityBelowNormal;
	vuart_tattr.stack_size = 4096;

	vuart_tid = osThreadNew(vuart_task, NULL, &vuart_tattr);
}
