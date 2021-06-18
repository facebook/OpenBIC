/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "serial_api.h"
#include "device_id.h"
#include "uart_aspeed.h"
#include "clk_aspeed.h"
#include "reset_aspeed.h"
void serial_init_direct(serial_t *obj, const serial_pinmap_t *pinmap)
{
	(void) pinmap;
	serial_cfg_t *p_defconfig = &obj->cfg;

	if (obj->init || obj->device->dev_id >= ASPEED_DEV_UART_NUM)
		return;

	aspeed_clk_enable(obj->device);
	aspeed_reset_deassert(obj->device);

	/* default: 115200 8N1 w/o flow control */
	p_defconfig->baud = 57600;
	p_defconfig->word_length = 8;
	p_defconfig->parity = 0;
	p_defconfig->stop_bit = 1;
	p_defconfig->flow_ctrl = 0;
	aspeed_uart_init(obj);

	obj->init = 1;
}
