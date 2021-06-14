/*
 * Copyright (c) Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include "reset_aspeed.h"
#include "clk_aspeed.h"
#include "wait.h"
#include "log.h"
#include "hal_def.h"
#include "aspeed_crypto_reg.h"
#include "crypto_aspeed.h"

uint32_t hace_base;

void aspeed_hace_init(hace_t *hace)
{
    aspeed_device_t *device = hace->device;	

	hace_base = device->base;

	if (device->init) {
		return;
	}
	aspeed_reset_assert(device);
	aspeed_clk_enable(device);
	aspeed_wait_ms(10);
	aspeed_reset_deassert(device);
	device->init = 1;
}