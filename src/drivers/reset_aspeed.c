/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdint.h>
#include <stdio.h>
#include "common.h"
#include "device_id.h"
#include "device.h"

int aspeed_reset_assert(aspeed_device_t *device)
{
	struct aspeed_reset_s *reset = device->reset;

	if (reset->reg_assert)
		writel(reset->bits, reset->reg_assert);
	
	return 0;
}

int aspeed_reset_deassert(aspeed_device_t *device)
{
	struct aspeed_reset_s *reset = device->reset;

	if (reset->reg_deassert)
		writel(reset->bits, reset->reg_deassert);
	
	return 0;
}