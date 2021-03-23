/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "common.h"

void writesl(void *addr, const void *data, int len)
{
	const uint32_t *src = (uint32_t *)data;
	volatile uint32_t *dst = (uint32_t *)addr;

	if (len == 0)
		return;

	if ((uint32_t)data & 0x3)
		DEBUG_HALT();

	while (len-- > 0)
		*dst = *src++;
}

void readsl(void *addr, const void *data, int len)
{
	const volatile uint32_t *src = (uint32_t *)addr;
	uint32_t *dst = (uint32_t *)data;

	if ((uint32_t)data & 0x3)
		DEBUG_HALT();

	while (len-- > 0)
		*dst++ = *src;
}