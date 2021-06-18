/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "common.h"
#include "irq_aspeed.h"

#define ASPEED_PERI_IRQ_DEF_PRIO	((1UL << __NVIC_PRIO_BITS) - 1UL)
static void* irq_context_tbl[256] = {0};

static void aspeed_irq_default_handler(void)
{
	while(1);
}

uint32_t aspeed_irq_get_current_irq(void)
{
	return ((SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) >> SCB_ICSR_VECTACTIVE_Pos) - NVIC_USER_IRQ_OFFSET;
}

void aspeed_irq_register(IRQn_Type irq_n, uint32_t isr, void *isr_context)
{
	NVIC_SetVector(irq_n, isr);
	irq_context_tbl[irq_n + NVIC_USER_IRQ_OFFSET] = isr_context;
	NVIC_EnableIRQ(irq_n);
}

void aspeed_irq_deregister(IRQn_Type irq_n)
{
	NVIC_DisableIRQ(irq_n);
	NVIC_SetVector(irq_n, (uint32_t)aspeed_irq_default_handler);
	irq_context_tbl[irq_n + NVIC_USER_IRQ_OFFSET] = 0;
}

void* aspeed_irq_get_isr_context(IRQn_Type irq_n)
{
	return irq_context_tbl[irq_n + NVIC_USER_IRQ_OFFSET];
}

void aspeed_irq_init(void)
{
	int i;

	for (i = 0; i < 240; i++) {
		NVIC_SetPriority(i, ASPEED_PERI_IRQ_DEF_PRIO);
	}
}