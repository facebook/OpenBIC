/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _IRQ_ASPEED_H_
#define _IRQ_ASPEED_H_
#include <stdint.h>
#include "config.h"
#include CONFIG_SOC_INCLUDE_FILE

uint32_t aspeed_irq_get_current_irq(void);
void aspeed_irq_register(IRQn_Type irq_n, uint32_t isr, void *isr_context);
void aspeed_irq_deregister(IRQn_Type irq_n);
void* aspeed_irq_get_isr_context(IRQn_Type irq_n);
void aspeed_irq_init(void);

#endif /* end of "#ifndef _IRQ_ASPEED_H_" */