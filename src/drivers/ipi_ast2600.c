/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdint.h>
#include <stdio.h>
#include "common.h"
#include "ipi_aspeed.h"
/* register offset */

/*
	CM3 writes VIC[0x18] : to trigger interrupr to CA7
	CA7 reads VIC[0x18]  : to know the pending status
	CA7 writes VIC[0x1c] : to clear the pending status

	CA7 writes VIC[0x28] : to trigger interrupr to CM3
	CM3 reads VIC[0x28]  : to know the pending status
	CM3 writes VIC[0x2c] : to clear the pending status
*/
#define IPI_TRIGGER 0x18 /* trigger CM3->CA7 inter-processor interript */

#define IPI_PENDING_STATUS 0x28 /* CA7->CM3 pending status */
#define IPI_PENDING_CLEAR 0x2c  /* clear CA7->CM3 pending */

/**
 * @brief static data for primary ISR context
*/
static ipi_t *ipi_obj;

/*
	CM3 writes VIC[0x18] : to trigger interrupt to CA7
	CA7 reads VIC[0x18]  : to know the pending status
	CA7 writes VIC[0x1c] : to clear the pending status

	CA7 writes VIC[0x28] : to trigger interrupr to CM3
	CM3 reads VIC[0x28]  : to know the pending status
	CM3 writes VIC[0x2c] : to clear the pending status
*/
static void ipi_primary_handler(void)
{
	uint32_t base = ipi_obj->device->base;
	uint32_t pending = readl(base + IPI_PENDING_STATUS);
	uint32_t i;

	printf("[IPI][ISR] pending status: %lx\n", pending);
#if 0	
	for (i = 0; i < 15; i++) {
		printf("ISPR[%d] = 0x%x\n", i, NVIC->ISPR[i]);
	}
#endif	
	writel(pending, base + IPI_PENDING_CLEAR);
	do {
		__asm volatile ("nop");
	} while(readl(base + IPI_PENDING_STATUS) != 0);

	/* TODO: add DSR to handle all IPIs */	

	i = 0;
	while (pending) {
		NVIC_ClearPendingIRQ(Ipi0_IRQn + i++);
		pending >>= 1;
	}
}

int aspeed_ipi_trigger(struct ipi_s *obj, uint32_t ipi)
{
	aspeed_ipi_priv_t *priv = obj->device->private;
	uint32_t base = obj->device->base;
	uint32_t mask;

	if (ipi > priv->n_irq)
		return -1;

	mask = BIT(ipi);	
	
	writel(readl(base + IPI_TRIGGER) | mask, base + IPI_TRIGGER);
	while((readl(base + IPI_TRIGGER) & mask) != mask);

	printf("[IPI] trigger IPI#%ld, mask: %lx\n", ipi, mask);

	return 0;
}

void aspeed_ipi_init(struct ipi_s *obj)
{
	aspeed_ipi_priv_t *priv = obj->device->private;
	uint32_t n_irq = priv->n_irq;
	uint32_t *irq = priv->irq_list;
	int i;
	
	//DEBUG_HALT();

    for (i = 0; i < n_irq; i++) {
    	NVIC_SetVector(*irq, (uint32_t)ipi_primary_handler);
    	NVIC_EnableIRQ(*irq++);
    }

	obj->device->init = 1;
	ipi_obj = obj;
}