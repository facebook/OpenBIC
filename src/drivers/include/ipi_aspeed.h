/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _IPI_ASPEED_H_
#define _IPI_ASPEED_H_
#include "objects.h"
typedef struct aspeed_ipi_priv_s {
	uint32_t n_irq;
	uint32_t irq_list[16];
} aspeed_ipi_priv_t;

int aspeed_ipi_trigger(struct ipi_s *obj, uint32_t ipi);
void aspeed_ipi_init(struct ipi_s *obj);

#endif /* end of "#ifndef _IPI_ASPEED_H_" */