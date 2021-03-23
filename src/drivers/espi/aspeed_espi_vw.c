/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include "common.h"
#include "cmsis_os.h"
#include "util.h"
#include "hal_def.h"
#include "espi_aspeed.h"
#include "aspeed_espi_reg.h"

static void aspeed_espi_vw_isr(void *arg)
{
	uint32_t reg;
	uint32_t evt, evt_int;

	reg = ESPI_RD(ESPI_INT_STS);

	if (reg & ESPI_INT_STS_VW_GPIOEVT)
		ESPI_WR(ESPI_INT_STS, ESPI_INT_STS_VW_GPIOEVT);

	if (reg & ESPI_INT_STS_VW_SYSEVT) {
		evt_int = ESPI_RD(ESPI_SYSEVT_INT_STS);
		ESPI_WR(ESPI_SYSEVT_INT_STS, evt_int);
		ESPI_WR(ESPI_INT_STS, ESPI_INT_STS_VW_SYSEVT);
	}

	if (reg & ESPI_INT_STS_VW_SYSEVT1) {
		evt = ESPI_RD(ESPI_SYSEVT1);
		evt_int = ESPI_RD(ESPI_SYSEVT1_INT_STS);

		if (evt_int & ESPI_SYSEVT1_INT_STS_SUSPEND_WARN)
			evt |= ESPI_SYSEVT1_SUSPEND_ACK;

		ESPI_WR(ESPI_SYSEVT1, evt);
		ESPI_WR(ESPI_SYSEVT1_INT_STS, evt_int);
		ESPI_WR(ESPI_INT_STS, ESPI_INT_STS_VW_SYSEVT1);
	}
}

void aspeed_espi_vw_init(struct espi_s *espi)
{
	uint32_t reg;

	espi->ch_isr[ESPI_CH_VW].handler = aspeed_espi_vw_isr;
	espi->ch_isr[ESPI_CH_VW].arg = espi;

	espi->ch_reset_isr[ESPI_CH_VW].handler = NULL;
	espi->ch_reset_isr[ESPI_CH_VW].arg = NULL;

	reg = ESPI_RD(ESPI_INT_EN)
		| ESPI_INT_EN_VW_SYSEVT1
		| ESPI_INT_EN_VW_GPIOEVT
		| ESPI_INT_EN_VW_SYSEVT;
	ESPI_WR(ESPI_INT_EN, reg);
}
