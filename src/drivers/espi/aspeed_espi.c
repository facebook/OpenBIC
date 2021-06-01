/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include "common.h"
#include "util.h"
#include "hal_def.h"
#include "irq_aspeed.h"
#include "gpio_aspeed.h"
#include "espi_aspeed.h"
#include "aspeed_espi_reg.h"
#include "reset_aspeed.h"
#include "clk_aspeed.h"

extern gpio_t gpio[];

uint32_t espi_base;

void aspeed_espi_perif_init(struct espi_s *obj);
void aspeed_espi_vw_init(struct espi_s *obj);
void aspeed_espi_oob_init(struct espi_s *obj);
void aspeed_espi_flash_init(struct espi_s *obj);

static void aspeed_espi_ctrl_init(void)
{
	uint32_t reg;

	reg = ESPI_RD(ESPI_CTRL);
	reg |= 0xef;
	ESPI_WR(ESPI_CTRL, reg);

	ESPI_WR(ESPI_SYSEVT_INT_T0, 0x0);
	ESPI_WR(ESPI_SYSEVT_INT_T1, 0x0);

	reg = ESPI_RD(ESPI_INT_EN);
	reg |= ESPI_INT_EN_HW_RST_DEASSERT;
#if defined(CONFIG_AST1030_SERIES)
	reg |= ESPI_INT_EN_HW_RST_ASSERT;
#endif

	ESPI_WR(ESPI_INT_EN, reg);

	ESPI_WR(ESPI_SYSEVT_INT_EN, 0xffffffff);

	ESPI_WR(ESPI_SYSEVT1_INT_EN, 0x1);
	ESPI_WR(ESPI_SYSEVT1_INT_T0, 0x1);
}

#if defined(CONFIG_AST2600_SERIES)
static void aspeed_espi_gpio_reset_isr(void *arg, uint32_t gpio_num)
{
	int i;
	uint32_t reg;
	uint32_t sw_mode;

	struct espi_s *espi = (struct espi_s *)arg;

	reg = ESPI_RD(ESPI_CTRL);
	sw_mode = (reg & ESPI_CTRL_FLASH_SW_MODE_MASK);

	aspeed_espi_ctrl_init();

	reg = ESPI_RD(ESPI_CTRL);
	reg &= ~ESPI_CTRL_FLASH_SW_MODE_MASK;
	reg |= sw_mode;
	ESPI_WR(ESPI_CTRL, reg);

	for (i = 0; i < ESPI_CH_NUM; ++i)
		if (espi->ch_reset_isr[i].handler)
			espi->ch_reset_isr[i].handler(espi->ch_reset_isr[i].arg);
}
#elif defined(CONFIG_AST1030_SERIES)
static void aspeed_espi_reset_isr(void)
{
	int i;
	uint32_t reg;
	uint32_t sw_mode;

	struct espi_s *espi = (struct espi_s *)aspeed_irq_get_isr_context(Espi_IRQn);

	reg = ESPI_RD(ESPI_INT_STS);
	if (!(reg & ESPI_INT_STS_HW_RST_ASSERT))
		return;

	reg = ESPI_RD(ESPI_CTRL);
	sw_mode = (reg & ESPI_CTRL_FLASH_SW_MODE_MASK);

	aspeed_espi_ctrl_init();

	reg = ESPI_RD(ESPI_CTRL);
	reg &= ~ESPI_CTRL_FLASH_SW_MODE_MASK;
	reg |= sw_mode;
	ESPI_WR(ESPI_CTRL, reg);

	for (i = 0; i < ESPI_CH_NUM; ++i)
		if (espi->ch_reset_isr[i].handler)
			espi->ch_reset_isr[i].handler(espi->ch_reset_isr[i].arg);

	ESPI_WR(ESPI_INT_STS, ESPI_INT_STS_HW_RST_ASSERT);
}
#endif

static void aspeed_espi_isr(void)
{
	uint32_t stat;
	uint32_t sysevt;

	struct espi_s *espi = (struct espi_s *)aspeed_irq_get_isr_context(Espi_IRQn);

	stat = ESPI_RD(ESPI_INT_STS);

#if defined(CONFIG_AST1030_SERIES)
	if (stat & ESPI_INT_STS_HW_RST_ASSERT)
		return aspeed_espi_reset_isr();
#endif

	if (espi->ch_isr[ESPI_CH_PERI].handler &&
	    (stat & ESPI_INT_STS_PERIF_BITS))
		espi->ch_isr[ESPI_CH_PERI].handler(espi->ch_isr[ESPI_CH_PERI].arg);

	if (espi->ch_isr[ESPI_CH_VW].handler &&
	    (stat & ESPI_INT_STS_VW_BITS))
		espi->ch_isr[ESPI_CH_VW].handler(espi->ch_isr[ESPI_CH_VW].arg);

	if (espi->ch_isr[ESPI_CH_OOB].handler &&
	    (stat & (ESPI_INT_STS_OOB_BITS | ESPI_INT_STS_HW_RST_DEASSERT)))
		espi->ch_isr[ESPI_CH_OOB].handler(espi->ch_isr[ESPI_CH_OOB].arg);

	if (espi->ch_isr[ESPI_CH_FLASH].handler &&
	    (stat & ESPI_INT_STS_FLASH_BITS))
		espi->ch_isr[ESPI_CH_FLASH].handler(espi->ch_isr[ESPI_CH_FLASH].arg);

	if (stat & ESPI_INT_STS_HW_RST_DEASSERT) {
		sysevt = ESPI_RD(ESPI_SYSEVT);
		sysevt |= (ESPI_SYSEVT_SLV_BOOT_STS | ESPI_SYSEVT_SLV_BOOT_DONE);
		ESPI_WR(ESPI_SYSEVT, sysevt);
		ESPI_WR(ESPI_INT_STS, ESPI_INT_STS_HW_RST_DEASSERT);
	}

	/* a dummy read to make sure W1C arrives HW */
	stat = ESPI_RD(ESPI_INT_STS);
}

void aspeed_espi_init(struct espi_s *obj)
{
	struct espi_s *espi = obj;

	espi_base = obj->device->base;

	aspeed_clk_enable(espi->device);

	aspeed_espi_ctrl_init();
	aspeed_espi_perif_init(espi);
	aspeed_espi_vw_init(espi);
	aspeed_espi_oob_init(espi);
	aspeed_espi_flash_init(espi);

	aspeed_irq_register(Espi_IRQn, (uint32_t)aspeed_espi_isr, espi);

#ifdef CONFIG_AST2600_SERIES
	gpio[0].int_cb_hook(&gpio[0], 0xb8, GPIO_FALLING_EDGE,
			    aspeed_espi_gpio_reset_isr, espi);
#endif
	espi->device->init = 1;
}
