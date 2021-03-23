/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <errno.h>
#include "common.h"
#include "util.h"
#include "hal_def.h"
#include "cmsis_os.h"
#include "log.h"
#include "cache_aspeed.h"
#include "irq_aspeed.h"
#include "pcc_aspeed.h"
#include "aspeed_lpc_reg.h"

enum pcc_fifo_threthold {
	PCC_FIFO_THR_1_BYTE,
	PCC_FIFO_THR_1_EIGHTH,
	PCC_FIFO_THR_2_EIGHTH,
	PCC_FIFO_THR_3_EIGHTH,
	PCC_FIFO_THR_4_EIGHTH,
	PCC_FIFO_THR_5_EIGHTH,
	PCC_FIFO_THR_6_EIGHTH,
	PCC_FIFO_THR_7_EIGHTH,
	PCC_FIFO_THR_8_EIGHTH,
};

static uint8_t pcc_dma_buf[ASPEED_PCC_DMA_SIZE] NON_CACHED_BSS __attribute__((aligned(ASPEED_PCC_DMA_SIZE)));

static void aspeed_pcc_isr_dma(pcc_t *pcc)
{
	uint32_t pre_idx, cur_idx;
	uint32_t reg = LPC_RD(PCCR2);

	if (!(reg & PCCR2_DMA_DONE))
		return;

	/* ack DMA IRQ */
	LPC_WR(PCCR2, reg);

	/* copy DMA buffer to message queue */
	reg = LPC_RD(PCCR6);

	cur_idx = reg & (pcc->dma.size - 1);
	pre_idx = pcc->dma.virt_idx;

	do {
		osMessageQueuePut(pcc->msg_q, &pcc->dma.virt[pre_idx], 0, 0);
		pre_idx = (pre_idx + 1) % pcc->dma.size;
	} while (pre_idx != cur_idx);

	pcc->dma.virt_idx = cur_idx;
}

static void aspeed_pcc_isr_fifo(pcc_t *pcc)
{
	uint32_t reg = LPC_RD(PCCR2);
	uint32_t post_code;

	if (reg & PCCR2_RX_OVR_INT) {
		log_info("RX FIFO overrun\n");
		LPC_WR(PCCR2, PCCR2_RX_OVR_INT);
	}

	if (reg & (PCCR2_RX_TMOUT_INT | PCCR2_RX_AVAIL_INT)) {
		while (reg & PCCR2_DATA_RDY) {
			post_code = LPC_RD(PCCR3) & PCCR3_FIFO_DATA_MASK;
			osMessageQueuePut(pcc->msg_q, &post_code, 0, 0);
			reg = LPC_RD(PCCR2);
		}
	}
}

static void aspeed_pcc_isr(void)
{
	pcc_t *pcc = (pcc_t *)aspeed_irq_get_isr_context(Pcc_IRQn);
	aspeed_device_t *pcc_dev = pcc->device;
	aspeed_pcc_priv_t *pcc_priv = pcc_dev->private;

	if (pcc_priv->dma_mode)
		return aspeed_pcc_isr_dma(pcc);
	else
		return aspeed_pcc_isr_fifo(pcc);
}

int aspeed_pcc_read(pcc_t *pcc, uint8_t *buf, uint32_t buf_sz)
{
	int i, count;
	aspeed_device_t *pcc_dev = pcc->device;

	if (pcc == NULL || buf == NULL)
		return -EINVAL;

	if (!pcc_dev->init)
		return -ENXIO;

	count = osMessageQueueGetCount(pcc->msg_q);
	if (count == 0)
		return -ENODATA;

	if (buf_sz < count)
		return -ENOSPC;

	for (i = 0; i < count; ++i )
		osMessageQueueGet(pcc->msg_q, buf + i, NULL, 0);

	return count;
}

void aspeed_pcc_init(pcc_t *pcc)
{
	uint32_t reg;
	aspeed_device_t *pcc_dev = pcc->device;
	aspeed_pcc_priv_t *pcc_priv = pcc_dev->private;

	if (pcc_dev->init) {
		log_error("PCC device is occupied\n");
		return;
	}

	/* always prepare larger space for the DMA case */
	pcc->msg_q = osMessageQueueNew(ASPEED_PCC_DMA_SIZE, sizeof(uint8_t), NULL);
	if (pcc->msg_q == NULL) {
		log_error("failed to allocate data queue for PCC\n");
		return;
	}

	if (pcc_priv->dma_mode) {
		pcc->dma.virt = pcc_dma_buf;
		pcc->dma.virt_idx = 0;
		pcc->dma.size = sizeof(pcc_dma_buf);
		pcc->dma.addr = TO_PHY_ADDR(pcc->dma.virt);
	}

	aspeed_irq_register(Pcc_IRQn, (uint32_t)aspeed_pcc_isr, pcc);

	/* record mode */
	reg = LPC_RD(PCCR0);
	reg &= ~PCCR0_MODE_SEL_MASK;
	reg |= ((pcc_priv->rec_mode << PCCR0_MODE_SEL_SHIFT) & PCCR0_MODE_SEL_MASK);
	LPC_WR(PCCR0, reg);

	/* port address */
	reg = LPC_RD(PCCR1);
	reg &= ~PCCR1_BASE_ADDR_MASK;
	reg |= ((pcc_priv->addr << PCCR1_BASE_ADDR_SHIFT) & PCCR1_BASE_ADDR_MASK);
	LPC_WR(PCCR1, reg);

	/* port address hight bits selection or parser control */
	reg = LPC_RD(PCCR0);
	reg &= ~PCCR0_ADDR_SEL_MASK;
	if (pcc_priv->rec_mode)
		reg |= ((pcc_priv->addr_hbit_sel << PCCR0_ADDR_SEL_SHIFT) & PCCR0_ADDR_SEL_MASK);
	else
		reg |= ((0x3 << PCCR0_ADDR_SEL_SHIFT) & PCCR0_ADDR_SEL_MASK);
	LPC_WR(PCCR0, reg);

	/* port address dont care bits */
	reg = LPC_RD(PCCR1);
	reg &= ~PCCR1_DONT_CARE_BITS_MASK;
	reg |= ((pcc_priv->addr_xbit << PCCR1_DONT_CARE_BITS_SHIFT) & PCCR1_DONT_CARE_BITS_MASK);
	LPC_WR(PCCR1, reg);

	/* DMA address and size (4-bytes unit) */
	if (pcc_priv->dma_mode) {
		LPC_WR(PCCR4, pcc->dma.addr);
		LPC_WR(PCCR5, pcc->dma.size / 4);
	}

	/* cleanup FIFO and enable PCC w/wo DMA */
	reg = LPC_RD(PCCR0);
	reg |= PCCR0_CLR_RX_FIFO;

	if (pcc_priv->dma_mode) {
		reg |= (PCCR0_EN_DMA_INT
				| PCCR0_EN_DMA_MODE);
	}
	else {
		reg &= ~PCCR0_RX_TRIG_LVL_MASK;
		reg |= ((PCC_FIFO_THR_4_EIGHTH << PCCR0_RX_TRIG_LVL_SHIFT)
				& PCCR0_RX_TRIG_LVL_MASK);

		reg |= (PCCR0_EN_RX_OVR_INT
				| PCCR0_EN_RX_TMOUT_INT
				| PCCR0_EN_RX_AVAIL_INT);
	}

	reg |= PCCR0_EN;
	LPC_WR(PCCR0, reg);

	pcc_dev->init = 1;
}
