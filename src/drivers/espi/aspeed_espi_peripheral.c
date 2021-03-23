/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include "common.h"
#include "cmsis_os.h"
#include "util.h"
#include "hal_def.h"
#include "espi_aspeed.h"
#include "cache_aspeed.h"
#include "aspeed_espi_reg.h"

static bool dma_mode = false;
static bool rx_ready = false;

static uint8_t pc_rx_buf[ESPI_PLD_LEN_MAX] NON_CACHED_BSS;
static uint8_t pc_tx_buf[ESPI_PLD_LEN_MAX] NON_CACHED_BSS;
static uint8_t np_tx_buf[ESPI_PLD_LEN_MAX] NON_CACHED_BSS;

static uint8_t memcyc_buf[CONFIG_DEVICE_ESPI_HOST_MAP_SIZE] __attribute__((aligned(CONFIG_DEVICE_ESPI_HOST_MAP_SIZE))) NON_CACHED_BSS;

static void aspeed_espi_perif_isr(void *arg)
{
	uint32_t sts = ESPI_RD(ESPI_INT_STS);

	if (sts & ESPI_INT_STS_PERIF_PC_RX_CMPLT)
		rx_ready = true;

	ESPI_WR(ESPI_INT_STS, sts & ESPI_INT_STS_PERIF_BITS);
}

static void aspeed_espi_perif_reset_isr(void *arg)
{
	uint32_t reg;
	struct espi_s *espi = (struct espi_s *)arg;
	struct aspeed_espi_priv_s *priv = (struct aspeed_espi_priv_s *)espi->device->private;

	ESPI_WR(ESPI_PERIF_PC_RX_SADDR, priv->perif.host_map_addr);
	ESPI_WR(ESPI_PERIF_PC_RX_TADDR, TO_PHY_ADDR(memcyc_buf));

	reg = ESPI_RD(ESPI_CTRL2);
	reg &= ~(ESPI_CTRL2_MEMCYC_RD_DIS | ESPI_CTRL2_MEMCYC_WR_DIS);
	ESPI_WR(ESPI_CTRL2, reg);

	if (dma_mode) {
		ESPI_WR(ESPI_PERIF_PC_RX_DMA, TO_PHY_ADDR(pc_rx_buf));
		ESPI_WR(ESPI_PERIF_PC_TX_DMA, TO_PHY_ADDR(pc_tx_buf));
		ESPI_WR(ESPI_PERIF_NP_TX_DMA, TO_PHY_ADDR(np_tx_buf));

		reg = ESPI_RD(ESPI_CTRL);
		reg |= (ESPI_CTRL_PERIF_NP_TX_DMA_EN
				| ESPI_CTRL_PERIF_PC_TX_DMA_EN
				| ESPI_CTRL_PERIF_PC_RX_DMA_EN);
		ESPI_WR(ESPI_CTRL, reg);
	}
}

int aspeed_espi_perif_pc_get_rx(struct aspeed_espi_xfer *xfer)
{
	int i;
	uint32_t reg;
	uint32_t cyc, tag, len;
	struct espi_comm_hdr *hdr;

	if (!xfer)
		return -EINVAL;

	if (!rx_ready)
		return -ENODATA;

	/* common header (i.e. cycle type, tag, and length) is taken by HW */
	reg = ESPI_RD(ESPI_PERIF_PC_RX_CTRL);
	cyc = (reg & ESPI_PERIF_PC_RX_CTRL_CYC_MASK) >> ESPI_PERIF_PC_RX_CTRL_CYC_SHIFT;
	tag = (reg & ESPI_PERIF_PC_RX_CTRL_TAG_MASK) >> ESPI_PERIF_PC_RX_CTRL_TAG_SHIFT;
	len = (reg & ESPI_PERIF_PC_RX_CTRL_LEN_MASK) >> ESPI_PERIF_PC_RX_CTRL_LEN_SHIFT;

	/*
	 * calculate the length of the rest part of the
	 * eSPI packet to be read from HW and copied to
	 * user space.
	 */
	switch (cyc) {
	case ESPI_PERIF_MSG:
		xfer->pkt_len = len + sizeof(struct espi_perif_msg);
		break;
	case ESPI_PERIF_MSG_D:
		xfer->pkt_len = ((len)? len : ESPI_PLD_LEN_MAX) +
			sizeof(struct espi_perif_msg);
		break;
	case ESPI_PERIF_SUC_CMPLT_D_MIDDLE:
	case ESPI_PERIF_SUC_CMPLT_D_FIRST:
	case ESPI_PERIF_SUC_CMPLT_D_LAST:
	case ESPI_PERIF_SUC_CMPLT_D_ONLY:
		xfer->pkt_len = ((len)? len : ESPI_PLD_LEN_MAX) +
			sizeof(struct espi_perif_cmplt);
		break;
	case ESPI_PERIF_SUC_CMPLT:
	case ESPI_PERIF_UNSUC_CMPLT:
		xfer->pkt_len = len + sizeof(struct espi_perif_cmplt);
		break;
	default:
		return -EFAULT;
	}

	hdr = (struct espi_comm_hdr *)xfer->pkt;
	hdr->cyc = cyc;
	hdr->tag = tag;
	hdr->len_h = len >> 8;
	hdr->len_l = len & 0xff;

	if (dma_mode)
		memcpy(hdr + 1, pc_rx_buf, xfer->pkt_len - sizeof(*hdr));
	else {
		for (i = sizeof(*hdr); i < xfer->pkt_len; ++i)
			xfer->pkt[i] = ESPI_RD(ESPI_PERIF_PC_RX_PORT) & 0xff;
	}

	ESPI_WR(ESPI_PERIF_PC_RX_CTRL, ESPI_PERIF_PC_RX_CTRL_PEND_SERV);

	rx_ready = false;

	return 0;
}

int aspeed_espi_perif_pc_put_tx(struct aspeed_espi_xfer *xfer)
{
	int i;
	uint32_t reg;
	uint32_t cyc, tag, len;
	struct espi_comm_hdr *hdr;

	if (!xfer)
		return -EINVAL;

	reg = ESPI_RD(ESPI_PERIF_PC_TX_CTRL);
	if (reg & ESPI_PERIF_PC_TX_CTRL_TRIGGER)
		return -EBUSY;

	hdr = (struct espi_comm_hdr *)xfer->pkt;

	/*
	 * common header (i.e. cycle type, tag, and length)
	 * part is written to HW registers
	 */
	if (dma_mode)
		memcpy(pc_tx_buf, hdr + 1, xfer->pkt_len - sizeof(*hdr));
	else
		for (i = sizeof(*hdr); i < xfer->pkt_len; ++i)
			ESPI_WR(ESPI_PERIF_PC_TX_PORT, xfer->pkt[i]);

	cyc = hdr->cyc;
	tag = hdr->tag;
	len = (hdr->len_h << 8) | (hdr->len_l & 0xff);

	reg = ((cyc << ESPI_PERIF_PC_TX_CTRL_CYC_SHIFT) & ESPI_PERIF_PC_TX_CTRL_CYC_MASK)
		| ((tag << ESPI_PERIF_PC_TX_CTRL_TAG_SHIFT) & ESPI_PERIF_PC_TX_CTRL_TAG_MASK)
		| ((len << ESPI_PERIF_PC_TX_CTRL_LEN_SHIFT) & ESPI_PERIF_PC_TX_CTRL_LEN_MASK)
		| ESPI_PERIF_PC_TX_CTRL_TRIGGER;

	ESPI_WR(ESPI_PERIF_PC_TX_CTRL, reg);

	return 0;
}

int aspeed_espi_perif_np_put_tx(struct aspeed_espi_xfer *xfer)
{
	int i;
	uint32_t reg;
	uint32_t cyc, tag, len;
	struct espi_comm_hdr *hdr;

	if (!xfer)
		return -EINVAL;

	reg = ESPI_RD(ESPI_PERIF_NP_TX_CTRL);

	if (reg & ESPI_PERIF_NP_TX_CTRL_TRIGGER)
		return -EBUSY;

	hdr = (struct espi_comm_hdr *)xfer->pkt;

	/*
	 * common header (i.e. cycle type, tag, and length)
	 * part is written to HW registers
	 */
	if (dma_mode)
		memcpy(pc_tx_buf, hdr + 1, xfer->pkt_len - sizeof(*hdr));
	else
		for (i = sizeof(*hdr); i < xfer->pkt_len; ++i)
			ESPI_WR(ESPI_PERIF_PC_TX_PORT, xfer->pkt[i]);

	cyc = hdr->cyc;
	tag = hdr->tag;
	len = (hdr->len_h << 8) | (hdr->len_l & 0xff);

	reg = ((cyc << ESPI_PERIF_NP_TX_CTRL_CYC_SHIFT) & ESPI_PERIF_NP_TX_CTRL_CYC_MASK)
		| ((tag << ESPI_PERIF_NP_TX_CTRL_TAG_SHIFT) & ESPI_PERIF_NP_TX_CTRL_TAG_MASK)
		| ((len << ESPI_PERIF_NP_TX_CTRL_LEN_SHIFT) & ESPI_PERIF_NP_TX_CTRL_LEN_MASK)
		| ESPI_PERIF_NP_TX_CTRL_TRIGGER;

	ESPI_WR(ESPI_PERIF_NP_TX_CTRL, reg);

	return 0;
}

void aspeed_espi_perif_init(struct espi_s *espi)
{
	uint32_t reg;
	struct aspeed_espi_priv_s *priv = (struct aspeed_espi_priv_s *)espi->device->private;

	ESPI_WR(ESPI_PERIF_PC_RX_SADDR, priv->perif.host_map_addr);
	ESPI_WR(ESPI_PERIF_PC_RX_TADDR, TO_PHY_ADDR(memcyc_buf));

	reg = SCU_RD(0xd8);
	reg &= ~(0x1);
	SCU_WR(0xd8, reg);

	reg = ESPI_RD(ESPI_CTRL2);
	reg &= ~(ESPI_CTRL2_MEMCYC_RD_DIS | ESPI_CTRL2_MEMCYC_WR_DIS);
	ESPI_WR(ESPI_CTRL2, reg);

	if (priv->perif.dma_mode) {
		dma_mode = true;

		ESPI_WR(ESPI_PERIF_PC_RX_DMA, TO_PHY_ADDR(pc_rx_buf));
		ESPI_WR(ESPI_PERIF_PC_TX_DMA, TO_PHY_ADDR(pc_tx_buf));
		ESPI_WR(ESPI_PERIF_NP_TX_DMA, TO_PHY_ADDR(np_tx_buf));

		reg = ESPI_RD(ESPI_CTRL);
		reg |= (ESPI_CTRL_PERIF_NP_TX_DMA_EN
				| ESPI_CTRL_PERIF_PC_TX_DMA_EN
				| ESPI_CTRL_PERIF_PC_RX_DMA_EN);
		ESPI_WR(ESPI_CTRL, reg);
	}

	espi->ch_isr[ESPI_CH_PERI].handler = aspeed_espi_perif_isr;
	espi->ch_isr[ESPI_CH_PERI].arg = espi;

	espi->ch_reset_isr[ESPI_CH_PERI].handler = aspeed_espi_perif_reset_isr;
	espi->ch_reset_isr[ESPI_CH_PERI].arg = espi;

	reg = ESPI_RD(ESPI_INT_EN)
		| ESPI_INT_EN_PERIF_NP_TX_ABT
		| ESPI_INT_EN_PERIF_PC_TX_ABT
		| ESPI_INT_EN_PERIF_NP_RX_ABT
		| ESPI_INT_EN_PERIF_PC_RX_ABT
		| ESPI_INT_EN_PERIF_NP_TX_ERR
		| ESPI_INT_EN_PERIF_PC_TX_ERR;
	ESPI_WR(ESPI_INT_EN, reg);
}
