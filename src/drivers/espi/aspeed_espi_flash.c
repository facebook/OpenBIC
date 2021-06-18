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
#include "log.h"
#include "espi_aspeed.h"
#include "cache_aspeed.h"
#include "aspeed_espi_reg.h"

static bool dma_mode = false;
static bool rx_ready = false;

static uint8_t tx_buf[ESPI_PLD_LEN_MAX] NON_CACHED_BSS;
static uint8_t rx_buf[ESPI_PLD_LEN_MAX] NON_CACHED_BSS;

static void aspeed_espi_flash_isr(void *arg)
{
	uint32_t sts = ESPI_RD(ESPI_INT_STS);

	if (sts & ESPI_INT_STS_FLASH_RX_CMPLT)
		rx_ready = true;

	ESPI_WR(ESPI_INT_STS, sts & ESPI_INT_STS_FLASH_BITS);
}

static void aspeed_espi_flash_reset_isr(void *arg)
{
	uint32_t reg;

	if (!dma_mode)
		return;

	ESPI_WR(ESPI_FLASH_TX_DMA, TO_PHY_ADDR(tx_buf));
	ESPI_WR(ESPI_FLASH_RX_DMA, TO_PHY_ADDR(rx_buf));

	reg = ESPI_RD(ESPI_CTRL);
	reg |= (ESPI_CTRL_FLASH_TX_DMA_EN | ESPI_CTRL_FLASH_RX_DMA_EN);
	ESPI_WR(ESPI_CTRL, reg);
}

int aspeed_espi_flash_get_rx(struct aspeed_espi_xfer *xfer)
{
	int i;
	uint32_t reg;
	uint32_t cyc, tag, len;
	struct espi_comm_hdr *hdr;

	if (xfer == NULL)
		return -EINVAL;

	if (!rx_ready)
		return -ENODATA;

	/*
	 * HW workaround
	 * The bit fields of the FLASH RX control register come
	 * from different HW paths. Namely, the bit fields may
	 * not be updated timely when the pending bit [31] is
	 * set.
	 * Thereby, we re-read the register to ensure all of
	 * the bit fields are updated before use.
	 */
	reg = ESPI_RD(ESPI_FLASH_RX_CTRL);
	reg = ESPI_RD(ESPI_FLASH_RX_CTRL);
	reg = ESPI_RD(ESPI_FLASH_RX_CTRL);

	/* common header (i.e. cycle type, tag, and length) is taken by HW */
	cyc = (reg & ESPI_FLASH_RX_CTRL_CYC_MASK) >> ESPI_FLASH_RX_CTRL_CYC_SHIFT;
	tag = (reg & ESPI_FLASH_RX_CTRL_TAG_MASK) >> ESPI_FLASH_RX_CTRL_TAG_SHIFT;
	len = (reg & ESPI_FLASH_RX_CTRL_LEN_MASK) >> ESPI_FLASH_RX_CTRL_LEN_SHIFT;

	/*
	 * calculate the length of the rest part of the
	 * eSPI packet to be read from HW and copied to
	 * user space.
	 */
	switch (cyc) {
	case ESPI_FLASH_READ:
	case ESPI_FLASH_WRITE:
	case ESPI_FLASH_ERASE:
		xfer->pkt_len = ((len)? len : ESPI_PLD_LEN_MAX) +
			sizeof(struct espi_flash_rwe);
		break;
	case ESPI_FLASH_SUC_CMPLT_D_MIDDLE:
	case ESPI_FLASH_SUC_CMPLT_D_FIRST:
	case ESPI_FLASH_SUC_CMPLT_D_LAST:
	case ESPI_FLASH_SUC_CMPLT_D_ONLY:
		xfer->pkt_len = ((len)? len : ESPI_PLD_LEN_MAX) +
			sizeof(struct espi_flash_cmplt);
		break;
	case ESPI_FLASH_SUC_CMPLT:
	case ESPI_FLASH_UNSUC_CMPLT:
		xfer->pkt_len = len + sizeof(struct espi_flash_cmplt);
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
		memcpy(hdr + 1, rx_buf, xfer->pkt_len - sizeof(*hdr));
	else
		for (i = sizeof(*hdr); i < xfer->pkt_len; ++i)
			xfer->pkt[i] = (uint8_t)(ESPI_RD(ESPI_FLASH_RX_PORT) & 0xff);

	ESPI_WR(ESPI_FLASH_RX_CTRL, ESPI_FLASH_RX_CTRL_PEND_SERV);

	rx_ready = false;

	return 0;
}

int aspeed_espi_flash_put_tx(struct aspeed_espi_xfer *xfer)
{
	int i;
	uint32_t reg;
	uint32_t cyc, tag, len;
	struct espi_comm_hdr *hdr;

	if (!xfer)
		return -EINVAL;

	reg = ESPI_RD(ESPI_FLASH_TX_CTRL);
	if (reg & ESPI_FLASH_TX_CTRL_TRIGGER)
		return -EBUSY;

	hdr = (struct espi_comm_hdr *)xfer->pkt;

	if (dma_mode)
		memcpy(tx_buf, hdr + 1, xfer->pkt_len - sizeof(*hdr));
	else
		for (i = sizeof(*hdr); i < xfer->pkt_len; ++i)
			ESPI_WR(ESPI_FLASH_TX_PORT, xfer->pkt[i]);

	cyc = hdr->cyc;
	tag = hdr->tag;
	len = (hdr->len_h << 8) | (hdr->len_l & 0xff);

	reg = ((cyc << ESPI_FLASH_TX_CTRL_CYC_SHIFT) & ESPI_FLASH_TX_CTRL_CYC_MASK)
		| ((tag << ESPI_FLASH_TX_CTRL_TAG_SHIFT) & ESPI_FLASH_TX_CTRL_TAG_MASK)
		| ((len << ESPI_FLASH_TX_CTRL_LEN_SHIFT) & ESPI_FLASH_TX_CTRL_LEN_MASK)
		| ESPI_FLASH_TX_CTRL_TRIGGER;

	ESPI_WR(ESPI_FLASH_TX_CTRL, reg);

	return 0;
}

void aspeed_espi_flash_init(struct espi_s *espi)
{
	uint32_t reg;
	struct aspeed_espi_priv_s *priv = (struct aspeed_espi_priv_s *)espi->device->private;

	reg = ESPI_RD(ESPI_CTRL);

	reg &= ~(ESPI_CTRL_FLASH_SW_MODE_MASK);
	reg |= ((priv->flash.safs_mode << ESPI_CTRL_FLASH_SW_MODE_SHIFT) & ESPI_CTRL_FLASH_SW_MODE_MASK);

	if (priv->flash.dma_mode) {
		dma_mode = true;

		ESPI_WR(ESPI_FLASH_TX_DMA, TO_PHY_ADDR(tx_buf));
		ESPI_WR(ESPI_FLASH_RX_DMA, TO_PHY_ADDR(rx_buf));
		reg |= (ESPI_CTRL_FLASH_TX_DMA_EN | ESPI_CTRL_FLASH_RX_DMA_EN);
	}

	ESPI_WR(ESPI_CTRL, reg);

	espi->ch_isr[ESPI_CH_FLASH].handler = aspeed_espi_flash_isr;
	espi->ch_isr[ESPI_CH_FLASH].arg = espi;

	espi->ch_reset_isr[ESPI_CH_FLASH].handler = aspeed_espi_flash_reset_isr;
	espi->ch_reset_isr[ESPI_CH_FLASH].arg = espi;

	reg = ESPI_RD(ESPI_INT_EN)
		| ESPI_INT_EN_FLASH_TX_ERR
		| ESPI_INT_EN_FLASH_TX_ABT
		| ESPI_INT_EN_FLASH_RX_ABT
		| ESPI_INT_EN_FLASH_TX_CMPLT
		| ESPI_INT_EN_FLASH_RX_CMPLT;
	ESPI_WR(ESPI_INT_EN, reg);

}
