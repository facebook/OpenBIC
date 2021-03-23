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
#include "cache_aspeed.h"
#include "espi_aspeed.h"
#include "aspeed_espi_reg.h"

#define OOB_DMA_UNLOCK	0x45535049

#define OOB_TX_DMA_DESC_NUM	2
#define OOB_TX_DMA_BUF_SIZE	(ESPI_PLD_LEN_MAX * OOB_TX_DMA_DESC_NUM)
#define OOB_RX_DMA_DESC_NUM	4
#define OOB_RX_DMA_BUF_SIZE	(ESPI_PLD_LEN_MAX * OOB_RX_DMA_DESC_NUM)

/* TX DMA descriptor type */
#define OOB_DMA_TX_DESC_CUST	0x04

struct oob_tx_dma_desc {
	uint32_t data_addr;
	uint8_t cyc;
	uint16_t tag : 4;
	uint16_t len : 12;
	uint8_t msg_type : 3;
	uint8_t raz0 : 1;
	uint8_t pec : 1;
	uint8_t int_en : 1;
	uint8_t pause : 1;
	uint8_t raz1 : 1;
	uint32_t raz2;
	uint32_t raz3;
} __attribute__((packed));

struct oob_rx_dma_desc {
	uint32_t data_addr;
	uint8_t cyc;
	uint16_t tag : 4;
	uint16_t len : 12;
	uint8_t raz : 7;
	uint8_t dirty : 1;
} __attribute__((packed));

static bool dma_mode = false;
static bool rx_ready = false;

static struct oob_tx_dma_desc tx_desc[OOB_TX_DMA_DESC_NUM] NON_CACHED_BSS;
static struct oob_rx_dma_desc rx_desc[OOB_RX_DMA_DESC_NUM] NON_CACHED_BSS;
static uint8_t tx_buf[OOB_TX_DMA_BUF_SIZE] NON_CACHED_BSS;
static uint8_t rx_buf[OOB_RX_DMA_BUF_SIZE] NON_CACHED_BSS;

static void aspeed_espi_oob_isr(void *arg)
{
	int i;
	uint32_t reg, sts;

	struct espi_s *espi = (struct espi_s *)arg;
	struct aspeed_espi_priv_s *priv = (struct aspeed_espi_priv_s *)espi->device->private;

	sts = ESPI_RD(ESPI_INT_STS);

	if (!(sts & (ESPI_INT_STS_HW_RST_DEASSERT | ESPI_INT_STS_OOB_BITS)))
		return;

	if (sts & ESPI_INT_STS_HW_RST_DEASSERT) {
		if (priv->oob.dma_mode) {
			for (i = 0; i < OOB_RX_DMA_DESC_NUM; ++i)
				rx_desc[i].dirty = 0;

			ESPI_WR(ESPI_OOB_TX_DMA, TO_PHY_ADDR(tx_desc));
			ESPI_WR(ESPI_OOB_TX_DMA_RB_SIZE, OOB_TX_DMA_DESC_NUM);
			ESPI_WR(ESPI_OOB_TX_DMA_RD_PTR, OOB_DMA_UNLOCK);
			ESPI_WR(ESPI_OOB_TX_DMA_WR_PTR, 0);

			ESPI_WR(ESPI_OOB_RX_DMA, TO_PHY_ADDR(rx_desc));
			ESPI_WR(ESPI_OOB_RX_DMA_RB_SIZE, OOB_RX_DMA_DESC_NUM);
			ESPI_WR(ESPI_OOB_RX_DMA_RD_PTR, OOB_DMA_UNLOCK);
			ESPI_WR(ESPI_OOB_RX_DMA_WS_PTR, 0);

			reg = ESPI_RD(ESPI_CTRL);
			reg |= (ESPI_CTRL_OOB_TX_DMA_EN
					| ESPI_CTRL_OOB_RX_DMA_EN);
			ESPI_WR(ESPI_CTRL, reg);

			ESPI_WR(ESPI_OOB_RX_DMA_WS_PTR, ESPI_OOB_RX_DMA_WS_PTR_RECV_EN);
		}

		reg = ESPI_RD(ESPI_CTRL);
		reg |= ESPI_CTRL_OOB_FW_RDY;
		ESPI_WR(ESPI_CTRL, reg);
	}

	if (sts & ESPI_INT_STS_OOB_RX_CMPLT)
		rx_ready = true;

	ESPI_WR(ESPI_INT_STS, ESPI_INT_STS_OOB_BITS);
}

static void aspeed_espi_oob_reset_isr(void *arg)
{
	uint32_t reg;

	struct espi_s *espi = (struct espi_s *)arg;
	struct aspeed_espi_priv_s *priv = (struct aspeed_espi_priv_s *)espi->device->private;

	reg = ESPI_RD(ESPI_CTRL);
	reg &= ~(ESPI_CTRL_OOB_FW_RDY | ESPI_CTRL_OOB_RX_SW_RST);
	ESPI_WR(ESPI_CTRL, reg);

	if (priv->oob.dma_mode) {
		reg = ESPI_RD(ESPI_CTRL);
		reg &= ~(ESPI_CTRL_OOB_TX_DMA_EN
				| ESPI_CTRL_OOB_RX_DMA_EN);
		ESPI_WR(ESPI_CTRL, reg);
	}
	else
		ESPI_WR(ESPI_OOB_RX_CTRL, ESPI_OOB_RX_CTRL_PEND_SERV);

	reg = ESPI_RD(ESPI_CTRL);
	reg |= ESPI_CTRL_OOB_RX_SW_RST;
	ESPI_WR(ESPI_CTRL, reg);
}

int aspeed_espi_oob_get_rx(struct aspeed_espi_xfer *xfer)
{
	int i;
	uint32_t reg;
	uint32_t cyc, tag, len;
	uint32_t wptr, sptr;
	struct espi_comm_hdr *hdr;
	struct oob_rx_dma_desc *d;

	if (!xfer)
		return -EINVAL;

	if (!rx_ready)
		return -ENODATA;

	if (dma_mode) {
		reg = ESPI_RD(ESPI_OOB_RX_DMA_WS_PTR);
		wptr = (reg & ESPI_OOB_RX_DMA_WS_PTR_WP_MASK) >> ESPI_OOB_RX_DMA_WS_PTR_WP_SHIFT;
		sptr = (reg & ESPI_OOB_RX_DMA_WS_PTR_SP_MASK) >> ESPI_OOB_RX_DMA_WS_PTR_SP_SHIFT;

		d = &rx_desc[sptr];
		if (!d->dirty)
			return -EFAULT;

		xfer->pkt_len = ((d->len) ? d->len : 0x1000) + sizeof(struct espi_comm_hdr);

		hdr = (struct espi_comm_hdr *)xfer->pkt;
		hdr->cyc = d->cyc;
		hdr->tag = d->tag;
		hdr->len_h = d->len >> 8;
		hdr->len_l = d->len & 0xff;
		memcpy(hdr + 1, rx_buf + (ESPI_PLD_LEN_MAX * sptr), xfer->pkt_len - sizeof(*hdr));

		d->dirty = 0;
		sptr = (sptr + 1) % OOB_RX_DMA_DESC_NUM;
		wptr = (wptr + 1) % OOB_RX_DMA_DESC_NUM;

		reg = ((wptr << ESPI_OOB_RX_DMA_WS_PTR_WP_SHIFT) & ESPI_OOB_RX_DMA_WS_PTR_WP_MASK)
		       | ((sptr << ESPI_OOB_RX_DMA_WS_PTR_SP_SHIFT) & ESPI_OOB_RX_DMA_WS_PTR_SP_MASK)
		       | ESPI_OOB_RX_DMA_WS_PTR_RECV_EN;

		ESPI_WR(ESPI_OOB_RX_DMA_WS_PTR, reg);

		rx_ready = rx_desc[sptr].dirty;
	}
	else {
		reg = ESPI_RD(ESPI_OOB_RX_CTRL);
		cyc = (reg & ESPI_OOB_RX_CTRL_CYC_MASK) >> ESPI_OOB_RX_CTRL_CYC_SHIFT;
		tag = (reg & ESPI_OOB_RX_CTRL_TAG_MASK) >> ESPI_OOB_RX_CTRL_TAG_SHIFT;
		len = (reg & ESPI_OOB_RX_CTRL_LEN_MASK) >> ESPI_OOB_RX_CTRL_LEN_SHIFT;

		/*
		 * calculate the length of the rest part of the
		 * eSPI packet to be read from HW and copied to
		 * user space.
		 */
		xfer->pkt_len = ((len)? len : ESPI_PLD_LEN_MAX) + sizeof(struct espi_comm_hdr);

		hdr = (struct espi_comm_hdr *)xfer->pkt;
		hdr->cyc = cyc;
		hdr->tag = tag;
		hdr->len_h = len >> 8;
		hdr->len_l = len & 0xff;

		for (i = sizeof(*hdr); i < xfer->pkt_len; ++i)
			xfer->pkt[i] = (uint8_t)(ESPI_RD(ESPI_OOB_RX_PORT) & 0xff);

		ESPI_WR(ESPI_OOB_RX_CTRL, ESPI_OOB_RX_CTRL_PEND_SERV);

		rx_ready = false;
	}

	return 0;
}

int aspeed_espi_oob_put_tx(struct aspeed_espi_xfer *xfer)
{
	int i;
	uint32_t reg;
	uint32_t cyc, tag, len;
	uint32_t rptr, wptr;
	struct espi_comm_hdr *hdr;
	struct oob_tx_dma_desc *d;

	if (!xfer)
		return -EINVAL;

	hdr = (struct espi_comm_hdr *)xfer->pkt;

	if (dma_mode) {
		/* kick HW to reflect the up-to-date read/write pointer */
		ESPI_WR(ESPI_OOB_TX_DMA_RD_PTR, ESPI_OOB_TX_DMA_RD_PTR_UPDATE);

		rptr = ESPI_RD(ESPI_OOB_TX_DMA_RD_PTR);
		wptr = ESPI_RD(ESPI_OOB_TX_DMA_WR_PTR);

		if (((wptr + 1) % OOB_TX_DMA_DESC_NUM) == rptr)
			return -EBUSY;

		d = &tx_desc[wptr];
		d->cyc = hdr->cyc;
		d->tag = hdr->tag;
		d->len = (hdr->len_h << 8) | (hdr->len_l & 0xff);
		d->msg_type = OOB_DMA_TX_DESC_CUST;

		memcpy(tx_buf + (ESPI_PLD_LEN_MAX * wptr), hdr + 1, xfer->pkt_len - sizeof(*hdr));

		wptr = (wptr + 1) % OOB_TX_DMA_DESC_NUM;
		wptr |= ESPI_OOB_TX_DMA_WR_PTR_SEND_EN;
		ESPI_WR(ESPI_OOB_TX_DMA_WR_PTR, wptr);
	}
	else {
		reg = ESPI_RD(ESPI_OOB_TX_CTRL);
		if (reg & ESPI_OOB_TX_CTRL_TRIGGER)
			return -EBUSY;

		for (i = sizeof(*hdr); i < xfer->pkt_len; ++i)
			ESPI_WR(ESPI_OOB_TX_PORT, xfer->pkt[i]);

		cyc = hdr->cyc;
		tag = hdr->tag;
		len = (hdr->len_h << 8) | (hdr->len_l & 0xff);

		reg = ((cyc << ESPI_OOB_TX_CTRL_CYC_SHIFT) & ESPI_OOB_TX_CTRL_CYC_MASK)
		       | ((tag << ESPI_OOB_TX_CTRL_TAG_SHIFT) & ESPI_OOB_TX_CTRL_TAG_MASK)
		       | ((len << ESPI_OOB_TX_CTRL_LEN_SHIFT) & ESPI_OOB_TX_CTRL_LEN_MASK)
		       | ESPI_OOB_TX_CTRL_TRIGGER;

		ESPI_WR(ESPI_OOB_TX_CTRL, reg);
	}

	return 0;
}

void aspeed_espi_oob_init(struct espi_s *espi)
{
	int i;

	uint32_t reg;
	struct aspeed_espi_priv_s *priv = (struct aspeed_espi_priv_s *)espi->device->private;

	if (priv->oob.dma_mode) {
		dma_mode = true;

		for (i = 0; i < OOB_TX_DMA_DESC_NUM; ++i)
			tx_desc[i].data_addr = TO_PHY_ADDR((uint32_t)(tx_buf + (i * ESPI_PLD_LEN_MAX)));

		for (i = 0; i < OOB_RX_DMA_DESC_NUM; ++i) {
			rx_desc[i].data_addr = TO_PHY_ADDR((rx_buf + (i * ESPI_PLD_LEN_MAX)));
			rx_desc[i].dirty = 0;
		}

		ESPI_WR(ESPI_OOB_TX_DMA, TO_PHY_ADDR(tx_desc));
		ESPI_WR(ESPI_OOB_TX_DMA_RB_SIZE, OOB_TX_DMA_DESC_NUM);
		ESPI_WR(ESPI_OOB_TX_DMA_RD_PTR, OOB_DMA_UNLOCK);
		ESPI_WR(ESPI_OOB_TX_DMA_WR_PTR, 0);

		ESPI_WR(ESPI_OOB_RX_DMA, TO_PHY_ADDR(rx_desc));
		ESPI_WR(ESPI_OOB_RX_DMA_RB_SIZE, OOB_RX_DMA_DESC_NUM);
		ESPI_WR(ESPI_OOB_RX_DMA_RD_PTR, OOB_DMA_UNLOCK);
		ESPI_WR(ESPI_OOB_RX_DMA_WS_PTR, 0);

		reg = ESPI_RD(ESPI_CTRL);
		reg |= (ESPI_CTRL_OOB_TX_DMA_EN
				| ESPI_CTRL_OOB_RX_DMA_EN);
		ESPI_WR(ESPI_CTRL, reg);

		ESPI_WR(ESPI_OOB_RX_DMA_WS_PTR, ESPI_OOB_RX_DMA_WS_PTR_RECV_EN);
	}

	espi->ch_isr[ESPI_CH_OOB].handler = aspeed_espi_oob_isr;
	espi->ch_isr[ESPI_CH_OOB].arg = espi;

	espi->ch_reset_isr[ESPI_CH_OOB].handler = aspeed_espi_oob_reset_isr;
	espi->ch_reset_isr[ESPI_CH_OOB].arg = espi;

	reg = ESPI_RD(ESPI_INT_EN)
		| ESPI_INT_EN_OOB_RX_TMOUT
		| ESPI_INT_EN_OOB_TX_ERR
		| ESPI_INT_EN_OOB_TX_ABT
		| ESPI_INT_EN_OOB_RX_ABT
		| ESPI_INT_EN_OOB_TX_CMPLT
		| ESPI_INT_EN_OOB_RX_CMPLT;
	ESPI_WR(ESPI_INT_EN, reg);


	reg = ESPI_RD(ESPI_CTRL);
	reg |= ESPI_CTRL_OOB_FW_RDY;
	ESPI_WR(ESPI_CTRL, reg);
}
