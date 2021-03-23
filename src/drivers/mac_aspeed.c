/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "common.h"
#include "objects.h"
#include "mac_reg_aspeed.h"
#include "reset_aspeed.h"
#include "clk_aspeed.h"
#include "mdio_aspeed.h"
#include "log.h"

/**
 * @brief MAC SW reset
*/
hal_status_t aspeed_mac_reset(mac_t *obj)
{
	uint32_t base = obj->device->base;
	mac_register_t *reg = (mac_register_t *)base;
	maccr_t cr;

	cr.value = 0;
	cr.fields.sw_rst = 1;
	reg->maccr.value = cr.value;
	while (reg->maccr.fields.sw_rst);
	
	return HAL_OK;
}

hal_status_t aspeed_mac_deinit(mac_t *obj)
{
	mac_register_t *reg = (mac_register_t *)(obj->device->base);

	reg->maccr.value = 0;
	obj->device->init = 0;

	return HAL_OK;
}
hal_status_t aspeed_mac_init(mac_t *obj)
{
	mac_register_t *reg = (mac_register_t *)(obj->device->base);
	maccr_t maccr;
	int i;

	aspeed_reset_deassert(obj->device);
	aspeed_clk_enable(obj->device);

	aspeed_mdio_init(obj->phy->mdio);
	aspeed_mac_reset(obj);

	/* set MAC address */
	reg->mac_madr = obj->mac_addr[0] << 8 | obj->mac_addr[1];
	reg->mac_ladr = obj->mac_addr[2] << 24 | obj->mac_addr[3] << 16 | obj->mac_addr[4] << 8 | obj->mac_addr[5];

	/* disable interrupt */
	reg->ier = 0;

	/* TODO: set descriptor bases */
	obj->txptr = 0;
	obj->rxptr = 0;
	reg->txr_badr = TO_PHY_ADDR((uint32_t)obj->txdes);
	reg->rxr_badr = TO_PHY_ADDR((uint32_t)obj->rxdes);
	
	for (i = 0; i < obj->n_txdes; i++) {
		obj->txdes[i].txdes3 = 0;
		obj->txdes[i].txdes0 = 0;
	}
	obj->txdes[obj->n_txdes - 1].txdes0 = MAC_TXDES0_EDOTR;

	for (i = 0; i < obj->n_rxdes; i++) {
		obj->rxdes[i].rxdes3 = TO_PHY_ADDR((uint32_t)obj->rx_pkt_buf[i]);
		obj->rxdes[i].rxdes0 = 0;
	}
	obj->rxdes[obj->n_rxdes - 1].rxdes0 = MAC_RXDES0_EDORR;

	/* set decriptor size */
	reg->dblac.fields.tdes_size = sizeof(mac_txdes_t) >> 3;
	reg->dblac.fields.rdes_size = sizeof(mac_rxdes_t) >> 3;

	/* set RX polling */
	reg->aptc.fields.rpoll_cnt = 0x1;
	
	/* default receive buffer size = 0x600 (1536) */
	reg->rbsr = 0x600;

	/* start HW */
	maccr.value = 0;
	maccr.fields.rxdma_en = 1;
	maccr.fields.rxmac_en = 1;
	maccr.fields.txdma_en = 1;
	maccr.fields.txmac_en = 1;
	maccr.fields.crc_apd = 1;
	maccr.fields.fulldup = 1;
	maccr.fields.rx_runt = 1;
	maccr.fields.rx_broadpkt_en = 1;
	if (obj->phy->speed == 1000) {
		maccr.fields.gmac_mode = 1;
		maccr.fields.speed_100 = 1;
	} else if (obj->phy->speed == 100) {
		maccr.fields.speed_100 = 1;
	}
	reg->maccr.value = maccr.value;

	/* TODO: PHY start */
	obj->device->init = 1;

	return HAL_OK;
}

hal_status_t aspeed_mac_set_loopback(mac_t *obj, uint8_t enable)
{
	mac_register_t *reg = (mac_register_t *)(obj->device->base);

	if (enable)
		reg->fear = reg->fear | BIT(30);
	else
		reg->fear = reg->fear & ~BIT(30);

	return HAL_OK;
}

hal_status_t aspeed_mac_xmit(mac_t *obj, void *packet, int length)
{
	mac_register_t *reg = (mac_register_t *)(obj->device->base);
	mac_txdes_t *curr_txdes = &obj->txdes[obj->txptr];

	curr_txdes->txdes3 = TO_PHY_ADDR((uint32_t)packet);
	curr_txdes->txdes0 |= MAC_TXDES0_FTS |
			    		  MAC_TXDES0_LTS |
			    		  MAC_TXDES0_TXBUF_SIZE(length) |
			    		  MAC_TXDES0_TXDMA_OWN ;

	reg->txpd = 1;
	obj->txptr = (obj->txptr + 1) & (obj->n_txdes - 1);
	return HAL_OK;
}

hal_status_t aspeed_mac_recv(mac_t *obj, void **packet, uint32_t *rxlen)
{
	mac_rxdes_t *curr_rxdes = &obj->rxdes[obj->rxptr];

	if (!(curr_rxdes->rxdes0 & MAC_RXDES0_RXPKT_RDY)) {
		log_debug("MAC RX timeout\n");
		return HAL_TIMEOUT;
	}

	if (curr_rxdes->rxdes0 & (MAC_RXDES0_RX_ERR |
				MAC_RXDES0_CRC_ERR |
				MAC_RXDES0_FTL |
				MAC_RXDES0_RUNT |
				MAC_RXDES0_RX_ODD_NB)) {
		log_debug("MAC RX error %08x\n", curr_rxdes->rxdes0);
		return HAL_ERROR;
	}

	*rxlen = MAC_RXDES0_VDBC(curr_rxdes->rxdes0);
	*packet = (uint8_t *)TO_VIR_ADDR(curr_rxdes->rxdes3);
	obj->rxptr = (obj->rxptr + 1) & (obj->n_rxdes - 1);

	return HAL_OK;
}

hal_status_t aspeed_mac_set_speed(mac_t *obj, uint32_t speed)
{
	maccr_t *maccr = (maccr_t *)obj->device->base;

	if (speed == 1000) {
		maccr->fields.gmac_mode = 1;
		maccr->fields.speed_100 = 1;
	} else if (speed == 100) {
		maccr->fields.gmac_mode = 0;
		maccr->fields.speed_100 = 1;
	} else {
		maccr->fields.gmac_mode = 0;
		maccr->fields.speed_100 = 0;
	}

	return HAL_OK;
}