/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _MAC_ASPEED_H_
#define _MAC_ASPEED_H_
#include "hal_def.h"
#include "objects.h"

typedef struct cfg_rgmii_s {
	uint32_t reg_set;
	uint32_t reg_clr;
	uint32_t bits;
} cfg_rgmii_t;
typedef struct aspeed_mac_priv_s {
	cfg_rgmii_t cfg_rgmii;
} aspeed_mac_priv_t;

hal_status_t aspeed_mac_reset(mac_t *obj);
hal_status_t aspeed_mac_set_loopback(mac_t *obj, uint8_t enable);
hal_status_t aspeed_mac_init(mac_t *obj);
hal_status_t aspeed_mac_deinit(mac_t *obj);
/**
 * @brief	add a packet into the TX queue
 * @param	[IN] obj: MAC object
 * @param	[IN] packet: pointer to the ethernet packet, FCS is not included
 * @param	[IN] length: length of the ethernet packet
 * @return	HAL_OK
*/
hal_status_t aspeed_mac_txpkt_add(mac_t *obj, void *packet, int length);

/**
 * @brief	push the packets in the TX queue to the MII
 * @param	[IN] obj: MAC object
 * @return	HAL_OK
*/
hal_status_t aspeed_mac_xmit(mac_t *obj);

/**
 * @brief	get a packet from the RX queue
 * @param	[IN] obj: MAC object
 * @param	[IN] packet: pointer to the ethernet packet
 * @param	[IN] rxlen: length of the ethernet packet
 * @return	HAL_OK
*/
hal_status_t aspeed_mac_recv(mac_t *obj, void **packet, uint32_t *rxlen);
hal_status_t aspeed_mac_init_rx_desc(mac_t *obj);
hal_status_t aspeed_mac_init_tx_desc(mac_t *obj);
hal_status_t aspeed_mac_set_speed(mac_t *obj, uint32_t speed);
#endif /* end of "#ifndef _MAC_ASPEED_H_" */