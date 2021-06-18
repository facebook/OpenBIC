/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/* phy_t::phy_mode */
#define PHY_INTERFACE_MODE_RMII			0
#define PHY_INTERFACE_MODE_RGMII		1
#define PHY_INTERFACE_MODE_RGMII_ID		2
#define PHY_INTERFACE_MODE_RGMII_RXID	3
#define PHY_INTERFACE_MODE_RGMII_TXID	4

/* phy_t::loopback */
#define PHY_LOOPBACK_OFF				0	/* PHY normal mode */
#define PHY_LOOPBACK_INT				1	/* PHY internal loopback mode */
#define PHY_LOOPBACK_EXT				2	/* PHY external loopback mode */

int phy_init(phy_t *obj);
void phy_bcm54616_config(phy_t *obj);
void phy_rtl8211f_config(phy_t *obj);