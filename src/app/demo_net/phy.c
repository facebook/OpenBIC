/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include "common.h"
#include "mdio_aspeed.h"

#define PHY_BMCR		0x00
#define PHY_BMSR		0x01
#define PHY_ID_1		0x02
#define PHY_ID_2		0x03
#define PHY_ANER        0x06	/* Auto-negotiation Expansion Register */
#define PHY_GBCR        0x09	/* 1000Base-T Control Register */
#define PHY_INER        0x12	/* Interrupt Enable Register */
#define PHY_CR1			0x18
#define PHY_SR          0x1a	/* PHY Specific Status Register */
#define PHY_EPAGSR		0x1f	/* realtek extension page selection */

#define BMCR_RESET		BIT(15)
#define BMCR_LOOPBACK	BIT(14)
#define BMCR_SPEED_0	BIT(13)	/* 1=100M, 0=10M */
#define BMCR_ANE		BIT(12)
#define BMCR_PWD		BIT(11)
#define BMCR_ISO		BIT(10)
#define BMCR_RESTART_ANE	BIT(9)
#define BMCR_DUPLEX		BIT(8)
#define BMCR_COLLI_TEST	BIT(7)
#define BMCR_SPEED_1	BIT(6)

#define BMCR_SPEED_1000 BMCR_SPEED_1
#define BMCR_SPEED_100	BMCR_SPEED_0
#define BMCR_SPEED_10	0

#define BMCR_BASIC_MASK	(BMCR_SPEED_1 | BMCR_DUPLEX | BMCR_ANE | BMCR_SPEED_0 | BMCR_LOOPBACK)

#define SR_SPEED		(0x3 << 4)
#define SR_SPEED_1000	(0x2 << 4)
#define SR_SPEED_100	(0x1 << 4)
#define SR_SPEED_10		(0x0 << 4)
#define SR_DUPLEX		BIT(3)
#define SR_LINK			BIT(2)

#if 0
/**
 * @brief	scan PHY ID on all possible PHY addresses (from 0 to 31)
*/
static void phy_scan(mdio_t *obj)
{
	int i;

	for (i = 0; i < 32; i++) {
		printf("[%02d] %04x %04x ", aspeed_mdio_read(obj, i, PHY_ID_1), aspeed_mdio_read(obj, i, PHY_ID_2));
		if ((i % 4) == 3)
			printf("\n");
	}
}
#endif

static void phy_basic_setting(mdio_t *obj)
{
	uint32_t value;

	value = aspeed_mdio_read(obj, obj->phy_addr, PHY_BMCR);
	value &= ~BMCR_BASIC_MASK;
	aspeed_mdio_write(obj, obj->phy_addr, PHY_BMCR, value);
}

static void phy_reset(mdio_t *obj)
{
	aspeed_mdio_write(obj, obj->phy_addr, PHY_BMCR, BMCR_RESET);
	while (aspeed_mdio_read(obj, obj->phy_addr, PHY_BMCR) & BMCR_RESET);

	phy_basic_setting(obj);
}

void phy_rtl8211f_config(phy_t *obj)
{
	mdio_t *mdio = obj->mdio;
	uint32_t mask;

	aspeed_mdio_write(mdio, mdio->phy_addr, PHY_EPAGSR, 0xa43);
	phy_reset(mdio);
	aspeed_mdio_write(mdio, mdio->phy_addr, PHY_BMCR, BMCR_ANE | BMCR_DUPLEX);
	
	mask = SR_SPEED | SR_DUPLEX | SR_LINK;
	while (aspeed_mdio_read(mdio, mdio->phy_addr, PHY_SR) & mask)
	aspeed_mdio_write(mdio, mdio->phy_addr, PHY_EPAGSR, 0);

}

void phy_bcm54616_config(phy_t *obj)
{
	mdio_t *mdio = obj->mdio;
	uint32_t bmcr, reg18, reg1c;

	phy_reset(mdio);

	bmcr = aspeed_mdio_read(mdio, mdio->phy_addr, PHY_BMCR);
	bmcr &= ~(BMCR_ISO | BMCR_PWD | BMCR_RESET);
	aspeed_mdio_write(mdio, mdio->phy_addr, PHY_BMCR, bmcr);
	
	//gbcr = aspeed_mdio_read(mdio, mdio->phy_addr, PHY_GBCR);

	/* reg 0x18, shadow value b'0111: misc control */
	aspeed_mdio_write(mdio, mdio->phy_addr, 0x18, 0x7007);
	reg18 = aspeed_mdio_read(mdio, mdio->phy_addr, 0x18);
	if (reg18 & BIT(8)) {
		/* RGMII RXD to RXC skew is enabled */
		reg18 = (reg18 & 0x0af0) | 0xf007;
		aspeed_mdio_write(mdio, mdio->phy_addr, 0x18, reg18);
	}
	
	/* reg 0x1c, shadow value b'0011: clock alignment control */
	aspeed_mdio_write(mdio, mdio->phy_addr, 0x1c, 0x0c00);
	reg1c = aspeed_mdio_read(mdio, mdio->phy_addr, 0x1c);
	if (reg1c & BIT(9)) {
		/* GTX clock delay is enabled */
		reg1c = BIT(15) | 0x0c00;
		aspeed_mdio_write(mdio, mdio->phy_addr, 0x1c, reg1c);
	}

	bmcr = (obj->loopback) ? BMCR_LOOPBACK : 0;
	if (obj->speed == 1000) {
		aspeed_mdio_write(mdio, mdio->phy_addr, PHY_GBCR, 0x1800);
		aspeed_mdio_write(mdio, mdio->phy_addr, PHY_BMCR, bmcr | BMCR_SPEED_1000);
	} else if (obj->speed == 100) {
		aspeed_mdio_write(mdio, mdio->phy_addr, PHY_BMCR, bmcr | BMCR_SPEED_100 | BMCR_DUPLEX);
	} else {
		aspeed_mdio_write(mdio, mdio->phy_addr, PHY_BMCR, bmcr | BMCR_SPEED_10 | BMCR_DUPLEX);
	}

	if (obj->loopback) {
		/* disable external loopback */
		aspeed_mdio_write(mdio, mdio->phy_addr, 0x18, 0x0400);
	} else {
		/* enable external loopback */
		aspeed_mdio_write(mdio, mdio->phy_addr, 0x18, 0x8400);
	}

	osDelay(100);
}

void phy_set_loopback(phy_t *obj, uint8_t enable)
{
	mdio_t *mdio = obj->mdio;
	uint32_t bmcr;

	bmcr = aspeed_mdio_read(mdio, mdio->phy_addr, PHY_BMCR);
	
	if (enable)
		bmcr |= BMCR_LOOPBACK;
	else	
		bmcr &= ~BMCR_LOOPBACK;

	aspeed_mdio_write(mdio, mdio->phy_addr, PHY_BMCR, bmcr);	
}