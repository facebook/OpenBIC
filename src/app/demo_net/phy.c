/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include "common.h"
#include "mdio_aspeed.h"
#include "phy.h"
#include "log.h"

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

/* BMCR bitfield */
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

/**
 * @brief	scan PHY ID on all possible PHY addresses (from 0 to 31)
*/
static int is_phy_id_valid(int id1, int id2)
{
	int mask = 0xffff;

	id1 &= mask;
	id2 &= mask;

	if (((id1 == 0) && (id2 == 0)) || ((id1 == mask) && (id2 == mask)))
		return 0;
	else
		return 1;
}

int phy_init(phy_t *obj)
{
	int i, addr = -1;
	int id1, id2;

	for (i = 0; i < 32; i++) {
		id1 = aspeed_mdio_read(obj->mdio, i, PHY_ID_1);
		id2 = aspeed_mdio_read(obj->mdio, i, PHY_ID_2);
		if (is_phy_id_valid(id1, id2))
			addr = i;

		log_debug("[%02d] %04x %04x\n", i, id1, id2);
	}

	obj->mdio->phy_addr = (addr == -1) ? 0 : addr;
	obj->id[0] = aspeed_mdio_read(obj->mdio, addr, PHY_ID_1);
	obj->id[1] = aspeed_mdio_read(obj->mdio, addr, PHY_ID_2);

#ifdef CONFIG_AST1030A0
	/* AST1030 MDIO read fail, fixed PHY driver */
	obj->config = phy_rtl8211f_config;
#else
	/* TODO: add PHY lookup table */
	if ((obj->id[0] == 0x001c) && (obj->id[1] == 0xc916))
		obj->config = phy_rtl8211f_config;
	else
		obj->config = phy_bcm54616_config;
#endif

	obj->config(obj);
	printf("phy addr: %04x\n", obj->mdio->phy_addr);
	printf("phy id  : %04x %04x\n\n", obj->id[0], obj->id[1]);

	return addr;
}

static void phy_default_config(phy_t *obj)
{
	mdio_t *mdio = obj->mdio;
	uint32_t bmcr = BMCR_DUPLEX;

	switch (obj->speed) {
	case 1000:
		bmcr |= BMCR_SPEED_1000;
		break;
	case 100:
		bmcr |= BMCR_SPEED_100;
		break;
	case 10:
	default:
		break;
	}

	if (obj->loopback == PHY_LOOPBACK_INT)
		bmcr |= BMCR_LOOPBACK;

	if (obj->autoneg)
		bmcr |= BMCR_ANE;

	aspeed_mdio_write(mdio, mdio->phy_addr, PHY_BMCR, bmcr);
}

static void phy_reset(mdio_t *obj)
{
	aspeed_mdio_write(obj, obj->phy_addr, PHY_BMCR, BMCR_RESET);
#ifdef CONFIG_AST1030A0
	osDelay(10);
#else
	while (aspeed_mdio_read(obj, obj->phy_addr, PHY_BMCR) & BMCR_RESET);
#endif
}

void phy_rtl8211f_config(phy_t *obj)
{
	mdio_t *mdio = obj->mdio;
	uint32_t reg, mask;
	uint32_t physr = SR_DUPLEX | SR_LINK;

	phy_reset(mdio);
	phy_default_config(obj);

#ifndef CONFIG_AST1030A0
	/* TX interface delay: page 0xd08 reg 0x11[8] */
	aspeed_mdio_write(mdio, mdio->phy_addr, PHY_EPAGSR, 0xd08);
	reg = aspeed_mdio_read(mdio, mdio->phy_addr, 0x11);
	reg &= ~BIT(8);
	if ((obj->phy_mode == PHY_INTERFACE_MODE_RGMII_ID) ||
		(obj->phy_mode == PHY_INTERFACE_MODE_RGMII_TXID)) {
			reg |= BIT(8);
	}
	aspeed_mdio_write(mdio, mdio->phy_addr, 0x11, reg);
	
	/* RX interface delay: page 0xd08 reg 0x15[3] */
	reg = aspeed_mdio_read(mdio, mdio->phy_addr, 0x15);
	reg &= ~BIT(3);
	if ((obj->phy_mode == PHY_INTERFACE_MODE_RGMII_ID) ||
		(obj->phy_mode == PHY_INTERFACE_MODE_RGMII_RXID)) {
			reg |= BIT(3);
	}
	aspeed_mdio_write(mdio, mdio->phy_addr, 0x15, reg);
#endif
	/* page 0xa43 reg 0x18[10], enable MDI loopback
	 *            reg 0x18[11], MDI loopback for short cable
	 */
	aspeed_mdio_write(mdio, mdio->phy_addr, PHY_EPAGSR, 0xa43);
	if (obj->loopback == PHY_LOOPBACK_EXT) {
		aspeed_mdio_write(mdio, mdio->phy_addr, PHY_CR1, 0x2d18);
	} else {
		aspeed_mdio_write(mdio, mdio->phy_addr, PHY_CR1, 0x2118);
	}

	/* page 0 reg 0x09[9], advertise 1000Base-T full-duplex capacity */
	aspeed_mdio_write(mdio, mdio->phy_addr, PHY_EPAGSR, 0);
	switch (obj->speed) {
	case 1000:
		aspeed_mdio_write(mdio, mdio->phy_addr, PHY_GBCR, BIT(9));
		physr |= 0x2 << 4;
		break;
	case 100:
		physr |= 0x1 << 4;
	case 10:
		aspeed_mdio_write(mdio, mdio->phy_addr, PHY_GBCR, 0);
		break;
	}

	/* page 0xa43 reg 0x1a */
	aspeed_mdio_write(mdio, mdio->phy_addr, PHY_EPAGSR, 0xa43);
	mask = SR_SPEED | SR_DUPLEX | SR_LINK;
#ifdef CONFIG_AST1030A0
	osDelay(100);
#else
	while ((aspeed_mdio_read(mdio, mdio->phy_addr, PHY_SR) & mask) != physr);
#endif
	aspeed_mdio_write(mdio, mdio->phy_addr, PHY_EPAGSR, 0);
}

void phy_bcm54616_config(phy_t *obj)
{
	mdio_t *mdio = obj->mdio;
	uint32_t reg18, reg1c;

	phy_reset(mdio);
	phy_default_config(obj);

	/*
	 * RX interface delay: reg 0x18, shadow value b'0111: misc control
	 * bit[8] RGMII RXD to RXC skew
	 */
	aspeed_mdio_write(mdio, mdio->phy_addr, 0x18, (0x7 << 12) | 0x7);
	reg18 = aspeed_mdio_read(mdio, mdio->phy_addr, 0x18);
	log_debug("brcm-phy reg18 = %04x\n", reg18);
	reg18 &= ~(GENMASK(14, 12) | BIT(8) | GENMASK(2, 0));
	reg18 |= BIT(15) | (0x7 << 12) | 0x7;
	if ((obj->phy_mode == PHY_INTERFACE_MODE_RGMII_ID) ||
		(obj->phy_mode == PHY_INTERFACE_MODE_RGMII_RXID)) {
			reg18 |= BIT(8);
	}
	aspeed_mdio_write(mdio, mdio->phy_addr, 0x18, reg18);

	/*
	 * TX interface delay: reg 0x1c, shadow value b'0011: clock alignment control
	 * bit[9] GTXCLK clock delay enable
	 */
	aspeed_mdio_write(mdio, mdio->phy_addr, 0x1c, 0x3 << 10);
	reg1c = aspeed_mdio_read(mdio, mdio->phy_addr, 0x1c);
	log_debug("brcm-phy reg1c = %04x\n", reg1c);
	reg1c &= ~(GENMASK(14, 10) | BIT(9));
	reg1c |= BIT(15) | (0x3 << 10);
	if ((obj->phy_mode == PHY_INTERFACE_MODE_RGMII_ID) ||
		(obj->phy_mode == PHY_INTERFACE_MODE_RGMII_TXID)) {
			reg1c |= BIT(9);
	}
	aspeed_mdio_write(mdio, mdio->phy_addr, 0x1c, reg1c);

	/*
	 * Special setting for external loopback 
	 */
	if (obj->loopback == PHY_LOOPBACK_EXT) {
		switch (obj->speed) {
		case 100:
			aspeed_mdio_write(mdio, mdio->phy_addr, 0x0, 0x2100);
			aspeed_mdio_write(mdio, mdio->phy_addr, 0x18, 0x8400);
			break;
		case 10:
			aspeed_mdio_write(mdio, mdio->phy_addr, 0x0, 0x0100);
			aspeed_mdio_write(mdio, mdio->phy_addr, 0x18, 0x8400);
		case 1000:
			aspeed_mdio_write(mdio, mdio->phy_addr, 0x9, 0x1800);
			aspeed_mdio_write(mdio, mdio->phy_addr, 0x0, 0x0140);
			aspeed_mdio_write(mdio, mdio->phy_addr, 0x18, 0x8400);
			break;
		}
	} else if (obj->loopback == PHY_LOOPBACK_INT) {
		/* BCM54213 needs to set force-link: reg1E[12] */
		if ((obj->id[0] == 0x600d) && (obj->id[1] == 0x84a2))
			aspeed_mdio_write(mdio, mdio->phy_addr, 0x1e, BIT(12));
	}

	osDelay(100);
}