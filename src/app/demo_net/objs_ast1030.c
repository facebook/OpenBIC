/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "objects.h"
#include "board_device.h"
#include "internal.h"

mdio_t mdio_objs[N_DEVICE] = {
	{ .device = &mdio0, .phy_addr = 0 },
};

phy_t phy_objs[N_DEVICE] = {
	{ .link = 0, .mdio = &mdio_objs[0], .loopback = 0 },
};

mac_t mac_objs[N_DEVICE] = {
	{ .device = &mac0, .phy = &phy_objs[0] },
};

static struct aspeed_sig_desc_s rgmii1[] = {
	{ 0x510, BIT(0), 0         	},
	{ 0x410, GENMASK(31, 28), 0 	},
	{ 0x414, GENMASK(7, 0), 0 	},
};

struct aspeed_group_config_s rgmii_pinctrl[N_DEVICE] = {
	{ "RGMII1", ARRAY_SIZE(rgmii1), rgmii1 },
};

static struct aspeed_sig_desc_s rmii1[] = {
	{ 0x514, BIT(0), 0         	},
	{ 0x410, GENMASK(31, 28), 0	},
	{ 0x414, GENMASK(7, 4) | BIT(2), 0	},
};

struct aspeed_group_config_s rmii_pinctrl[N_DEVICE] = {
	{ "RMII1", ARRAY_SIZE(rmii1), rmii1 },
};

#if 0
static struct aspeed_sig_desc_s rmii1_rclk_oe[] = {
	{ 0x350, BIT(29), 0         	},
};
#endif

static struct aspeed_sig_desc_s mdio1_link[] = {
	{ 0x410, BIT(26) | BIT(25), 0	},
};

struct aspeed_group_config_s mdio_pinctrl[N_DEVICE] = {
	{ "MDIO1", ARRAY_SIZE(mdio1_link), mdio1_link },
};