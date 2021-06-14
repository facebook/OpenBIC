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
	{ .device = &mdio1, .phy_addr = 0 },
	{ .device = &mdio2, .phy_addr = 0 },
	{ .device = &mdio3, .phy_addr = 0 },
};

phy_t phy_objs[N_DEVICE] = {
	{ .link = 0, .mdio = &mdio_objs[0], .loopback = 0 },
	{ .link = 0, .mdio = &mdio_objs[1], .loopback = 0 },
	{ .link = 0, .mdio = &mdio_objs[2], .loopback = 0 },
	{ .link = 0, .mdio = &mdio_objs[3], .loopback = 0 }
};

mac_t mac_objs[N_DEVICE] = {
	{ .device = &mac0, .phy = &phy_objs[0] },
	{ .device = &mac1, .phy = &phy_objs[1] },
	{ .device = &mac2, .phy = &phy_objs[2] },
	{ .device = &mac3, .phy = &phy_objs[3] },
};

static struct aspeed_sig_desc_s rgmii1[] = {
	{ 0x500, BIT(6), 0         	},
	{ 0x400, GENMASK(11, 0), 0 	},
};

static struct aspeed_sig_desc_s rgmii2[] = {
	{ 0x500, BIT(7), 0		},
	{ 0x400, GENMASK(23, 12), 0 	},
};

static struct aspeed_sig_desc_s rgmii3[] = {	
	{ 0x510, BIT(0), 0         },
	{ 0x410, GENMASK(27, 16), 0	},
};

static struct aspeed_sig_desc_s rgmii4[] = {
	{ 0x510, BIT(1), 0         },
	{ 0x410, GENMASK(31, 28), 1	},
	{ 0x4b0, GENMASK(31, 28), 0	},
	{ 0x474, GENMASK(7, 0), 1	},
	{ 0x414, GENMASK(7, 0), 1	},
	{ 0x4b4, GENMASK(7, 0), 0	},
};

struct aspeed_group_config_s rgmii_pinctrl[N_DEVICE] = {
	{ "RGMII1", ARRAY_SIZE(rgmii1), rgmii1 },
	{ "RGMII2", ARRAY_SIZE(rgmii2), rgmii2 },
	{ "RGMII3", ARRAY_SIZE(rgmii3), rgmii3 },
	{ "RGMII4", ARRAY_SIZE(rgmii4), rgmii4 },
};

static struct aspeed_sig_desc_s rmii1[] = {
	{ 0x504, BIT(6), 0         	},
	{ 0x400, GENMASK(3, 0), 0	},
	{ 0x400, GENMASK(11, 6), 0	},
};

static struct aspeed_sig_desc_s rmii2[] = {
	{ 0x504, BIT(7), 0         	},
	{ 0x400, GENMASK(15, 12), 0	},
	{ 0x400, GENMASK(23, 18), 0	},
};

static struct aspeed_sig_desc_s rmii3[] = {
	{ 0x514, BIT(0), 0         	},
	{ 0x410, GENMASK(27, 22), 0	},
	{ 0x410, GENMASK(19, 16), 0	},
};

static struct aspeed_sig_desc_s rmii4[] = {
	{ 0x514, BIT(1), 0         	},
	{ 0x410, GENMASK(7, 2), 1	},
	{ 0x410, GENMASK(31, 28), 1	},
	{ 0x414, GENMASK(7, 2), 1	},
	{ 0x4B0, GENMASK(31, 28), 0	},
	{ 0x4B4, GENMASK(7, 2), 0	},
};

struct aspeed_group_config_s rmii_pinctrl[N_DEVICE] = {
	{ "RMII1", ARRAY_SIZE(rmii1), rmii1 },
	{ "RMII2", ARRAY_SIZE(rmii2), rmii2 },
	{ "RMII3", ARRAY_SIZE(rmii3), rmii3 },
	{ "RMII4", ARRAY_SIZE(rmii4), rmii4 },
};

#if 0
static struct aspeed_sig_desc_s rmii1_rclk_oe[] = {
	{ 0x340, BIT(29), 0         	},
};
static struct aspeed_sig_desc_s rmii2_rclk_oe[] = {
	{ 0x340, BIT(30), 0         	},
};

static struct aspeed_sig_desc_s rmii3_rclk_oe[] = {
	{ 0x350, BIT(29), 0         	},
};
static struct aspeed_sig_desc_s rmii4_rclk_oe[] = {
	{ 0x350, BIT(30), 0         	},
};
#endif

static struct aspeed_sig_desc_s mdio1_link[] = {
	{ 0x430, BIT(17) | BIT(16), 0	},
};

static struct aspeed_sig_desc_s mdio2_link[] = {
	{ 0x470, BIT(13) | BIT(12), 1	},
	{ 0x410, BIT(13) | BIT(12), 0	},
};

static struct aspeed_sig_desc_s mdio3_link[] = {
	{ 0x470, BIT(1) | BIT(0), 1	},
	{ 0x410, BIT(1) | BIT(0), 0	},
};

static struct aspeed_sig_desc_s mdio4_link[] = {
	{ 0x470, BIT(3) | BIT(2), 1	},
	{ 0x410, BIT(3) | BIT(2), 0	},
};

struct aspeed_group_config_s mdio_pinctrl[N_DEVICE] = {
	{ "MDIO1", ARRAY_SIZE(mdio1_link), mdio1_link },
	{ "MDIO2", ARRAY_SIZE(mdio2_link), mdio2_link },
	{ "MDIO3", ARRAY_SIZE(mdio3_link), mdio3_link },
	{ "MDIO4", ARRAY_SIZE(mdio4_link), mdio4_link },
};