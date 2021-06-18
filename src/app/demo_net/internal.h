/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _NET_INTERNAL_H_
#define _NET_INTERNAL_H_
#include "FreeRTOS_CLI.h"

#if defined(CONFIG_AST2600_SERIES)
#define N_DEVICE	4
#elif defined(CONFIG_AST1030_SERIES)
#define N_DEVICE	1
#else
#error "unsupported soc"
#endif

#define MAX_DELAY_TAPS_RGMII_TX		63
#define MAX_DELAY_TAPS_RGMII_RX		63
#define MAX_DELAY_TAPS_RMII_TX		1
#define MAX_DELAY_TAPS_RMII_RX		63

struct aspeed_sig_desc_s {
	uint32_t offset;
	uint32_t reg_set;
	int clr;
};
struct aspeed_group_config_s {
	char *group_name;
	int ndescs;
	struct aspeed_sig_desc_s *descs;
};

extern mdio_t mdio_objs[N_DEVICE];
extern phy_t phy_objs[N_DEVICE];
extern mac_t mac_objs[N_DEVICE];

/* mdio command */
#ifdef CONFIG_DEMO_MDIO
const CLI_Command_Definition_t mdio_cmd;
void mdio_cmd_register_obj(mdio_t *obj);
#endif

/* nettest command */
const CLI_Command_Definition_t netdiag_cmd;
void mac_cmd_register_obj(mac_t *obj);

int net_enable_mdio_pin(int mdio_idx);
int net_enable_rgmii_pin(int rgmii_idx);
int net_enable_rmii_pin(int rmii_idx);
int net_connect_mdio(int mac_idx, int mdio_idx);
hal_status_t net_get_packet(mac_t *obj, void **packet, uint32_t *rxlen, int max_try);
#endif /* end of "#ifndef _NET_INTERNAL_H_" */