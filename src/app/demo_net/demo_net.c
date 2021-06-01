/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdint.h>
#include "objects.h"
#include "internal.h"
#include "mdio_aspeed.h"
#include "mac_aspeed.h"

int net_connect_mdio(int mac_idx, int mdio_idx)
{
	if ((mac_idx > N_DEVICE) || (mdio_idx > N_DEVICE))
		return -1;

	mac_objs[mac_idx].phy->mdio = &mdio_objs[mdio_idx];
	return 0;
}

void net_enable_pin(struct aspeed_group_config_s *group)
{
	struct aspeed_sig_desc_s *desc;
	uint32_t ndescs = group->ndescs;
	int i;

	for (i = 0; i < ndescs; i++) {
		desc = &group->descs[i];
		if (desc->clr)
			clrbits(SCU_BASE + desc->offset, desc->reg_set);
		else
			setbits(SCU_BASE + desc->offset, desc->reg_set);
	}
}

extern struct aspeed_group_config_s mdio_pinctrl[N_DEVICE];
extern struct aspeed_group_config_s rgmii_pinctrl[N_DEVICE];
extern struct aspeed_group_config_s rmii_pinctrl[N_DEVICE];

int net_enable_mdio_pin(int mdio_idx)
{
	net_enable_pin(&mdio_pinctrl[mdio_idx]);
	return 0;
}

int net_enable_rgmii_pin(int rgmii_idx)
{
	net_enable_pin(&rgmii_pinctrl[rgmii_idx]);
	return 0;
}

int net_enable_rmii_pin(int rmii_idx)
{
	net_enable_pin(&rmii_pinctrl[rmii_idx]);
	return 0;
}

hal_status_t net_get_packet(mac_t *obj, void **packet, uint32_t *rxlen, int max_try)
{
	hal_status_t status;
	int try = 0;

	do {
		status = aspeed_mac_recv(obj, packet, rxlen);
		if (status != HAL_TIMEOUT)
			break;

		osDelay(1);
	} while (++try < max_try);

	return status;
}


void demo_net_init(void)
{
	int i;

	for (i = 0; i < N_DEVICE; i++)
    	aspeed_mdio_init(&mdio_objs[i]);

#if CONFIG_DEMO_MDIO
	for (i = 0; i < N_DEVICE; i++)
		mdio_cmd_register_obj(&mdio_objs[i]);

	FreeRTOS_CLIRegisterCommand(&mdio_cmd);
#endif

	for (i = 0; i < N_DEVICE; i++)
		mac_cmd_register_obj(&mac_objs[i]);

	FreeRTOS_CLIRegisterCommand(&netdiag_cmd);
}