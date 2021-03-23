/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "objects.h"
#include "hal_def.h"
hal_status_t aspeed_mdio_init(mdio_t *obj);
int aspeed_mdio_read(mdio_t *obj, int phy_addr, int reg_addr);
int aspeed_mdio_write(mdio_t *obj, int phy_addr, int reg_addr, int data);
int aspeed_mdio_clrset(mdio_t *obj, int phy_addr, int reg_addr, int clr, int set);