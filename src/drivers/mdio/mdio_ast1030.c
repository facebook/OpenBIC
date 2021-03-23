/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "common.h"
#include "objects.h"
#include "hal_def.h"
#include "reset_aspeed.h"
#include "wait.h"

typedef union {
	volatile  uint32_t value;
	struct {
		volatile uint32_t reg_addr : 5;  	/* [ 4: 0] */
		volatile uint32_t phy_addr : 5;  	/* [ 9: 5] */
		volatile uint32_t op_code : 2;  	/* [11:10] */
		volatile uint32_t st_code : 1;  	/* [12] */
		volatile uint32_t reserved : 2;  	/* [14:13] */
		volatile uint32_t fire_busy : 1;  	/* [15] */
		volatile uint32_t miiwdata : 16;	/* [31: 16] */
	} fields;
} control_reg_t;

typedef union {
	volatile  uint32_t value;
	struct {
		volatile uint32_t miirdata : 16;	/* [15: 0] */
		volatile uint32_t reserved : 8;  	/* [23:16] */
		volatile uint32_t cycle : 8;  		/* [31:24] */
	} fields;
} data_reg_t;

typedef struct {
	control_reg_t ctrl;
	data_reg_t	data;
} mdio_register_t;

hal_status_t aspeed_mdio_init(mdio_t *obj)
{
	aspeed_reset_deassert(obj->device);
	return HAL_OK;
}

int aspeed_mdio_read(mdio_t *obj, int phy_addr, int reg_addr)
{
	mdio_register_t *reg = (mdio_register_t *)(obj->device->base);
	control_reg_t ctrl;

	ctrl.value = 0;
	ctrl.fields.fire_busy = 1;
	ctrl.fields.st_code = 1;
	ctrl.fields.op_code = 0x2;
	ctrl.fields.phy_addr = phy_addr;
	ctrl.fields.reg_addr = reg_addr;
	reg->ctrl.value = ctrl.value;
	
	for (int i = 0; i < 10; i++) {
		if (0 == reg->ctrl.fields.fire_busy) {
			return reg->data.fields.miirdata;
		}
		aspeed_wait_ms(10);
	}

	return -1;
}

int aspeed_mdio_write(mdio_t *obj, int phy_addr, int reg_addr, int data)
{
	mdio_register_t *reg = (mdio_register_t *)(obj->device->base);
	control_reg_t ctrl;

	ctrl.value = 0;
	ctrl.fields.fire_busy = 1;
	ctrl.fields.st_code = 1;
	ctrl.fields.op_code = 0x1;
	ctrl.fields.phy_addr = phy_addr;
	ctrl.fields.reg_addr = reg_addr;
	ctrl.fields.miiwdata = data;
	reg->ctrl.value = ctrl.value;
	
	for (int i = 0; i < 10; i++) {
		if (0 == reg->ctrl.fields.fire_busy) {
			return 0;
		}
		aspeed_wait_ms(10);
	}

	return -1;
}

int aspeed_mdio_clrset(mdio_t *obj, int phy_addr, int reg_addr, int clr, int set)
{
	int data = aspeed_mdio_read(obj, phy_addr, reg_addr);

	if (data == -1)
		return -1;

	data &= ~clr;
	data |= set;
	return aspeed_mdio_write(obj, phy_addr, reg_addr, data);
}