/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <errno.h>
#include "common.h"
#include "util.h"
#include "hal_def.h"
#include "cmsis_os.h"
#include "log.h"
#include "irq_aspeed.h"
#include "bt_aspeed.h"
#include "aspeed_lpc_reg.h"

/* IPMI 2.0 - BT Interface Registers */
#define BT_CTRL		IBTCR4
#define 	BT_CTRL_B_BUSY	BIT(7)
#define 	BT_CTRL_H_BUSY	BIT(6)
#define 	BT_CTRL_OEM0	BIT(5)
#define 	BT_CTRL_SMS_ATN	BIT(4)
#define 	BT_CTRL_B2H_ATN	BIT(3)
#define 	BT_CTRL_H2B_ATN	BIT(2)
#define 	BT_CTRL_CLR_RDP	BIT(1)
#define 	BT_CTRL_CLR_WRP	BIT(0)
#define	BT_HOST2BMC	IBTCR5
#define BT_BMC2HOST	IBTCR5
#define	BT_INTMASK	IBTCR6

/* IPMI 2.0 - BT_CTRL register bit R/W by BMC */
static inline void clr_b_busy(bt_t *bt)
{
	if (LPC_RD(BT_CTRL) & BT_CTRL_B_BUSY)
		LPC_WR(BT_CTRL, BT_CTRL_B_BUSY);
}

static inline void set_b_busy(bt_t *bt)
{
	if (!(LPC_RD(BT_CTRL) & BT_CTRL_B_BUSY))
		LPC_WR(BT_CTRL, BT_CTRL_B_BUSY);
}

static inline void clr_oem0(bt_t *bt)
{
	LPC_WR(BT_CTRL, BT_CTRL_OEM0);
}

static inline void set_sms_atn(bt_t *bt)
{
	LPC_WR(BT_CTRL, BT_CTRL_SMS_ATN);
}

static inline void set_b2h_atn(bt_t *bt)
{
	LPC_WR(BT_CTRL, BT_CTRL_B2H_ATN);
}

static inline void clr_h2b_atn(bt_t *bt)
{
	LPC_WR(BT_CTRL, BT_CTRL_H2B_ATN);
}

static inline void clr_rd_ptr(bt_t *bt)
{
	LPC_WR(BT_CTRL, BT_CTRL_CLR_RDP);
}

static inline void clr_wr_ptr(bt_t *bt)
{
	LPC_WR(BT_CTRL, BT_CTRL_CLR_WRP);
}

static void aspeed_bt_isr(void)
{
	/*
	 * simply ack IRQ as currently
	 * only polling implementation
	 * is supported
	 */
	LPC_WR(IBTCR2, LPC_RD(IBTCR2));
}

int aspeed_bt_read(bt_t *bt, uint8_t *buf, uint32_t buf_sz)
{
	int i, len;
	uint32_t reg;

	if (bt == NULL || buf == NULL)
		return -EINVAL;

	reg = LPC_RD(BT_CTRL);
	if (!(reg & BT_CTRL_H2B_ATN))
		return -EAGAIN;

	set_b_busy(bt);
	clr_h2b_atn(bt);
	clr_rd_ptr(bt);

	buf[0] = LPC_RD(BT_HOST2BMC);
	len = (int)buf[0];
	len = ((len + 1) > buf_sz) ? buf_sz : len + 1;

	/* we pass the length back as well */
	for (i = 1; i < len; ++i)
		buf[i] = LPC_RD(BT_HOST2BMC);

	clr_b_busy(bt);

	return len;
}

int aspeed_bt_write(bt_t *bt, uint8_t *buf, uint32_t buf_sz)
{
	int i;
	uint32_t reg;

	if (bt == NULL || buf == NULL)
		return -EINVAL;

	/* Length + NetFn/LUN + Seq + Cmd + CmpltCode */
	if (buf_sz < 5 || buf_sz > ASPEED_BT_BUF_SIZE)
		return -EINVAL;

	reg = LPC_RD(BT_CTRL);
	if (reg & (BT_CTRL_H_BUSY | BT_CTRL_B2H_ATN))
		return -EAGAIN;

	clr_wr_ptr(bt);

	for (i = 0; i < buf_sz; ++i)
		LPC_WR(BT_BMC2HOST, buf[i]);

	set_b2h_atn(bt);

	return i;
}

void aspeed_bt_init(bt_t *bt)
{
	uint32_t reg;
	aspeed_device_t *bt_dev = bt->device;
	aspeed_bt_priv_t *bt_priv = bt_dev->private;

	if (bt_dev->init) {
		log_error("BT is occupied\n");
		return;
	}

	aspeed_irq_register(Bt_IRQn, (uint32_t)aspeed_bt_isr, bt);

	reg = LPC_RD(IBTCR1);
	reg |= (IBTCR1_INT_EN_H2B | IBTCR1_INT_EN_HBUSY);
	LPC_WR(IBTCR1, reg);

	reg = ((bt_priv->addr << IBTCR0_ADDR_SHIFT) & IBTCR0_ADDR_MASK)
		| ((bt_priv->sirq << IBTCR0_SIRQ_SHIFT) & IBTCR0_SIRQ_MASK)
		| IBTCR0_EN_CLR_SLV_RDP
		| IBTCR0_EN_CLR_SLV_WRP
		| IBTCR0_EN;
	LPC_WR(IBTCR0, reg);

	bt_dev->init = 1;

	clr_b_busy(bt);
}
