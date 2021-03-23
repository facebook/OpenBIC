/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdint.h>
#include <stdio.h>
#include "common.h"
#include "device.h"

typedef union {
	volatile uint32_t value;
	struct {
		volatile uint32_t tx_delay_1				: 6;	/* bit[5:0] */
		volatile uint32_t tx_delay_2				: 6;	/* bit[11:6] */
		volatile uint32_t rx_delay_1				: 6;	/* bit[17:12] */
		volatile uint32_t rx_delay_2				: 6;	/* bit[23:18] */
		volatile uint32_t rx_clk_inv_1 				: 1;	/* bit[24] */
		volatile uint32_t rx_clk_inv_2 				: 1;	/* bit[25] */
		volatile uint32_t rmii_tx_data_at_falling_1 : 1;	/* bit[26] */
		volatile uint32_t rmii_tx_data_at_falling_2 : 1;	/* bit[27] */
		volatile uint32_t rgmiick_pad_dir			: 1;	/* bit[28] */
		volatile uint32_t rmii_50m_oe_1 			: 1;	/* bit[29] */
		volatile uint32_t rmii_50m_oe_2				: 1;	/* bit[30] */
		volatile uint32_t rgmii_125m_o_sel 			: 1;	/* bit[31] */
	} fields;
} mac_delay_1g_t;

typedef union {
	volatile uint32_t value;
	struct {
		volatile uint32_t tx_delay_1				: 6;	/* bit[5:0] */
		volatile uint32_t tx_delay_2				: 6;	/* bit[11:6] */
		volatile uint32_t rx_delay_1				: 6;	/* bit[17:12] */
		volatile uint32_t rx_delay_2				: 6;	/* bit[23:18] */
		volatile uint32_t rx_clk_inv_1 				: 1;	/* bit[24] */
		volatile uint32_t rx_clk_inv_2 				: 1;	/* bit[25] */
		volatile uint32_t reserved_0 				: 6;	/* bit[31:26] */
	} fields;
} mac_delay_100_10_t;

uint32_t aspeed_clk_get_hclk(void)
{
	/* FIXME: this is the forced value on FPGA. */
#ifdef CONFIG_TARGET_FPGA_AST1030
	return (48000000);
#else
	return (200000000);
#endif
}

uint32_t aspeed_clk_get_pclk(void)
{
#ifdef CONFIG_TARGET_FPGA_AST1030
	return (12000000);
#else
	return (50000000);
#endif
}

/**
 * SCU310[5:0] -> UART6 to UART1
 * SCU314[12:6] -> UART13 to UART7
 * 	1: 192MHz / 13
 * 	0: 24MHz / 13
*/
uint32_t aspeed_clk_get_uart_clk(aspeed_device_t *device)
{
	uint32_t idx, addr;

	if (device->dev_id >= ASPEED_DEV_UART6) {
		addr = SCU_BASE + 0x314;
		idx = device->dev_id - ASPEED_DEV_UART6 + 6;
	} else {
		addr = SCU_BASE + 0x310;
		idx = device->dev_id - ASPEED_DEV_UART0;
	}

	if (readl(addr) & BIT(idx))
		return (192000000 / 13);
	else
		return (24000000 / 13);
}

int aspeed_clk_enable(aspeed_device_t *device)
{
	struct aspeed_clk_s *clk = device->clk;

	if (clk->reg_enable)
		writel(clk->bits, clk->reg_enable);
	
	return 0;
}

int aspeed_clk_disable(aspeed_device_t *device)
{
	struct aspeed_clk_s *clk = device->clk;

	if (clk->reg_disable)
		writel(clk->bits, clk->reg_disable);
	
	return 0;
}

void aspeed_clk_set_rgmii_delay(ASPEED_DEV_ID macdev, uint8_t speed, uint8_t tx, uint8_t rx)
{
	uint32_t addr, offset;

	addr = SCU_BASE + 0x350;

	if (speed == 0)
		offset = 0xc;
	else if (speed == 1)
		offset = 0x8;
	else
		offset = 0;
	
	if (offset) {
		/* 100M/10M delay configuration */
		mac_delay_100_10_t *reg = (mac_delay_100_10_t *)(addr + offset);
		reg->fields.tx_delay_1 = tx;
		reg->fields.rx_delay_1 = rx;
	} else {
		mac_delay_1g_t *reg = (mac_delay_1g_t *)(addr + offset);
		reg->fields.tx_delay_1 = tx;
		reg->fields.rx_delay_1 = rx;
	}
}