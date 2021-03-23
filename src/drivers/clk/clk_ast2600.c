/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdint.h>
#include <stdio.h>
#include "common.h"
#include "device.h"
#include "log.h"

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

#ifdef CONFIG_TARGET_FPGA_AST1030
static uint32_t aspeed_uart4_clk_table[4] = { 24000000 / 13, 24000000 / 13,
					      24000000 / 13, 24000000 / 13 };
#else
static uint32_t aspeed_uart4_clk_table[4] = { 24000000, 192000000,
					      24000000 / 13, 192000000 / 13 };
#endif

uint32_t aspeed_clk_get_hclk(void)
{
	/* TODO: calculate frequcy from the SCU setting */
	return (200000000);
}

uint32_t aspeed_clk_get_pclk(void)
{
	/* TODO: calculate frequcy from the SCU setting */
	return (37500000);
}

uint32_t aspeed_clk_get_apb2(void)
{
	/* TODO: calculate frequcy from the SCU setting */
	return (100000000);
}

uint32_t aspeed_clk_get_uart_clk(aspeed_device_t *device)
{
    uint32_t idx = 0;
	uint32_t ret = 0;

    switch (device->dev_id) {
    case ASPEED_DEV_UART0:
    case ASPEED_DEV_UART1:
    case ASPEED_DEV_UART2:
    case ASPEED_DEV_UART3:
    case ASPEED_DEV_UART5:
		/* TBD */
		break;
    case ASPEED_DEV_UART4:
		idx = 0;
		if (readl(SCU_BASE + 0xc0) & BIT(12))
		    idx |= BIT(1);

		if (readl(SCU_BASE + 0x304) & BIT(14))
		    idx |= BIT(0);

		ret = aspeed_uart4_clk_table[idx];
		break;
    case ASPEED_DEV_UART6:
    case ASPEED_DEV_UART7:
    case ASPEED_DEV_UART8:
    case ASPEED_DEV_UART9:
    case ASPEED_DEV_UART10:
    case ASPEED_DEV_UART11:
    case ASPEED_DEV_UART12:
		/* TBD */
		break;
    }

    return ret;
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
	uint32_t addr, offset, group;

	switch (macdev) {
		case ASPEED_DEV_MAC0:
			addr = SCU_BASE + 0x340;
			group = 0;
			break;
		case ASPEED_DEV_MAC1:
			addr = SCU_BASE + 0x340;
			group = 1;
			break;
		case ASPEED_DEV_MAC2:
			addr = SCU_BASE + 0x350;
			group = 0;
			break;
		case ASPEED_DEV_MAC3:
			addr = SCU_BASE + 0x350;
			group = 1;
			break;
		default:
			return;
	}

	if (speed == 0)
		offset = 0xc;
	else if (speed == 1)
		offset = 0x8;
	else
		offset = 0;
	
	if (offset) {
		/* 100M/10M delay configuration */
		mac_delay_100_10_t *reg = (mac_delay_100_10_t *)(addr + offset);
		if (group) {
			reg->fields.tx_delay_2 = tx;
			reg->fields.rx_delay_2 = rx;
		} else {
			reg->fields.tx_delay_1 = tx;
			reg->fields.rx_delay_1 = rx;
		}
	} else {
		mac_delay_1g_t *reg = (mac_delay_1g_t *)(addr + offset);
		if (group) {
			reg->fields.tx_delay_2 = tx;
			reg->fields.rx_delay_2 = rx;
		} else {
			reg->fields.tx_delay_1 = tx;
			reg->fields.rx_delay_1 = rx;
		}
	}
}