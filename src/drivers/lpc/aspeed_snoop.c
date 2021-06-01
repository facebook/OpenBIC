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
#include "snoop_aspeed.h"
#include "aspeed_lpc_reg.h"

static void aspeed_snoop_isr(void)
{
	uint32_t stat, reg;
	uint8_t data;
	snoop_t *snoop = (snoop_t *)aspeed_irq_get_isr_context(Snoop_IRQn);

	stat = LPC_RD(HICR6);
	reg = LPC_RD(SNPWDR);

	if (stat & HICR6_STR_SNP0W) {
		data = ((reg & SNPWDR_DATA0_MASK) >> SNPWDR_DATA0_SHIFT);
		osMessageQueuePut(snoop->msg_q[0], &data, 0, 0);
	}

	if (stat & HICR6_STR_SNP1W) {
		data = ((reg & SNPWDR_DATA1_MASK) >> SNPWDR_DATA1_SHIFT);
		osMessageQueuePut(snoop->msg_q[1], &data, 0, 0);
	}

	LPC_WR(HICR6, HICR6_STR_SNP0W | HICR6_STR_SNP1W);
}

static void aspeed_snoop_enable(snoop_t *snoop, uint32_t ch)
{
	uint32_t hicr5, hicrb, snpwadr;
	aspeed_snoop_priv_t *snoop_priv = snoop->device->private;

	snoop->msg_q[ch] = osMessageQueueNew(
			ASPEED_SNOOP_MSG_QUEUE_SIZE, sizeof(uint8_t), NULL);
	if (snoop->msg_q[ch] == NULL) {
		log_error("failed to allocate data queue for snoop%d\n", ch);
		return;
	}

	hicr5 = LPC_RD(HICR5);
	snpwadr = LPC_RD(SNPWADR);
	hicrb = LPC_RD(HICRB);

	switch (ch) {
		case 0:
			hicr5 |= (HICR5_EN_SNP0W | HICR5_ENINT_SNP0W);
			snpwadr &= ~(SNPWADR_ADDR0_MASK);
			snpwadr |= ((snoop_priv->chan[ch].port_addr << SNPWADR_ADDR0_SHIFT) & SNPWADR_ADDR0_MASK);
			hicrb |= HICRB_ENSNP0D;
			break;
		case 1:
			hicr5 |= (HICR5_EN_SNP1W | HICR5_ENINT_SNP1W);
			snpwadr &= ~(SNPWADR_ADDR1_MASK);
			snpwadr |= ((snoop_priv->chan[ch].port_addr << SNPWADR_ADDR1_SHIFT) & SNPWADR_ADDR1_MASK);
			hicrb |= HICRB_ENSNP1D;
			break;
		default:
			log_error("unsupported snoop channel %d\n", ch);
			return;
	}

	LPC_WR(HICR5, hicr5);
	LPC_WR(SNPWADR, snpwadr);
	LPC_WR(HICRB, hicrb);
}

int aspeed_snoop_read(snoop_t *snoop, uint32_t chan, uint8_t *buf, uint32_t buf_sz)
{
	int i, count;
	osMessageQueueId_t mq;

	aspeed_device_t *snoop_dev = snoop->device;
	aspeed_snoop_priv_t *snoop_priv = snoop_dev->private;

	if (snoop == NULL || buf == NULL)
		return -EINVAL;

	if (chan >= ASPEED_SNOOP_CHANNEL_NUM)
		return -EINVAL;

	if (!snoop_dev->init)
		return -ENXIO;

	if (!snoop_priv->chan[chan].enable)
		return -EPERM;

	mq = snoop->msg_q[chan];

	count = osMessageQueueGetCount(mq);
	if (count == 0)
		return -ENODATA;

	if (buf_sz < count)
		return -ENOSPC;

	for (i = 0; i < count; ++i)
		osMessageQueueGet(mq, buf + i, NULL, 0);

	return count;
}

void aspeed_snoop_init(snoop_t *snoop)
{
	int i;
	aspeed_device_t *snoop_dev = snoop->device;
	aspeed_snoop_priv_t *snoop_priv = snoop_dev->private;

	if (snoop_dev->init) {
		log_error("snoop device is occupied\n");
		return;
	}

	aspeed_irq_register(Snoop_IRQn, (uint32_t)aspeed_snoop_isr, snoop);

	for (i = 0; i < ASPEED_SNOOP_CHANNEL_NUM; ++i)
		if (snoop_priv->chan[i].enable)
			aspeed_snoop_enable(snoop, i);

	snoop_dev->init = 1;
}
