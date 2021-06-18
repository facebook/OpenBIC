/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "objects.h"
#include "hal_def.h"
#include "log.h"
#include "i3cdma_reg_aspeed.h"
#include "i3cdma_aspeed.h"
#include "reset_aspeed.h"

/**
 * @brief translate bytes size into block size
 * @param [IN] nbytes: transfer size in byte
 * @param [IN] block_size_in_log2: block size in log2, e.g. block size = 16bytes -> block_size_in_log2 = 4
 * @return transfer size in number of the block size
*/
static uint32_t nbytes_to_nblocks(uint32_t nbytes, uint32_t block_size_in_log2)
{
	nbytes += (1 << block_size_in_log2) - 1;
	return (nbytes >> block_size_in_log2);
}

/**
 * @brief clear all interrupt status
 * @param [IN] reg: register pointer to the i3c dma engine
*/
static void aspeed_i3cdma_clear_status(aspeed_i3cdma_reg_t *reg)
{
	log_debug("int status: tfr = %08x %08x\n", reg->raw.tfr, reg->status.tfr);
	log_debug("int status: block = %08x %08x\n", reg->raw.block, reg->status.block);
	log_debug("int status: srctran = %08x %08x\n", reg->raw.srctran, reg->status.srctran);
	log_debug("int status: dsttran = %08x %08x\n", reg->raw.dsttran, reg->status.dsttran);
	log_debug("int status: err = %08x %08x\n", reg->raw.err, reg->status.err);

	/* clear all interrupts  */
	reg->clear.tfr = 0xffffffff;
	reg->clear.block = 0xffffffff;
	reg->clear.srctran = 0xffffffff;
	reg->clear.dsttran = 0xffffffff;
	reg->clear.err = 0xffffffff;

	/* ensure all interrupts have been cleared */
	while(reg->raw.tfr);
	while(reg->raw.block);
	while(reg->raw.srctran);
	while(reg->raw.dsttran);
	while(reg->raw.err);

	while(reg->status.tfr);
	while(reg->status.block);
	while(reg->status.srctran);
	while(reg->status.dsttran);
	while(reg->status.err);
}

hal_status_t aspeed_i3cdma_go(i3cdma_t *obj, i3cdma_desc_t *desc)
{
	uint32_t base = obj->device->base;
	aspeed_i3cdma_reg_t *reg = (aspeed_i3cdma_reg_t *)base;

	aspeed_i3cdma_clear_status(reg);

	//reg->dma_cfg.b.dma_en = 0;
	//while (reg->dma_cfg.b.dma_en);

	/* enable DMA engine */
	reg->dma_cfg.b.dma_en = 1;

	/* set ch_en and ch_en_we */
	reg->ch_en = BIT(desc->channel) | BIT(desc->channel + 8);

	return HAL_OK;
}

hal_status_t aspeed_i3cdma_wait_done(i3cdma_t *obj, i3cdma_desc_t *desc)
{
	uint32_t base = obj->device->base;
	aspeed_i3cdma_reg_t *reg = (aspeed_i3cdma_reg_t *)base;

	/* wait DMA done */
	while (0 == (reg->raw.block & BIT(desc->channel)));
	reg->ch_en = BIT(desc->channel + 8);

	return HAL_OK;
}

hal_status_t aspeed_i3cdma_prepare(i3cdma_t *obj, i3cdma_desc_t *desc)
{
	uint32_t base = obj->device->base;
	aspeed_i3cdma_reg_t *reg = (aspeed_i3cdma_reg_t *)base;
	uint32_t idx = desc->channel;
	ctl_lo_reg_t ctl_lo_reg;
	ctl_hi_reg_t ctl_hi_reg;
	cfg_lo_reg_t cfg_lo_reg;
	cfg_hi_reg_t cfg_hi_reg;

	ctl_lo_reg.w = 0;
	ctl_hi_reg.w = 0;
	cfg_lo_reg.w = 0;
	cfg_hi_reg.w = 0;

	/* link-list is not supported now */
	reg->chan_reg[idx].llp = 0;

	if (desc->direction == DMA_MEM_TO_DEV) {
		reg->chan_reg[idx].sar = TO_PHY_ADDR(desc->src_addr);
		reg->chan_reg[idx].dar = desc->dst_addr - 0x60000000;

		ctl_lo_reg.b.src_tr_width = TR_WIDTH_128_BITS;
		ctl_lo_reg.b.sinc = ADDR_INC;
		ctl_lo_reg.b.src_msize = 0x1;

		ctl_lo_reg.b.dst_tr_width = TR_WIDTH_32_BITS;
		ctl_lo_reg.b.dinc = ADDR_NOCHANGE;
		ctl_lo_reg.b.dst_msize = 0x1;

		ctl_lo_reg.b.tt_fc = TT_MEM_TO_DEV_FC_DMAC;

		cfg_lo_reg.b.reload_dst = 0x1;

		ctl_lo_reg.b.sms = 1;
		ctl_lo_reg.b.dms = 1;

		cfg_hi_reg.w = 0x4000;

	} else {
		reg->chan_reg[idx].sar = desc->src_addr - 0x60000000;
		reg->chan_reg[idx].dar = TO_PHY_ADDR(desc->dst_addr);

		ctl_lo_reg.b.src_tr_width = TR_WIDTH_32_BITS;
		ctl_lo_reg.b.sinc = ADDR_NOCHANGE;
		ctl_lo_reg.b.src_msize = 0x1;

		ctl_lo_reg.b.dst_tr_width = TR_WIDTH_128_BITS;
		ctl_lo_reg.b.dinc = ADDR_INC;
		ctl_lo_reg.b.dst_msize = 0x1;

		//ctl_lo_reg.b.tt_fc = TT_DEV_TO_MEM_FC_DMAC;
		ctl_lo_reg.b.tt_fc = TT_DEV_TO_MEM_FC_DEV;

		cfg_lo_reg.b.reload_src = 0x1;

		cfg_hi_reg.w = 0x0080;
	}

	ctl_hi_reg.b.block_ts = nbytes_to_nblocks(desc->nbytes, ctl_lo_reg.b.src_tr_width);
	
	reg->chan_reg[idx].ctl_lo.w = ctl_lo_reg.w;
	reg->chan_reg[idx].ctl_hi.w = ctl_hi_reg.w;
	reg->chan_reg[idx].cfg_lo.w = cfg_lo_reg.w;
	reg->chan_reg[idx].cfg_hi.w = cfg_hi_reg.w;
	log_trace("%08x sar[31: 0] = %08x\n", (uint32_t)&reg->chan_reg[idx].sar, reg->chan_reg[idx].sar);
	log_trace("%08x dar[31: 0] = %08x\n", (uint32_t)&reg->chan_reg[idx].dar, reg->chan_reg[idx].dar);
	log_trace("%08x ctl[31: 0] = %08x, %08x\n", (uint32_t)&reg->chan_reg[idx].ctl_lo.w, ctl_lo_reg.w, reg->chan_reg[idx].ctl_lo.w);
	log_trace("%08x ctl[63:32] = %08x, %08x\n", (uint32_t)&reg->chan_reg[idx].ctl_hi.w, ctl_hi_reg.w, reg->chan_reg[idx].ctl_hi.w);
	log_trace("%08x cfg[31: 0] = %08x, %08x\n", (uint32_t)&reg->chan_reg[idx].cfg_lo.w, cfg_lo_reg.w, reg->chan_reg[idx].cfg_lo.w);
	log_trace("%08x cfg[63:32] = %08x, %08x\n", (uint32_t)&reg->chan_reg[idx].cfg_hi.w, cfg_hi_reg.w, reg->chan_reg[idx].cfg_hi.w);

	return HAL_OK;
}

hal_status_t aspeed_i3cdma_schedule_rx(i3cdma_t *obj, i3cdma_desc_t *desc)
{
	uint32_t base = obj->device->base;
	aspeed_i3cdma_reg_t *reg = (aspeed_i3cdma_reg_t *)base;

	reg->chan_reg[desc->channel].sar = desc->src_addr;
	reg->chan_reg[desc->channel].dar = desc->dst_addr;
	return HAL_OK;
}

hal_status_t aspeed_i3cdma_request_channel(i3cdma_t *obj, uint32_t *channel)
{
	uint32_t i = 0;
	uint32_t value;

	/* find inactive channel */
	value = obj->in_used & GENMASK(7, 0);
	for (i = 0; value & 0x1; i++)
		value = value >> 1;

	if (i > 7)
		return HAL_BUSY;

	obj->in_used |= BIT(i);
	*channel = i;

	return HAL_OK;
}

hal_status_t aspeed_i3cdma_init(i3cdma_t *obj)
{

	if (0 == obj->device->init) {
		obj->in_used = 0;
		obj->device->init = 1;
	}
	aspeed_reset_deassert(obj->device);

	return HAL_OK;
}