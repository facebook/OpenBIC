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
#include "kcs_aspeed.h"
#include "clk_aspeed.h"
#include "aspeed_lpc_reg.h"

#define KCS_DUMMY_ZERO	0x0

/* IPMI 2.0 - Table 9-1, KCS Interface Status Register Bits */
#define KCS_STR_STATE_MASK	GENMASK(7, 6)
#define	KCS_STR_STATE_SHIFT	6
#define	KCS_STR_CMD_DAT		BIT(3)
#define	KCS_STR_SMS_ATN		BIT(2)
#define	KCS_STR_IBF		BIT(1)
#define	KCS_STR_OBF		BIT(0)

/* IPMI 2.0 - Table 9-2, KCS Interface State Bits */
enum kcs_state {
	KCS_STATE_IDLE,
	KCS_STATE_READ,
	KCS_STATE_WRITE,
	KCS_STATE_ERROR,
	KCS_STATE_NUM
};

/* IPMI 2.0 - Table 9-3, KCS Interface Control Codes */
enum kcs_cmd_code {
	KCS_CMD_GET_STATUS_ABORT	= 0x60,
	KCS_CMD_WRITE_START			= 0x61,
	KCS_CMD_WRITE_END			= 0x62,
	KCS_CMD_READ_BYTE			= 0x68,
	KCS_CMD_NUM
};

/* IPMI 2.0 - Table 9-4, KCS Interface Status Codes */
enum kcs_error_code {
	KCS_NO_ERROR				= 0x00,
	KCS_ABORTED_BY_COMMAND		= 0x01,
	KCS_ILLEGAL_CONTROL_CODE	= 0x02,
	KCS_LENGTH_ERROR			= 0x06,
	KCS_UNSPECIFIED_ERROR		= 0xff
};

/* IPMI 2.0 - Figure 9. KCS Phase in Transfer Flow Chart */
enum kcs_phase {
	KCS_PHASE_IDLE,

	KCS_PHASE_WRITE_START,
	KCS_PHASE_WRITE_DATA,
	KCS_PHASE_WRITE_END_CMD,
	KCS_PHASE_WRITE_DONE,

	KCS_PHASE_WAIT_READ,
	KCS_PHASE_READ,

	KCS_PHASE_ABORT_ERROR1,
	KCS_PHASE_ABORT_ERROR2,
	KCS_PHASE_ERROR,

	KCS_PHASE_NUM
};

static void kcs_set_state(kcs_t *kcs, enum kcs_state stat)
{
	uint32_t reg;

	reg = LPC_RD(kcs->str);
	reg &= ~(KCS_STR_STATE_MASK);
	reg |= (stat << KCS_STR_STATE_SHIFT) & KCS_STR_STATE_MASK;
	LPC_WR(kcs->str, reg);
}

static void kcs_write_data(kcs_t *kcs, uint8_t data)
{
	LPC_WR(kcs->odr, data);
}

static uint8_t kcs_read_data(kcs_t *kcs)
{
	return LPC_RD(kcs->idr);
}

static void kcs_force_abort(kcs_t *kcs)
{
	kcs_set_state(kcs, KCS_STATE_ERROR);
	kcs_read_data(kcs);
	kcs_write_data(kcs, KCS_DUMMY_ZERO);
	kcs->ibuf_avail = 0;
	kcs->ibuf_idx = 0;
	kcs->phase = KCS_PHASE_ERROR;
}

static void kcs_handle_cmd(kcs_t *kcs)
{
	uint8_t cmd;

	kcs_set_state(kcs, KCS_STATE_WRITE);
	kcs_write_data(kcs, KCS_DUMMY_ZERO);

	cmd = kcs_read_data(kcs);
	switch (cmd) {
		case KCS_CMD_WRITE_START:
			kcs->phase = KCS_PHASE_WRITE_START;
			kcs->error = KCS_NO_ERROR;
			break;

		case KCS_CMD_WRITE_END:
			if (kcs->phase != KCS_PHASE_WRITE_DATA) {
				kcs_force_abort(kcs);
				break;
			}
			kcs->phase = KCS_PHASE_WRITE_END_CMD;
			break;

		case KCS_CMD_GET_STATUS_ABORT:
			if (kcs->error == KCS_NO_ERROR)
				kcs->error = KCS_ABORTED_BY_COMMAND;

			kcs->phase = KCS_PHASE_ABORT_ERROR1;
			break;

		default:
			kcs_force_abort(kcs);
			kcs->error = KCS_ILLEGAL_CONTROL_CODE;
			break;
	}
}

static void kcs_handle_data(kcs_t *kcs)
{
	uint8_t data;

	switch (kcs->phase) {
		case KCS_PHASE_WRITE_START:
			kcs->phase = KCS_PHASE_WRITE_DATA;
		/* fall through */
		case KCS_PHASE_WRITE_DATA:
			if (kcs->ibuf_idx < kcs->ibuf_sz) {
				kcs_set_state(kcs, KCS_STATE_WRITE);
				kcs_write_data(kcs, KCS_DUMMY_ZERO);
				kcs->ibuf[kcs->ibuf_idx] = kcs_read_data(kcs);
				kcs->ibuf_idx++;
			}
			else {
				kcs_force_abort(kcs);
				kcs->error = KCS_LENGTH_ERROR;
			}
			break;

		case KCS_PHASE_WRITE_END_CMD:
			if (kcs->ibuf_idx < kcs->ibuf_sz) {
				kcs_set_state(kcs, KCS_STATE_READ);
				kcs->ibuf[kcs->ibuf_idx] = kcs_read_data(kcs);
				kcs->ibuf_idx++;
				kcs->ibuf_avail = 1;
#ifdef USE_OS_FLAG_FOR_WAIT
				osEventFlagsSet(kcs->evt_id, 0x1);
#endif
				kcs->phase = KCS_PHASE_WRITE_DONE;
			}
			else {
				kcs_force_abort(kcs);
				kcs->error = KCS_LENGTH_ERROR;
			}
			break;

		case KCS_PHASE_READ:
			if (kcs->obuf_idx == kcs->obuf_data_sz)
				kcs_set_state(kcs, KCS_STATE_IDLE);

			data = kcs_read_data(kcs);
			if (data != KCS_CMD_READ_BYTE) {
				kcs_set_state(kcs, KCS_STATE_ERROR);
				kcs_write_data(kcs, KCS_DUMMY_ZERO);
				break;
			}

			if (kcs->obuf_idx == kcs->obuf_data_sz) {
				kcs_write_data(kcs, KCS_DUMMY_ZERO);
				kcs->phase = KCS_PHASE_IDLE;
				break;
			}

			kcs_write_data(kcs, kcs->obuf[kcs->obuf_idx]);
			kcs->obuf_idx++;
			break;

		case KCS_PHASE_ABORT_ERROR1:
			kcs_set_state(kcs, KCS_STATE_READ);
			kcs_read_data(kcs);
			kcs_write_data(kcs, kcs->error);
			kcs->phase = KCS_PHASE_ABORT_ERROR2;
			break;

		case KCS_PHASE_ABORT_ERROR2:
			kcs_set_state(kcs, KCS_STATE_IDLE);
			kcs_read_data(kcs);
			kcs_write_data(kcs, KCS_DUMMY_ZERO);
			kcs->phase = KCS_PHASE_IDLE;

		default:
			kcs_force_abort(kcs);
			break;
	}

}

static void aspeed_kcs_comm_isr(kcs_t *kcs)
{
	uint32_t stat;

	stat = LPC_RD(kcs->str);
	if (stat & KCS_STR_IBF) {
		if (stat & KCS_STR_CMD_DAT)
			kcs_handle_cmd(kcs);
		else
			kcs_handle_data(kcs);
	}
}

static void aspeed_kcs1_isr(void)
{
	aspeed_kcs_comm_isr(aspeed_irq_get_isr_context(Kcs1_IRQn));
}

static void aspeed_kcs2_isr(void)
{
	aspeed_kcs_comm_isr(aspeed_irq_get_isr_context(Kcs2_IRQn));
}

static void aspeed_kcs3_isr(void)
{
	aspeed_kcs_comm_isr(aspeed_irq_get_isr_context(Kcs3_IRQn));
}

static void aspeed_kcs4_isr(void)
{
	aspeed_kcs_comm_isr(aspeed_irq_get_isr_context(Kcs4_IRQn));
}

static void aspeed_kcs_config_ioregs(struct kcs_s *kcs)
{
	aspeed_kcs_priv_t *kcs_priv = (aspeed_kcs_priv_t *)kcs->device->private;

	switch (kcs_priv->chan) {
		case KCS_CH1:
			kcs->idr = IDR1;
			kcs->odr = ODR1;
			kcs->str = STR1;
			break;
		case KCS_CH2:
			kcs->idr = IDR2;
			kcs->odr = ODR2;
			kcs->str = STR2;
			break;
		case KCS_CH3:
			kcs->idr = IDR3;
			kcs->odr = ODR3;
			kcs->str = STR3;
			break;
		case KCS_CH4:
			kcs->idr = IDR4;
			kcs->odr = ODR4;
			kcs->str = STR4;
			break;
		default:
			break;
	}
}

static void aspeed_kcs_config_irq(struct kcs_s *kcs)
{
	aspeed_kcs_priv_t *kcs_priv = (aspeed_kcs_priv_t *)kcs->device->private;

	switch (kcs_priv->chan) {
		case KCS_CH1:
			aspeed_irq_register(Kcs1_IRQn, (uint32_t)aspeed_kcs1_isr, kcs);
			break;
		case KCS_CH2:
			aspeed_irq_register(Kcs2_IRQn, (uint32_t)aspeed_kcs2_isr, kcs);
			break;
		case KCS_CH3:
			aspeed_irq_register(Kcs3_IRQn, (uint32_t)aspeed_kcs3_isr, kcs);
			break;
		case KCS_CH4:
			aspeed_irq_register(Kcs4_IRQn, (uint32_t)aspeed_kcs4_isr, kcs);
			break;
		default:
			break;
	}
}

static void aspeed_kcs_config_addr(struct kcs_s *kcs)
{
	uint32_t reg;
	aspeed_kcs_priv_t *kcs_priv = kcs->device->private;

	switch (kcs_priv->chan) {
		case KCS_CH1:
			reg = LPC_RD(HICR4);
			reg &= ~(HICR4_LADR12AS);
			LPC_WR(HICR4, reg);
			LPC_WR(LADR12H, kcs_priv->addr >> 8);
			LPC_WR(LADR12L, kcs_priv->addr & 0xff);
			break;
		case KCS_CH2:
			reg = LPC_RD(HICR4);
			reg |= HICR4_LADR12AS;
			LPC_WR(HICR4, reg);
			LPC_WR(LADR12H, kcs_priv->addr >> 8);
			LPC_WR(LADR12L, kcs_priv->addr & 0xff);
			break;
		case KCS_CH3:
			LPC_WR(LADR3H, kcs_priv->addr >> 8);
			LPC_WR(LADR3L, kcs_priv->addr & 0xff);
			break;
		case KCS_CH4:
			LPC_WR(LADR4, kcs_priv->addr & 0xffff);
			break;
		default:
			break;
	}
}

static void aspeed_kcs_enable_chan(struct kcs_s *kcs)
{
	uint32_t reg;
	aspeed_kcs_priv_t *kcs_priv = kcs->device->private;

	switch(kcs_priv->chan) {
		case KCS_CH1:
			reg = LPC_RD(HICR2) | HICR2_IBFIF1;
			LPC_WR(HICR2, reg);
			reg = LPC_RD(HICR0) | HICR0_LPC1E;
			LPC_WR(HICR0, reg);
			break;
		case KCS_CH2:
			reg = LPC_RD(HICR2) | HICR2_IBFIF2;
			LPC_WR(HICR2, reg);
			reg = LPC_RD(HICR0) | HICR0_LPC2E;
			LPC_WR(HICR0, reg);
			break;
		case KCS_CH3:
			reg = LPC_RD(HICR2) | HICR2_IBFIF3;
			LPC_WR(HICR2, reg);
			reg = LPC_RD(HICR0) | HICR0_LPC3E;
			LPC_WR(HICR0, reg);
			break;
		case KCS_CH4:
			reg = LPC_RD(HICRB) | HICRB_IBFIF4 | HICRB_LPC4E;
			LPC_WR(HICRB, reg);
			break;
		default:
			break;
	}
}

int aspeed_kcs_read(struct kcs_s *kcs, uint8_t *buf, uint32_t buf_sz)
{
	int ret;

	if (kcs == NULL || buf == NULL)
		return -EINVAL;

	if (buf_sz < kcs->ibuf_idx)
		return -ENOSPC;

#ifdef USE_OF_FLAG_FOR_WAIT
	osEventFlagsWait(kcs->evt_id, 0x1, osFlagsWaitAll, osWaitForever);
#else
	while (!kcs->ibuf_avail)
		osDelay(1000);
#endif

	vPortEnterCritical();
	if (kcs->phase == KCS_PHASE_WRITE_DONE) {
		memcpy(buf, kcs->ibuf, kcs->ibuf_idx);
		ret = kcs->ibuf_idx;

		kcs->phase = KCS_PHASE_WAIT_READ;
		kcs->ibuf_avail = 0;
		kcs->ibuf_idx = 0;
	}
	else {
		kcs_force_abort(kcs);
		ret = -EPERM;
	}
	vPortExitCritical();

	return ret;
}

int aspeed_kcs_write(struct kcs_s *kcs, uint8_t *buf, uint32_t buf_sz)
{
	int ret;

	/* a minimum response size is 3: netfn + cmd + cmplt_code */
	if (buf_sz < 3 || buf_sz > kcs->obuf_sz)
		return -EINVAL; 

	vPortEnterCritical();
	if (kcs->phase == KCS_PHASE_WAIT_READ) {
		kcs->phase = KCS_PHASE_READ;
		kcs->obuf_idx = 1;
		kcs->obuf_data_sz = buf_sz;
		memcpy(kcs->obuf, buf, buf_sz);
		kcs_write_data(kcs, kcs->obuf[0]);
		ret = buf_sz;
	}
	else {
		ret = -EPERM;
	}
	vPortExitCritical();

	return ret;
}

void aspeed_kcs_init(struct kcs_s *kcs)
{
	aspeed_device_t *kcs_dev = kcs->device;
	aspeed_kcs_priv_t *kcs_priv = (aspeed_kcs_priv_t *)kcs_dev->private;

	if (kcs_priv->chan >= KCS_CH_NUM) {
		log_error("invalid KCS channel: %d\n", kcs_priv->chan);
		return;
	}

	if (kcs_dev->init) {
		log_error("KCS%d is occupied\n", kcs_priv->chan + 1);
		return;
	}

#ifdef USE_OS_FLAG_FOR_WAIT
	kcs->evt_id = osEventFlagsNew(NULL);
	if (kcs->evt_id == NULL) {
		log_error("failed to create event ID\n");
		return;
	}
#endif

	kcs->ibuf_idx = 0;
	kcs->ibuf_sz = ASPEED_KCS_BUF_SIZE;
	kcs->ibuf_avail = 0;
	kcs->ibuf = pvPortMalloc(kcs->ibuf_sz);
	if (kcs->ibuf == NULL) {
		log_error("failed to allocate KCS input buffer\n");
		return;
	}

	kcs->obuf_idx = 0;
	kcs->obuf_sz = ASPEED_KCS_BUF_SIZE;
	kcs->obuf_data_sz = 0;
	kcs->obuf = pvPortMalloc(kcs->obuf_sz);
	if (kcs->obuf == NULL) {
		log_error("failed to allocate KCS output buffer\n");
		return;
	}

	kcs->phase = KCS_PHASE_IDLE;

	aspeed_kcs_config_ioregs(kcs);
	aspeed_kcs_config_irq(kcs);
	aspeed_kcs_config_addr(kcs);
	aspeed_kcs_enable_chan(kcs);

	kcs_dev->init = 1;

	log_info("KCS%d: addr=0x%x, idr=0x%x, odr=0x%x, str=0x%x\n",
			kcs_priv->chan + 1, kcs_priv->addr,
			kcs->idr, kcs->odr, kcs->str);
}
