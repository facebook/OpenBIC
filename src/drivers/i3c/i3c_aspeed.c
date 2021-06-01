/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include "common.h"
#include "util.h"
#include "hal_def.h"
#include "i3c_reg_aspeed.h"
#include "i3c_aspeed.h"
#include "i3c_api.h"
#include "i3cdma_aspeed.h"
#include "reset_aspeed.h"
#include "clk_aspeed.h"
#include "cmsis_os.h"
#include "ccc.h"
#include "io.h"
#include <string.h>
#include "log.h"
#include "wait.h"

#define DEV_ID_TO_I3C_INDEX(x)	(x - ASPEED_DEV_I3C0)
#define USE_OS_FLAG_FOR_WAIT
#ifdef USE_OS_FLAG_FOR_WAIT
#include "irq_aspeed.h"
#define DEV_ID_TO_EVENT_FLAG(x)	(1 << DEV_ID_TO_I3C_INDEX(x))
#endif

//#define IOP_ASPEED_WORKAROUND

#define I3C_GLOBAL_CTL(x)	((x) * 0x10 + 0x10)
#define I3C_GLOBAL_SET(x)	((x) * 0x10 + 0x14)

#define DEFAULT_SLAVE_STATIC_ADDR	0x74
#define DEFAULT_SLAVE_INST_ID		0x4
#define DEFAULT_SLAVE_SETUP			((DEFAULT_SLAVE_INST_ID << 16) 		|\
									(DEFAULT_SLAVE_STATIC_ADDR << 8)	|\
									0xc4)

#define I3C_BROADCAST_ADDR			0x7e
#define I3C_MAX_ADDR				GENMASK(6, 0)

#define SCL_OP_MODE_CLK				400000		/* default 400kHz fast mode */
#define SCL_OP_MODE_PERIOD_NS		(1000000000 / SCL_OP_MODE_CLK)
#define SCL_PP_MODE_CLK				12500000
#define SCL_PP_MODE_PERIOD_NS		(1000000000 / SCL_PP_MODE_CLK)

#define CEIL_BYTES_TO_DWORDS(x)		(((x) + 3) >> 2)

typedef struct i3c_cmd_s {
	uint32_t cmd_lo;
	uint32_t cmd_hi;
	uint16_t tx_len;
	const void *tx_buf;
	uint16_t rx_len;
	void *rx_buf;
	uint8_t error;
} i3c_cmd_t;

typedef struct i3c_xfer_s {
	int32_t ret;
	uint32_t ncmds;
	i3c_cmd_t cmds[8];
} i3c_xfer_t;

#ifdef USE_OS_FLAG_FOR_WAIT
static osEventFlagsId_t evt_id;
#endif

void aspeed_i3c_global_init(struct i3c_global_s *obj)
{
	uint32_t base = obj->device->base;
	int i;

	aspeed_reset_assert(obj->device);
	/* OS is not start yet, use cpu looping for waiting */
	aspeed_wait_ms(1);
	aspeed_reset_deassert(obj->device);

	for (i = 0; i < ASPEED_DEV_I3C_NUM - ASPEED_DEV_I3C0; i++)
		writel(DEFAULT_SLAVE_SETUP, base + I3C_GLOBAL_SET(i));

#ifdef USE_OS_FLAG_FOR_WAIT
	/* init event ID for ISR */
	evt_id = osEventFlagsNew(NULL);
	if (evt_id == NULL)
	    log_error("fail to create evt_id\n");
#endif
}


/*
1e7a2000: 80000102 80080000 00000301 00000000                                                                           │
1e7a2010: 00000000 00000000 8000ff00 01000101                                                                           │
1e7a2020: 01010001 00000000 00000000 ffffffff                                                                           │
1e7a2030: ffffffff 00000000 0000000b 00000000                                                                           │
1e7a2040: 00000210 00000210 00000000 00080010                                                                           │
1e7a2050: 00000020 01000007 00000000 00100280                                                                           │
1e7a2060: 00020200 00000000 00000100 000000b0                                                                           │
1e7a2070: 00000000 00004000 00070022 00ff00ff
1e7a2080: 00000000 00000000 00000000 00000000
1e7a2090: 0000003f 00000000 00000000 00000000
1e7a20a0: 00000000 00000000 00000000 00000000
1e7a20b0: 00000000 00fb00fb 00090009 00fb00fb
1e7a20c0: 00fb00fb 00000000 5b291910 00030000
1e7a20d0: 00000000 000000fb 00000020 00000000
1e7a20e0: 3130302a 65613632 00000179 00000017
1e7a20f0: 00000000 00000000 00000000 00000000

1e7a2280: 00700000 c58330c3 74277cc7 b84a1dd0
1e7a2290: bd8f8163 ad046151 3d6c6250 d4a3b6d2
1e7a22a0: b1741553 ec34c9be 75981e31 ee8014f8
1e7a22b0: a20a813b 47d35e4b c569deaf 00fe0000
1e7a22c0: 00700000 00700000 00700000 00700000
1e7a22d0: 00700000 00700000 00700000 00700000
1e7a22e0: 00700000 00700000 00700000 00700000
1e7a22f0: 00700000 00700000 00700000 00700000

ccc log:
i3c0: dw_i3c_ccc_set:cmd_hi=0x00000001 cmd_lo=0x44008300 tx_len=0 id=6
i3c0: dw_i3c_ccc_set:cmd_hi=0x00010001 cmd_lo=0x44008080 tx_len=1 id=1
i3c0: TX data = 0000000b
i3c0: dw_i3c_ccc_set:cmd_hi=0x00010001 cmd_lo=0x4400c380 tx_len=1 id=135
i3c0: TX data = 000000e0
i3c0: dw_i3c_ccc_get:cmd_hi=0x00060001 cmd_lo=0x5400c680 rx_len=6 id=141
i3c0: dw_i3c_ccc_get:cmd_hi=0x00010001 cmd_lo=0x5400c700 rx_len=1 id=142
i3c0: dw_i3c_ccc_get:cmd_hi=0x00010001 cmd_lo=0x5400c780 rx_len=1 id=143
i3c0: dw_i3c_ccc_get:cmd_hi=0x00020001 cmd_lo=0x5400c600 rx_len=2 id=140
i3c0: dw_i3c_ccc_get:cmd_hi=0x00020001 cmd_lo=0x5400c580 rx_len=2 id=139
i3c0: dw_i3c_ccc_set:cmd_hi=0x00000001 cmd_lo=0x44009480 tx_len=0 id=41

read slave[0x12] for 1 byte
i3c0: dw_i3c_master_priv_xfers:cmd_hi=0x00020001 cmd_lo=0x04000000 tx_len=2 rx_l0
i3c0: dw_i3c_master_priv_xfers:cmd_hi=0x00010001 cmd_lo=0x54000008 tx_len=0 rx_l1
i3c0: TX data = 00000012
*/

static int master_get_addr_pos(struct i3c_s *master, uint8_t addr)
{
	int pos;

	for (pos = 0; pos < master->maxdevs; pos++) {
		if (addr == master->addrs[pos])
			return pos;
	}

	return -1;
}

static void aspeed_i3c_wr_tx_fifo(struct i3c_s *obj, const uint8_t *bytes,
				  int nbytes)
{
	aspeed_device_t *master = obj->device;

	if (0 == nbytes)
		return;

#ifdef CONFIG_DEVICE_I3CDMA
	if (obj->tx_dma_en) {
		writel(RESET_CTRL_TX_FIFO, master->base + RESET_CTRL);
		obj->tx_dma_desc->src_addr = (uint32_t)bytes;
		obj->tx_dma_desc->dst_addr = master->base + RX_TX_DATA_PORT;
		obj->tx_dma_desc->nbytes = nbytes;
		obj->tx_dma_desc->direction = DMA_MEM_TO_DEV;
		aspeed_i3cdma_prepare(obj->tx_dma_desc->dma, obj->tx_dma_desc);
		aspeed_i3cdma_go(obj->tx_dma_desc->dma, obj->tx_dma_desc);
		aspeed_i3cdma_wait_done(obj->tx_dma_desc->dma, obj->tx_dma_desc);
	} else 
#endif
	{
		writesl((void *)(master->base + RX_TX_DATA_PORT), bytes, (nbytes >> 2));
		if (nbytes & 0x3) {
			uint32_t tmp = 0;

			memcpy(&tmp, bytes + (nbytes & ~0x3), nbytes & 3);
			writesl((void *)(master->base + RX_TX_DATA_PORT), &tmp, 1);
			log_debug("TX data = %08x\n", tmp);
		}
	}
}

static void aspeed_i3c_read_rx_fifo(struct i3c_s *obj, uint8_t *bytes, int nbytes)
{
	aspeed_device_t *master = obj->device;

	if (0 == nbytes)
		return;

#ifdef CONFIG_DEVICE_I3CDMA
	if (obj->rx_dma_en) {
		obj->rx_dma_desc->src_addr = master->base + RX_TX_DATA_PORT;
		obj->rx_dma_desc->dst_addr = (uint32_t)bytes;
		obj->rx_dma_desc->nbytes = nbytes;
		obj->rx_dma_desc->direction = DMA_DEV_TO_MEM;
		log_debug("rx_dma: src=%08x, dst=%08x, nbytes=%d\n", obj->rx_dma_desc->src_addr, obj->rx_dma_desc->dst_addr, obj->rx_dma_desc->nbytes);
		aspeed_i3cdma_prepare(obj->rx_dma_desc->dma, obj->rx_dma_desc);
		aspeed_i3cdma_go(obj->rx_dma_desc->dma, obj->rx_dma_desc);
		aspeed_i3cdma_wait_done(obj->rx_dma_desc->dma, obj->rx_dma_desc);
		writel(RESET_CTRL_RX_FIFO, master->base + RESET_CTRL);
	} else 
#endif
	{
		readsl((void *)master->base + RX_TX_DATA_PORT, bytes, nbytes / 4);
		if (nbytes & 3) {
			uint32_t tmp;

			readsl((void *)master->base + RX_TX_DATA_PORT, &tmp, 1);
			memcpy(bytes + (nbytes & ~3), &tmp, nbytes & 3);
		}
	}
}

static void aspeed_i3c_start_xfer(struct i3c_s *obj, i3c_xfer_t *xfer)
{
	uint32_t base = obj->device->base;
	uint32_t thld_ctrl;
	int i;

	/* step 1: write TX buffer */
	for (i = 0; i < xfer->ncmds; i++) {
	    i3c_cmd_t *cmd = &xfer->cmds[i];
	    aspeed_i3c_wr_tx_fifo(obj, cmd->tx_buf, cmd->tx_len);
	}

	/* step 2: set RX queue threshold to the number of thansfers */
	thld_ctrl = readl(base + QUEUE_THLD_CTRL);
	thld_ctrl &= ~QUEUE_THLD_CTRL_RESP_BUF_MASK;
	thld_ctrl |= QUEUE_THLD_CTRL_RESP_BUF(xfer->ncmds);
	log_trace("set 0x1c = %08x\n", thld_ctrl);
	writel(thld_ctrl, base + QUEUE_THLD_CTRL);

	//if (obj->dma_en)
	//	aspeed_i3cdma_wait_done(obj->dma_desc->dma, obj->dma_desc);

	/* step 3: write command port */
	for (i = 0; i < xfer->ncmds; i++) {
		i3c_cmd_t *cmd = &xfer->cmds[i];

		writel(cmd->cmd_hi, base + COMMAND_QUEUE_PORT);
		writel(cmd->cmd_lo, base + COMMAND_QUEUE_PORT);
#ifdef IOP_ASPEED_WORKAROUND
		aspeed_wait_us(100);
#endif
	}
}

static void aspeed_i3c_end_xfer(struct i3c_s *obj, i3c_xfer_t *xfer)
{
	uint32_t base = obj->device->base;
	uint32_t nresp, i;

	xfer->ret = 0;
	nresp = readl(base + QUEUE_STATUS_LEVEL);
	nresp = QUEUE_STATUS_LEVEL_RESP(nresp);

	for (i = 0; i < nresp; i++) {
		i3c_cmd_t *cmd;
		uint32_t resp;

		resp = readl(base + RESPONSE_QUEUE_PORT);
		log_trace("i3c%d get resp %08x\n", DEV_ID_TO_I3C_INDEX(obj->device->dev_id), resp);
		cmd = &xfer->cmds[RESPONSE_PORT_TID(resp)];
		cmd->rx_len = RESPONSE_PORT_DATA_LEN(resp);
		cmd->error = RESPONSE_PORT_ERR_STATUS(resp);

		if (cmd->error) {
			log_error("i3c%d get error: resume\n", DEV_ID_TO_I3C_INDEX(obj->device->dev_id));
			xfer->ret = cmd->error;
			setbits(base + DEVICE_CTRL, DEV_CTRL_RESUME);
#if 0
			while (readl(base + DEVICE_CTRL) & DEV_CTRL_RESUME);
#endif
		}

		if (cmd->rx_len && !cmd->error)
			aspeed_i3c_read_rx_fifo(obj, cmd->rx_buf, cmd->rx_len);
	}
#ifdef USE_OS_FLAG_FOR_WAIT
	writel(INTR_MASTER_MASK, base + INTR_SIGNAL_EN);
#endif
}

static int aspeed_i3c_ccc_set(struct i3c_s *obj, struct i3c_ccc_cmd *ccc)
{
	i3c_xfer_t xfer;
	i3c_cmd_t *cmd;
	int ret, pos = 0;

	if (ccc->id & I3C_CCC_DIRECT) {
		pos = master_get_addr_pos(obj, ccc->dests[0].addr);
		if (pos < 0)
			return pos;
    }

	xfer.ncmds = 1;

    cmd = xfer.cmds;
    cmd->tx_buf = ccc->dests[0].payload.data;
    cmd->tx_len = ccc->dests[0].payload.len;

    cmd->cmd_hi = COMMAND_PORT_ARG_DATA_LEN(ccc->dests[0].payload.len) |
		  COMMAND_PORT_TRANSFER_ARG;

    cmd->cmd_lo = COMMAND_PORT_CP | COMMAND_PORT_DEV_INDEX(pos) |
		  COMMAND_PORT_CMD(ccc->id) | COMMAND_PORT_TOC |
		  COMMAND_PORT_ROC;

    log_trace("cmd_hi=0x%08x cmd_lo=0x%08x tx_len=%d\n", cmd->cmd_hi, cmd->cmd_lo,
	  cmd->tx_len);

    aspeed_i3c_start_xfer(obj, &xfer);
	/* wait for a second */
	xfer.ret = HAL_TIMEOUT;
#ifdef USE_OS_FLAG_FOR_WAIT
	osEventFlagsWait(evt_id, DEV_ID_TO_EVENT_FLAG(obj->device->dev_id),
			 osFlagsWaitAny, osWaitForever);
#else
	uint8_t count = 0;
	while (1) {
		if (readl(obj->device->base + INTR_STATUS))
			break;
		
		aspeed_wait_ms(1);
		count++;
		if (count > 100) {
			log_error("CCC time out\n");
			break;
		}
	}
#endif	

	aspeed_i3c_end_xfer(obj, &xfer);

	ret = xfer.ret;

    return ret;
}

static int aspeed_i3c_ccc_get(struct i3c_s *obj, struct i3c_ccc_cmd *ccc)
{
	i3c_xfer_t xfer;
	i3c_cmd_t *cmd;
	int ret = 0, pos;

	pos = master_get_addr_pos(obj, ccc->dests[0].addr);
	if (pos < 0)
		return pos;

	cmd = xfer.cmds;
	cmd->rx_buf = ccc->dests[0].payload.data;
	cmd->rx_len = ccc->dests[0].payload.len;

	cmd->cmd_hi = COMMAND_PORT_ARG_DATA_LEN(ccc->dests[0].payload.len) |
		      COMMAND_PORT_TRANSFER_ARG;

	cmd->cmd_lo = COMMAND_PORT_READ_TRANSFER |
		      COMMAND_PORT_CP |
		      COMMAND_PORT_DEV_INDEX(pos) |
		      COMMAND_PORT_CMD(ccc->id) |
		      COMMAND_PORT_TOC |
		      COMMAND_PORT_ROC;

	log_trace("cmd_hi=0x%08x cmd_lo=0x%08x rx_len=%d\n", cmd->cmd_hi, cmd->cmd_lo, cmd->rx_len);
#if 0
	dw_i3c_master_enqueue_xfer(master, xfer);
	if (!wait_for_completion_timeout(&xfer->comp, XFER_TIMEOUT))
		dw_i3c_master_dequeue_xfer(master, xfer);

	ret = xfer->ret;
	if (xfer->cmds[0].error == RESPONSE_ERROR_IBA_NACK)
		ccc->err = I3C_ERROR_M2;
	dw_i3c_master_free_xfer(xfer);
#endif
	return ret;
}

static int aspeed_i3c_send_ccc_cmd(struct i3c_s *obj, struct i3c_ccc_cmd *ccc)
{
	int ret = 0;

	if (ccc->rnw)
		ret = aspeed_i3c_ccc_get(obj, ccc);
	else
		ret = aspeed_i3c_ccc_set(obj, ccc);

	return ret;
}

static int aspeed_i3c_send_rstdaa(struct i3c_s *obj, uint8_t addr)
{
	struct i3c_ccc_cmd_dest dest;
	struct i3c_ccc_cmd cmd;

	log_trace("issue RSTDAA CCC\n");

	dest.addr = addr;
	dest.payload.len = 0;
	dest.payload.data = NULL;

	cmd.rnw = 0;
	cmd.id = I3C_CCC_RSTDAA(addr == I3C_BROADCAST_ADDR);
	cmd.dests = &dest;
	cmd.ndests = 1;
	cmd.err = I3C_ERROR_UNKNOWN;

	aspeed_i3c_send_ccc_cmd(obj, &cmd);
	return 0;
}

/**
 * @brief send SETAASA CCC
*/
hal_status_t aspeed_i3c_send_setaasa(struct i3c_s *obj)
{
	struct i3c_ccc_cmd_dest dest;
	struct i3c_ccc_cmd cmd;

	log_trace("issue AASA CCC\n");

	dest.addr = 0;
	dest.payload.len = 0;
	dest.payload.data = NULL;

	cmd.rnw = 0;
	cmd.id = I3C_CCC_SETAASA;
	cmd.dests = &dest;
	cmd.ndests = 1;
	cmd.err = I3C_ERROR_UNKNOWN;

	aspeed_i3c_send_ccc_cmd(obj, &cmd);
	return HAL_OK;
}

/**
 * @brief send ENEC CCC
*/
hal_status_t aspeed_i3c_send_enec(struct i3c_s *obj, uint8_t addr, uint8_t evt)
{
	struct i3c_ccc_cmd_dest dest;
	struct i3c_ccc_cmd cmd;
	uint8_t event = evt;

	log_trace("issue ENEC CCC\n");

	dest.addr = addr;
	dest.payload.len = 1;
	dest.payload.data = &event;

	cmd.rnw = 0;
	cmd.id = I3C_CCC_ENEC(addr);
	cmd.dests = &dest;
	cmd.ndests = 1;
	cmd.err = I3C_ERROR_UNKNOWN;

	aspeed_i3c_send_ccc_cmd(obj, &cmd);
	return HAL_OK;
}

hal_status_t aspeed_i3c_send_setmrl(struct i3c_s *obj, uint8_t addr, uint16_t mrl, uint8_t ibi_payload_size)
{
	struct i3c_ccc_cmd_dest dest;
	struct i3c_ccc_cmd cmd;
	uint8_t payload[4];

	log_trace("issue SETMRL CCC: mrl=%d, ibi payload=%d\n", mrl, ibi_payload_size);
	payload[0] = mrl >> 8;
	payload[1] = mrl & 0xff;
	payload[2] = ibi_payload_size & 0xff;

	dest.addr = addr;
	dest.payload.len = (ibi_payload_size) ? 3 : 2;
	dest.payload.data = payload;

	cmd.rnw = 0;
	cmd.id = I3C_CCC_SETMRL(addr);
	cmd.dests = &dest;
	cmd.ndests = 1;
	cmd.err = I3C_ERROR_UNKNOWN;

	aspeed_i3c_send_ccc_cmd(obj, &cmd);
	return HAL_OK;
}

hal_status_t aspeed_i3c_send_getmrl(struct i3c_s *obj, uint8_t addr, uint8_t ibi_en, uint8_t *mrl)
{
	struct i3c_ccc_cmd_dest dest;
	struct i3c_ccc_cmd cmd;
	uint8_t payload[4];

	log_trace("issue GETMRL CCC\n");

	dest.addr = addr;
	dest.payload.len = 2;
	if (ibi_en)
		dest.payload.len++;
	dest.payload.data = payload;

	cmd.rnw = 1;
	cmd.id = I3C_CCC_GETMRL;
	cmd.dests = &dest;
	cmd.ndests = 1;
	cmd.err = I3C_ERROR_UNKNOWN;

	aspeed_i3c_send_ccc_cmd(obj, &cmd);
	memcpy(mrl, payload, dest.payload.len);

	return HAL_OK;
}

static hal_status_t aspeed_i3c_clk_init(struct i3c_s *obj)
{
	aspeed_device_t *device = obj->device;
	aspeed_i3c_priv_t *priv = (aspeed_i3c_priv_t *)device->private;
	uint32_t clk_period = priv->clk_period;
	uint32_t scl_period_h, scl_period_l, scl_timing; 
	uint8_t hcnt, lcnt;

	/* init clock and timing */
	log_debug("core clk period = %d ns\n", clk_period);

	if (obj->bus_context) {
		/* OP mode: I3C SCL = 12.5MHz */
		scl_period_h = scl_period_l = SCL_OP_MODE_PERIOD_NS >> 1;

		if (scl_period_h < I3C_BUS_OP_THIGH_MIN_NS)
			scl_period_h = I3C_BUS_OP_THIGH_MIN_NS;
		if (scl_period_l < I3C_BUS_OP_TLOW_MIN_NS)
			scl_period_l = I3C_BUS_OP_TLOW_MIN_NS;
		
		hcnt = DIV_ROUND_UP(scl_period_h, clk_period);
		lcnt = DIV_ROUND_UP(scl_period_l, clk_period);
		scl_timing = SCL_I3C_TIMING_HCNT(hcnt) | SCL_I3C_TIMING_LCNT(lcnt);
		writel(scl_timing, device->base + SCL_I3C_OD_TIMING);
		
		scl_timing = SCL_I2C_FM_TIMING_HCNT(hcnt) | SCL_I2C_FM_TIMING_LCNT(lcnt);
		writel(scl_timing, device->base + SCL_I2C_FM_TIMING);
		scl_timing = SCL_I2C_FMP_TIMING_HCNT(hcnt) | SCL_I2C_FMP_TIMING_LCNT(lcnt);
		writel(scl_timing, device->base + SCL_I2C_FMP_TIMING);

		/* PP mode: I3C SCL = 12.5MHz */
		if (obj->i2c_mode)
			scl_period_h = scl_period_l = SCL_OP_MODE_PERIOD_NS >> 1;
		else
			scl_period_h = scl_period_l = SCL_PP_MODE_PERIOD_NS >> 1;

		if (scl_period_h < I3C_BUS_PP_THIGH_MIN_NS)
			scl_period_h = I3C_BUS_PP_THIGH_MIN_NS;
		if (scl_period_l < I3C_BUS_PP_TLOW_MIN_NS)
			scl_period_l = I3C_BUS_PP_TLOW_MIN_NS;
		hcnt = DIV_ROUND_UP(scl_period_h, clk_period);
		lcnt = DIV_ROUND_UP(scl_period_l, clk_period);
		scl_timing = SCL_I3C_TIMING_HCNT(hcnt) | SCL_I3C_TIMING_LCNT(lcnt);
		writel(scl_timing, device->base + SCL_I3C_PP_TIMING);
 	} else {
		 log_warn("not support non-JEDEC device yet\n");
	}
#if 0
	/* not necessary to set these regs. */
	lcnt = DIV_ROUND_UP(core_rate, I3C_BUS_SDR1_SCL_RATE) - hcnt;
	scl_timing = SCL_EXT_LCNT_1(lcnt);
	lcnt = DIV_ROUND_UP(core_rate, I3C_BUS_SDR2_SCL_RATE) - hcnt;
	scl_timing |= SCL_EXT_LCNT_2(lcnt);
	lcnt = DIV_ROUND_UP(core_rate, I3C_BUS_SDR3_SCL_RATE) - hcnt;
	scl_timing |= SCL_EXT_LCNT_3(lcnt);
	lcnt = DIV_ROUND_UP(core_rate, I3C_BUS_SDR4_SCL_RATE) - hcnt;
	scl_timing |= SCL_EXT_LCNT_4(lcnt);
	writel(scl_timing, device->base + SCL_EXT_LCNT_TIMING);
#endif
	return HAL_OK;
}


static void aspeed_i3c_attach_i3cdev(struct i3c_s *obj)
{
    int idx = 0;
	int n_i3cdev = 0;
    i3c_slave_t *slave = obj->slaves;

    for (idx = 0; idx < obj->n_slaves; idx++) {
		obj->free_pos &= ~BIT(idx);
		obj->addrs[idx] = slave->static_addr;
		slave->dynamic_addr = slave->assign_dynamic_addr;

		if (slave->i2c_mode) {
		    writel(DEV_ADDR_TABLE_LEGACY_I2C_DEV |
				   DEV_ADDR_TABLE_STATIC_ADDR(slave->static_addr),
			   	   obj->device->base + DEV_ADDR_TABLE_LOC(obj->datstartaddr, idx));
		} else {
			uint32_t value = DEV_ADDR_TABLE_DYNAMIC_ADDR(slave->assign_dynamic_addr);
#ifdef CONFIG_I3C_IBI
			value |= DEV_ADDR_TABLE_IBI_WITH_DATA;
#endif
		    writel(value, obj->device->base + DEV_ADDR_TABLE_LOC(obj->datstartaddr, idx));
			n_i3cdev++;
		}

		slave++;   
    }

	obj->i2c_mode = !n_i3cdev;
}

/**
 * @param [IN] role: DEVICE_CTRL_OP_MODE_MASTER or DEVICE_CTRL_OP_MODE_SLAVE
*/
static inline void aspeed_i3c_set_role(struct i3c_s *obj, uint8_t role)
{
	clrsetbits(obj->device->base + DEVICE_CTRL_EXTENDED, GENMASK(1, 0), role);
}

static inline void aspeed_i3c_enable(struct i3c_s *obj)
{
#ifdef CONFIG_I3C_IBI
	setbits(obj->device->base + DEVICE_CTRL, DEV_CTRL_ENABLE | DEV_CRTL_IBI_PAYLOAD_EN);
#else
	setbits(obj->device->base + DEVICE_CTRL, DEV_CTRL_ENABLE);
#endif
}

static inline void aspeed_i3c_dma_enable(struct i3c_s *obj)
{
	setbits(obj->device->base + DEVICE_CTRL, DEV_CTRL_DMA_EN);
}

/**
 *	@brief	init bus clock, HW queue threshold and master address
*/
static hal_status_t aspeed_i3c_bus_init(struct i3c_s *obj) 
{
	aspeed_device_t *device = obj->device;
	uint32_t thld_ctrl;

	aspeed_i3c_set_role(obj, obj->role);


	thld_ctrl = readl(device->base + QUEUE_THLD_CTRL);
	thld_ctrl &= ~QUEUE_THLD_CTRL_RESP_BUF_MASK;
	writel(thld_ctrl, device->base + QUEUE_THLD_CTRL);

	thld_ctrl = readl(device->base + DATA_BUFFER_THLD_CTRL);
	thld_ctrl &= ~DATA_BUFFER_THLD_CTRL_RX_BUF;
	writel(thld_ctrl, device->base + DATA_BUFFER_THLD_CTRL);

	writel(INTR_ALL, device->base + INTR_STATUS);

	if (obj->role) {
		writel(INTR_SLAVE_MASK, device->base + INTR_STATUS_EN);
		writel(INTR_SLAVE_MASK, device->base + INTR_SIGNAL_EN);
		/* init slave static address */
		writel(DEV_ADDR_STATIC_ADDR_VALID | DEV_ADDR_STATIC(obj->self_addr),
			   device->base + DEVICE_ADDR);
	} else {
		writel(INTR_MASTER_MASK, device->base + INTR_STATUS_EN);
		writel(INTR_MASTER_MASK, device->base + INTR_SIGNAL_EN);
		/* init master address */
		writel(DEV_ADDR_DYNAMIC_ADDR_VALID | DEV_ADDR_DYNAMIC(obj->self_addr),
			   device->base + DEVICE_ADDR);

		/* attach slave devices into DAT */
		aspeed_i3c_attach_i3cdev(obj);
	}

#ifdef CONFIG_I3C_IBI
	thld_ctrl = readl(device->base + QUEUE_THLD_CTRL);
	thld_ctrl &= ~(QUEUE_THLD_CTRL_IBI_STA_MASK | QUEUE_THLD_CTRL_IBI_DAT_MASK);
	thld_ctrl |= QUEUE_THLD_CTRL_IBI_STA(1);
	thld_ctrl |= QUEUE_THLD_CTRL_IBI_DAT(CONFIG_I3C_IBI_MAX_PAYLOAD >> 2);
	writel(thld_ctrl, device->base + QUEUE_THLD_CTRL);

	writel(0, device->base + IBI_SIR_REQ_REJECT);
	writel(0, device->base + IBI_MR_REQ_REJECT);
#endif

	aspeed_i3c_clk_init(obj);
	return HAL_OK;
}

static uint8_t even_parity(uint8_t p)
{
	p ^= p >> 4;
	p &= 0xf;
	return (0x9669 >> p) & 1;
}

#ifdef USE_OS_FLAG_FOR_WAIT
static void aspeed_i3c_mqueue_receive(i3c_t *obj, uint32_t nbytes, uint32_t reg)
{
	i3c_msg_t *msg;
	uint32_t base = obj->device->base;
	uint32_t j, nwords;
	uint32_t *ptr;

	nwords = CEIL_BYTES_TO_DWORDS(nbytes);
	log_trace("ibi: nbytes:%d, nwords:%d\n", nbytes, nwords);

	msg = &obj->mq.msgs[obj->mq.ptr];
	ptr = (uint32_t *)msg->buf;

	for (j = 0; j < nwords; j++) {
		*ptr++ = readl(base + reg);
	}

	msg->len = nbytes;
	osMessageQueuePut(obj->mq.id, msg, 0, 0);
	obj->mq.ptr++;
	obj->mq.ptr &= obj->mq.entries - 1;
}

static void aspeed_i3c_ibi_receive(struct i3c_s *obj)
{
	uint32_t base = obj->device->base;
	uint32_t i, nstatus, nbytes;

	nstatus = readl(base + QUEUE_STATUS_LEVEL);
	nstatus = QUEUE_STATUS_IBI_STATUS_CNT(nstatus);
	log_trace("ibi: nstatus = %d\n", nstatus);
	if (nstatus) {
		for (i = 0; i < nstatus; i++) {
			nbytes = readl(base + IBI_QUEUE_DATA) & GENMASK(7, 0);
			aspeed_i3c_mqueue_receive(obj, nbytes, IBI_QUEUE_DATA);
		}
		writel(RESET_CTRL_IBI_QUEUE, base + RESET_CTRL);
		writel(INTR_IBI_THLD_STAT, base + INTR_STATUS);
	}
}

void aspeed_i3c_isr_common(struct i3c_s *obj)
{
	uint32_t base = obj->device->base;
	uint32_t index = DEV_ID_TO_I3C_INDEX(obj->device->dev_id);
	uint32_t status, ret = 0;

	status = readl(base + INTR_STATUS);
	log_debug("i3c%d isr: status=%08x\n", index, status);

	if (status & INTR_IBI_THLD_STAT) {
		aspeed_i3c_ibi_receive(obj);
	}

	if (status & INTR_RESP_READY_STAT) {
		/* disable interrupt source and process the RX data in task mode */
		ret = readl(base + INTR_SIGNAL_EN);
		ret &= ~INTR_RESP_READY_STAT;
		writel(ret, base + INTR_SIGNAL_EN);
	}

	if (status & INTR_TRANSFER_ERR_STAT) {
		ret = readl(base + INTR_SIGNAL_EN);
		ret &= ~INTR_TRANSFER_ERR_STAT;
		writel(ret, base + INTR_SIGNAL_EN);
		writel(INTR_TRANSFER_ERR_STAT, base + INTR_STATUS);
		log_error("i3c%d isr: transfer error\n", index);
	} 
	
	if (status & INTR_TRANSFER_ABORT_STAT) {
		log_error("i3c%d isr: transfer abort\n", index);
		writel(INTR_TRANSFER_ABORT_STAT, base + INTR_STATUS);	
	}

	ret = osEventFlagsSet(evt_id, DEV_ID_TO_EVENT_FLAG(obj->device->dev_id));
	if (ret < 0)
		log_error("i3c%d isr: set evt flag fail: %d\n", index, ret);
}

void aspeed_i3c_isr(void)
{
	uint32_t n_irq = aspeed_irq_get_current_irq();
	struct i3c_s *obj = (struct i3c_s *)aspeed_irq_get_isr_context(n_irq);

	aspeed_i3c_isr_common(obj);	
}
#endif

static void aspeed_i3c_slave_isr(void)
{
	uint32_t n_irq = aspeed_irq_get_current_irq();
	struct i3c_s *obj = (struct i3c_s *)aspeed_irq_get_isr_context(n_irq);

	uint32_t base = obj->device->base;
	uint32_t index = DEV_ID_TO_I3C_INDEX(obj->device->dev_id);
	uint32_t i, status;

	status = readl(base + INTR_STATUS);
	log_debug("i3c%d isr: status=%08x\n", index, status);

	if (status & INTR_CCC_UPDATED_STAT) {
		uint32_t event = readl(base + SLV_EVENT_CTRL);
		uint32_t cm_state = PRESENT_STATE_CM_ST_STS(readl(base + PRESENT_STATE));

		if (CM_ST_STS_HALT == cm_state) {
			log_debug("i3c%d isr: slave in halt state\n", index);
			setbits(base + DEVICE_CTRL, DEV_CTRL_RESUME);
		}

		log_debug("i3c%d isr: slave event=%08x\n", index, event);
		if (event & SLV_EVENT_CTRL_MRL_UPD)
			log_debug("i3c%d isr: master set mrl=%d\n", index, readl(base + SLV_MAX_LEN) >> 16);

		if (event & SLV_EVENT_CTRL_MWL_UPD)
			log_debug("i3c%d isr: master set mwl=%d\n", index, readl(base + SLV_MAX_LEN) & GENMASK(15, 0));

		writel(event, base + SLV_EVENT_CTRL);

	}

	if (status & INTR_RESP_READY_STAT) {
		uint32_t nresp = QUEUE_STATUS_LEVEL_RESP(readl(base + QUEUE_STATUS_LEVEL));
		uint32_t resp;

		for (i = 0; i < nresp; i++) {
			resp = readl(base + RESPONSE_QUEUE_PORT);
			log_trace("i3c%d isr: resp %08x\n", index, resp);

			if (RESPONSE_NO_ERROR == RESPONSE_PORT_ERR_STATUS(resp)) {
				if (resp & RESPONSE_PORT_SLAVE_RX_RSP)
					aspeed_i3c_mqueue_receive(obj, RESPONSE_PORT_DATA_LEN(resp), RX_TX_DATA_PORT);
			} else if (RESPONSE_ERROR_MST_EARLY_TERMINATION == RESPONSE_PORT_ERR_STATUS(resp)) {
				log_warn("flush %d bytes in i3c slave tx fifo\n", RESPONSE_PORT_DATA_LEN(resp));
				writel(RESET_CTRL_TX_FIFO, base + RESET_CTRL);
			}
		}
		i = osEventFlagsSet(evt_id, DEV_ID_TO_EVENT_FLAG(obj->device->dev_id));
	}

	if (status & INTR_TRANSFER_ERR_STAT) {
		goto slave_isr_resume;
	}

	goto slave_isr_end;

slave_isr_resume:
	setbits(base + DEVICE_CTRL, DEV_CTRL_RESUME);

slave_isr_end:
	writel(status, base + INTR_STATUS);
}

hal_status_t aspeed_i3c_init(struct i3c_s *obj)
{
	aspeed_device_t *i3c = obj->device;
	aspeed_i3c_priv_t *priv = (aspeed_i3c_priv_t *)i3c->private;
	uint32_t i3c_idx = DEV_ID_TO_I3C_INDEX(i3c->dev_id);
	uint32_t base = i3c->base;
	int ret;

	if (NULL == &obj->slaves) {
		log_warn("no I3C slave assigned\n");
	}

	if (i3c->init) {
		log_error("I3C%d is occupied\n", i3c_idx + 1);
		return HAL_BUSY;
	}

	aspeed_clk_enable(i3c);
	aspeed_reset_assert(i3c);
	aspeed_wait_ms(10);
	aspeed_reset_deassert(i3c);

	writel(0x3f, base + RESET_CTRL);
	while (readl(base + RESET_CTRL));

	/* clear all interrupt status */
	writel(INTR_ALL, base + INTR_STATUS);

	ret = readl(base + DEVICE_ADDR_TABLE_POINTER);
	obj->datstartaddr = ret;
	obj->maxdevs = priv->max_addr_entry;
	obj->free_pos = GENMASK(obj->maxdevs - 1, 0);

	if (!obj->role) {
		/* AST2600A1 CCC workaround */
		obj->free_pos &= ~BIT(obj->maxdevs - 1);
		ret = (even_parity(I3C_BROADCAST_ADDR) << 7) | I3C_BROADCAST_ADDR;
		obj->addrs[obj->maxdevs - 1] = ret;
		writel(DEV_ADDR_TABLE_DYNAMIC_ADDR(ret),
			   base + DEV_ADDR_TABLE_LOC(obj->datstartaddr, obj->maxdevs - 1));
	}

	i3c->init = 1;

#ifdef USE_OS_FLAG_FOR_WAIT
	if (!obj->role) {
		aspeed_irq_register(priv->irq, (uint32_t)aspeed_i3c_isr, obj);
	} else {
		aspeed_irq_register(priv->irq, (uint32_t)aspeed_i3c_slave_isr, obj);
		writel(readl(obj->device->base + DEVICE_CTRL) | DEV_CTRL_AUTO_HJ_DIS,
		   obj->device->base + DEVICE_CTRL);
	}
#endif

	aspeed_i3c_bus_init(obj);
	
	if (obj->tx_dma_en || obj->rx_dma_en)
		aspeed_i3c_dma_enable(obj);
	
	aspeed_i3c_enable(obj);

	/* discover slaves on the bus (master only) */
	if (!obj->role) {
		/* emit RSTDAA CCC: send twice for AST2600 I3C slave workaround */
		aspeed_i3c_send_rstdaa(obj, I3C_BROADCAST_ADDR);
		aspeed_i3c_send_rstdaa(obj, I3C_BROADCAST_ADDR);

		/* emit SETAASA CCC */
		if (obj->i2c_mode == 0) {
			aspeed_i3c_send_setaasa(obj);
#ifdef CONFIG_I3C_IBI
			aspeed_i3c_send_enec(obj, I3C_BROADCAST_ADDR, I3C_CCC_EVENT_SIR);
			aspeed_i3c_send_setmrl(obj, I3C_BROADCAST_ADDR, CONFIG_I3C_MRL, CONFIG_I3C_IBI_MAX_PAYLOAD);
			for (int i = 0; i < obj->n_slaves; i++) {
				aspeed_i3c_send_getmrl(obj, obj->slaves->dynamic_addr, 1, (uint8_t *)&ret);
				log_trace("slave[%d]: ret=%08x, read length=%d, ibi length=%d\n", i, ret, (ret & 0xff) << 8 | (ret & 0xff00) >> 8, ret >> 16);
			}
#else
			aspeed_i3c_send_setmrl(obj, I3C_BROADCAST_ADDR, CONFIG_I3C_MRL, 0);
			for (int i = 0; i < obj->n_slaves; i++) {
				aspeed_i3c_send_getmrl(obj, obj->slaves->dynamic_addr, 1, &ret);
				log_trace("slave[%d]: ret=%08x, read length=%d\n", i, ret, (ret & 0xff) << 8 | (ret & 0xff00) >> 8);
			}
#endif
		}
	}

	return HAL_OK;
}

hal_status_t aspeed_i3c_priv_xfer(struct i3c_s *obj, uint32_t slave_idx, i3c_usr_xfer_t *xfers, uint32_t nxfers)
{
	i3c_xfer_t xfer;
	i3c_cmd_t *cmd;
	int i;

	if (nxfers == 0)
		return HAL_OK;
	
	xfer.ncmds = nxfers;

	for (i = 0; i < nxfers; i++) {
		cmd = &xfer.cmds[i];
		cmd->cmd_hi = COMMAND_PORT_ARG_DATA_LEN(xfers[i].len) | COMMAND_PORT_TRANSFER_ARG;
		cmd->cmd_lo = COMMAND_PORT_SPEED(0);
		if (xfers[i].rnw) {
			/* read command */
			cmd->cmd_lo |= COMMAND_PORT_READ_TRANSFER;
			cmd->rx_buf = xfers[i].data.out;
			cmd->rx_len = xfers[i].len;
			cmd->tx_buf = NULL;
			cmd->tx_len = 0;
		} else {
			/* write command */
			cmd->tx_buf = xfers[i].data.in;
			cmd->tx_len = xfers[i].len;
			cmd->rx_buf = NULL;
			cmd->rx_len = 0;
		}
		
		cmd->cmd_lo |= COMMAND_PORT_TID(i) | COMMAND_PORT_DEV_INDEX(slave_idx) |
				       COMMAND_PORT_ROC | COMMAND_PORT_TOC;
		
		log_trace("[%s] cmd_hi=0x%08x, cmd_lo=0x%08x\n", xfers[i].rnw ? "rd" : "wr", cmd->cmd_hi, cmd->cmd_lo);
	}

	aspeed_i3c_start_xfer(obj, &xfer);
#ifdef USE_OS_FLAG_FOR_WAIT
	osEventFlagsWait(evt_id, DEV_ID_TO_EVENT_FLAG(obj->device->dev_id),
			 osFlagsWaitAny, osWaitForever);
#else		
	while (1) {
		if (readl(obj->device->base + INTR_STATUS))
			break;
		
		aspeed_wait_ms(1);
	}
#endif 
	aspeed_i3c_end_xfer(obj, &xfer);

	return xfer.ret;
}
/**
 * I2C read
 * [AST /]$ i2cget -f -y 16 0x51 0x0
* dw-i3c-master 1e7a2000.i3c0: dw_i3c_master_i2c_xfers:cmd_hi=0x00010001 cmd_lo=0x04000000 tx_len=1 rx_len=0
* dw-i3c-master 1e7a2000.i3c0: dw_i3c_master_i2c_xfers:cmd_hi=0x00010001 cmd_lo=0x54000008 tx_len=0 rx_len=1
* dw-i3c-master 1e7a2000.i3c0: TX data = 00000000
* 0x51
* [AST /]$ i2cget -f -y 16 0x51 0x1
* dw-i3c-master 1e7a2000.i3c0: dw_i3c_master_i2c_xfers:cmd_hi=0x00010001 cmd_lo=0x04000000 tx_len=1 rx_len=0
* dw-i3c-master 1e7a2000.i3c0: dw_i3c_master_i2c_xfers:cmd_hi=0x00010001 cmd_lo=0x54000008 tx_len=0 rx_len=1
* dw-i3c-master 1e7a2000.i3c0: TX data = 00000001
* 0x18
*/
hal_status_t aspeed_i3c_i2c_read(struct i3c_s *obj, uint32_t slave_idx, uint32_t addr, uint32_t length, uint8_t *data)
{
	i3c_xfer_t xfer;
	i3c_cmd_t *cmd;
	uint8_t tx_buf[2] = {addr, 0};

	if (length == 0)
		return HAL_ERROR;
	
	xfer.ncmds = 2;

	/* wr cmd */
	cmd = &xfer.cmds[0];
	cmd->cmd_hi = COMMAND_PORT_ARG_DATA_LEN(1) | COMMAND_PORT_TRANSFER_ARG;
	cmd->cmd_lo = COMMAND_PORT_TID(0) | COMMAND_PORT_DEV_INDEX(slave_idx) |
		       COMMAND_PORT_ROC;
	cmd->tx_buf = tx_buf;
	cmd->tx_len = 1;
	cmd->rx_buf = NULL;
	cmd->rx_len = 0;
	log_trace("cmd_hi=0x%08x, cmd_lo=0x%08x\n", cmd->cmd_hi, cmd->cmd_lo);

	/* rd cmd */
	cmd = &xfer.cmds[1];
	cmd->cmd_hi = COMMAND_PORT_ARG_DATA_LEN(length) | COMMAND_PORT_TRANSFER_ARG;
	cmd->cmd_lo = COMMAND_PORT_READ_TRANSFER | COMMAND_PORT_SPEED(0);
	cmd->cmd_lo |= COMMAND_PORT_TID(1) | COMMAND_PORT_DEV_INDEX(slave_idx) |
		       COMMAND_PORT_ROC | COMMAND_PORT_TOC;
	cmd->rx_buf = data;
	cmd->rx_len = length;
	cmd->tx_buf = NULL;
	cmd->tx_len = 0;

	log_trace("cmd_hi=0x%08x, cmd_lo=0x%08x\n", cmd->cmd_hi, cmd->cmd_lo);

	aspeed_i3c_start_xfer(obj, &xfer);

#ifdef USE_OS_FLAG_FOR_WAIT
	osEventFlagsWait(evt_id, DEV_ID_TO_EVENT_FLAG(obj->device->dev_id),
			 osFlagsWaitAny, osWaitForever);
#else	
	while (1) {
		if (readl(obj->device->base + INTR_STATUS))
			break;
		
		aspeed_wait_ms(1);
	}
#endif	
	aspeed_i3c_end_xfer(obj, &xfer);
	
	return HAL_OK;
}

/**
 * I2C read and write
 * [AST /]$ i2cset -f -y 16 0x51 64 0
 * dw-i3c-master 1e7a2000.i3c0: dw_i3c_master_i2c_xfers:cmd_hi=0x00020001 cmd_lo=0x44000000 tx_len=2 rx_len=0
 * dw-i3c-master 1e7a2000.i3c0: TX data = 00000040
 * [AST /]$ i2cget -f -y 16 0x51 64
 * dw-i3c-master 1e7a2000.i3c0: dw_i3c_master_i2c_xfers:cmd_hi=0x00010001 cmd_lo=0x04000000 tx_len=1 rx_len=0
 * dw-i3c-master 1e7a2000.i3c0: dw_i3c_master_i2c_xfers:cmd_hi=0x00010001 cmd_lo=0x54000008 tx_len=0 rx_len=1
 * dw-i3c-master 1e7a2000.i3c0: TX data = 00000040
 * 0x00
 * [AST /]$ i2cset -f -y 16 0x51 64 0x40
 * dw-i3c-master 1e7a2000.i3c0: dw_i3c_master_i2c_xfers:cmd_hi=0x00020001 cmd_lo=0x44000000 tx_len=2 rx_len=0
 * dw-i3c-master 1e7a2000.i3c0: TX data = 00004040
 * [AST /]$ i2cget -f -y 16 0x51 64
 * dw-i3c-master 1e7a2000.i3c0: dw_i3c_master_i2c_xfers:cmd_hi=0x00010001 cmd_lo=0x04000000 tx_len=1 rx_len=0
 * dw-i3c-master 1e7a2000.i3c0: dw_i3c_master_i2c_xfers:cmd_hi=0x00010001 cmd_lo=0x54000008 tx_len=0 rx_len=1
 * dw-i3c-master 1e7a2000.i3c0: TX data = 00000040
 * 0x40
*/
hal_status_t aspeed_i3c_i2c_write(struct i3c_s *obj, uint32_t slave_idx, uint32_t addr, uint32_t length, uint8_t *data)
{
	i3c_xfer_t xfer;
	i3c_cmd_t *cmd;
	uint8_t *tx_buf;

	if (length == 0)
		return HAL_ERROR;
	
	xfer.ncmds = 1;
	
	/* FIXME: is there a way to prevent copy? */
	tx_buf = pvPortMalloc(length + 2);
	memset(tx_buf, 0, length + 2);
	tx_buf[0] = addr;
	memcpy(&tx_buf[2], data, length);

	/* wr cmd */
	cmd = &xfer.cmds[0];
	cmd->cmd_hi = COMMAND_PORT_ARG_DATA_LEN(length + 2) | COMMAND_PORT_TRANSFER_ARG;
	cmd->cmd_lo = COMMAND_PORT_SPEED(0);
	cmd->cmd_lo |= COMMAND_PORT_TID(0) | COMMAND_PORT_DEV_INDEX(slave_idx) |
		       COMMAND_PORT_ROC | COMMAND_PORT_TOC;
	cmd->tx_buf = tx_buf;
	cmd->tx_len = 2 + length;
	cmd->rx_buf = NULL;
	cmd->rx_len = 0;
	log_trace("[wr cmd] cmd_hi=0x%08x, cmd_lo=0x%08x\n", cmd->cmd_hi, cmd->cmd_lo);

	aspeed_i3c_start_xfer(obj, &xfer);
#ifdef USE_OS_FLAG_FOR_WAIT
	osEventFlagsWait(evt_id, DEV_ID_TO_EVENT_FLAG(obj->device->dev_id),
			 osFlagsWaitAny, osWaitForever);
#else		
	while (1) {
		if (readl(obj->device->base + INTR_STATUS))
			break;
		
		aspeed_wait_ms(1);
	}
#endif	
	aspeed_i3c_end_xfer(obj, &xfer);
	vPortFree(tx_buf);
	
	return HAL_OK;
}

/**
 * Slave APIs
 */
hal_status_t aspeed_i3c_slave_issue_sir(struct i3c_s *obj, uint8_t *data,
										uint32_t nbytes)
{
#ifdef CONFIG_I3C_IBI
	/**
	 * workarond for AST2600 & AST1030A0:
	 * the slave device can only send fixed length of IBI payload
	*/
	//aspeed_i3c_wr_tx_fifo(obj, data, nbytes);
	aspeed_i3c_wr_tx_fifo(obj, data, CONFIG_I3C_IBI_MAX_PAYLOAD - 1);
	writel(1, obj->device->base + SLV_INTR_REQ);

	osEventFlagsWait(evt_id, DEV_ID_TO_EVENT_FLAG(obj->device->dev_id),
			 osFlagsWaitAny, osWaitForever);
#endif
	return HAL_OK;
}

hal_status_t aspeed_i3c_slave_wr_resp(struct i3c_s *obj, uint8_t *data, uint32_t nbytes)
{
	uint32_t base = obj->device->base;

	if (nbytes > CONFIG_I3C_MRL)
		nbytes = CONFIG_I3C_MRL;

	clrsetbits(base + QUEUE_THLD_CTRL, QUEUE_THLD_CTRL_RESP_BUF_MASK, QUEUE_THLD_CTRL_RESP_BUF(1));

	aspeed_i3c_wr_tx_fifo(obj, data, nbytes);
	writel(nbytes << 16, obj->device->base + COMMAND_QUEUE_PORT);

	osEventFlagsWait(evt_id, DEV_ID_TO_EVENT_FLAG(obj->device->dev_id),
			 osFlagsWaitAny, osWaitForever);

	return HAL_OK;
}