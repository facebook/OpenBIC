/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _I3C_ASPEED_H_
#define _I3C_ASPEED_H_
#include "hal_def.h"
#include "objects.h"

#define ASPEED_I3C_TX_FIFO_SIZE		256
#define ASPEED_I3C_RX_FIFO_SIZE		256

typedef struct i3c_usr_xfer_s {
	union {
		uint8_t *in;
		uint8_t *out;
	} data;
	uint32_t len;
	uint32_t rnw;
} i3c_usr_xfer_t;

typedef struct aspeed_i3c_priv_s {
	/* clock period in nano-second */
	uint32_t clk_period;
	uint32_t max_addr_entry;

	/* irq index */
	uint32_t irq;
} aspeed_i3c_priv_t;

void aspeed_i3c_global_init(struct i3c_global_s *obj);
hal_status_t aspeed_i3c_init(struct i3c_s *obj);
hal_status_t aspeed_i3c_i2c_read(struct i3c_s *obj, uint32_t slave_idx, uint32_t addr, uint32_t length, uint8_t *data);
hal_status_t aspeed_i3c_i2c_write(struct i3c_s *obj, uint32_t slave_idx, uint32_t addr, uint32_t length, uint8_t *data);
hal_status_t aspeed_i3c_slave_issue_sir(struct i3c_s *obj, uint8_t *data, uint32_t nbytes);
hal_status_t aspeed_i3c_slave_wr_resp(struct i3c_s *obj, uint8_t *data, uint32_t nbytes);
hal_status_t aspeed_i3c_send_setaasa(struct i3c_s *obj);
hal_status_t aspeed_i3c_priv_xfer(struct i3c_s *obj, uint32_t slave_idx, i3c_usr_xfer_t *xfers, uint32_t nxfers);
#endif /* end of "ifndef _I3C_ASPEED_H_" */