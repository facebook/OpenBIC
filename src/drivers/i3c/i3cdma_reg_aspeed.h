/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdint.h>

/* ctl_lo_reg_s.b.dst_tr_width and src_tr_width */
#define TR_WIDTH_8_BITS					0x0	/*  1 byte -> 2^0 bytes */
#define TR_WIDTH_16_BITS				0x1	/*  2 byte -> 2^1 bytes */
#define TR_WIDTH_32_BITS				0x2	/*  4 byte -> 2^2 bytes */
#define TR_WIDTH_64_BITS				0x3	/*  8 byte -> 2^3 bytes */
#define TR_WIDTH_128_BITS				0x4	/* 16 byte -> 2^4 bytes */
#define TR_WIDTH_256_BITS				0x5	/* 32 byte -> 2^5 bytes */

/* ctl_lo_reg_s.b.dinc and sinc */
#define ADDR_INC						0x0
#define ADDR_DEC						0x1
#define ADDR_NOCHANGE					0x2

/* ctl_lo_reg_s.b.tt_fc */
#define TT_MEM_TO_DEV_FC_DMAC			0x1	// transfer type: memory to device, flow controller: DMAC
#define TT_DEV_TO_MEM_FC_DMAC			0x2	// transfer type: device to memory, flow controller: DMAC
/* --- */
#define TT_DEV_TO_MEM_FC_DEV			0x4 // transfer type: device to memory, flow controller: device
#define TT_MEM_TO_DEV_FC_DEV			0x6 // transfer type: memory to device, flow controller: device

typedef union ctl_lo_reg_s {
	volatile uint32_t w;
	struct {
		volatile uint32_t int_en		:1;		/* 0 */
		volatile uint32_t dst_tr_width	:3;		/* 3:1 */
		volatile uint32_t src_tr_width	:3;		/* 6:4 */
		volatile uint32_t dinc			:2;		/* 8:7 */
		volatile uint32_t sinc			:2;		/* 10:9 */
		volatile uint32_t dst_msize		:3;		/* 13:11 */
		volatile uint32_t src_msize		:3;		/* 16:14 */
		volatile uint32_t src_gather_en	:1;		/* 17 */
		volatile uint32_t dst_scatter_en:1;		/* 18 */
		volatile uint32_t rsvd_ctl		:1;		/* 19 */
		volatile uint32_t tt_fc			:3;		/* 22:20 */
		volatile uint32_t dms			:2;		/* 24:23 */
		volatile uint32_t sms			:2;		/* 26:25 */
		volatile uint32_t llp_dst_en	:1;		/* 27 */
		volatile uint32_t llp_src_en	:1;		/* 28 */
		volatile uint32_t rsvd1_ctl		:3;		/* 31:29 */
	} b;
} ctl_lo_reg_t;

typedef union ctl_hi_reg_s {
	volatile uint32_t w;
	struct {
		volatile uint32_t block_ts		:5;		/* 36:32 */
		volatile uint32_t rsvd0			:7;		/* 43:37 */
		volatile uint32_t done			:1;		/* 44 */
		volatile uint32_t rsvd1			:19;	/* 63:45 */
	} b;
} ctl_hi_reg_t;

typedef union cfg_lo_reg_s {
	volatile uint32_t w;
	struct {
		volatile uint32_t 				:5;		/* 4:0 */
		volatile uint32_t ch_prior		:3;		/* 7:5 */
		volatile uint32_t ch_susp		:1;		/* 8 */
		volatile uint32_t fifo_empty	:1;		/* 9 */
		volatile uint32_t hs_sel_dst	:1;		/* 10 */
		volatile uint32_t hs_sel_src	:1;		/* 11 */
		volatile uint32_t lock_ch_l		:2;		/* 13:12 */
		volatile uint32_t lock_b_l		:2;		/* 15:14 */
		volatile uint32_t lock_ch		:1;		/* 16 */
		volatile uint32_t lock_b		:1;		/* 17 */
		volatile uint32_t dst_hs_pol	:1;		/* 18 */
		volatile uint32_t src_hs_pol	:1;		/* 19 */
		volatile uint32_t max_abrst		:10;	/* 29:20 */
		volatile uint32_t reload_src	:1;		/* 30 */
		volatile uint32_t reload_dst	:1;		/* 31 */
	} b;
} cfg_lo_reg_t;

typedef union cfg_hi_reg_s {
	volatile uint32_t w;
	struct {
		volatile uint32_t fcmode		:1;		/* 32 */
		volatile uint32_t fifo_mode		:1;		/* 33 */
		volatile uint32_t protctl		:3;		/* 36:34 */
		volatile uint32_t ds_upd_en		:1;		/* 37 */
		volatile uint32_t ss_upd_en		:1;		/* 38 */
		volatile uint32_t src_per		:4;		/* 42:39 */
		volatile uint32_t dst_per		:4;		/* 46:43 */
		volatile uint32_t 				:17;	/* 63:47 */
	} b;
} cfg_hi_reg_t;

typedef struct chan_reg_s {
	volatile uint32_t sar;					/* offset 0x00 */
	volatile uint32_t sar_rsvd;
	volatile uint32_t dar;					/* offset 0x08 */
	volatile uint32_t dar_rsvd;
	volatile uint32_t llp;					/* offset 0x10 */
	volatile uint32_t llp_rsvd;
	volatile ctl_lo_reg_t ctl_lo;			/* offset 0x18 */
	volatile ctl_hi_reg_t ctl_hi;			/* offset 0x1c */
	volatile uint32_t sstat;				/* offset 0x20 */
	volatile uint32_t sstat_rsvd;
	volatile uint32_t dstat;				/* offset 0x28 */
	volatile uint32_t dstat_rsvd;
	volatile uint32_t sstatar;				/* offset 0x30 */
	volatile uint32_t sstatar_rsvd;
	volatile uint32_t dstatar;				/* offset 0x38 */
	volatile uint32_t dstatar_rsvd;
	volatile cfg_lo_reg_t cfg_lo;			/* offset 0x40 */
	volatile cfg_hi_reg_t cfg_hi;			/* offset 0x44 */
	volatile uint32_t sgr;					/* offset 0x48 */
	volatile uint32_t sgr_rsvd;
	volatile uint32_t dsr;					/* offset 0x50 */
	volatile uint32_t dsr_rsvd;
} chan_reg_t;

typedef struct intr_reg_s {
	volatile uint32_t tfr;
	volatile uint32_t tfr_rsvd;
	volatile uint32_t block;
	volatile uint32_t block_rsvd;
	volatile uint32_t srctran;
	volatile uint32_t srctran_rsvd;
	volatile uint32_t dsttran;
	volatile uint32_t dsttran_rsvd;
	volatile uint32_t err;
	volatile uint32_t err_rsvd;
} intr_reg_t;

typedef union dma_cfg_reg_s {
	volatile uint32_t w;
	struct {
		volatile uint32_t dma_en				:1;		/* 0 */
		volatile uint32_t						:31;	/* 31:1 */
	} b;
} dma_cfg_reg_t;


typedef struct aspeed_i3cdma_reg_s {
	/* channel registers */
	volatile chan_reg_t chan_reg[8];		/* 0x000 - 0x2b8 */
	/* interrupt registers */
	volatile intr_reg_t raw;				/* 0x2c0 - 0x2e4 */
	volatile intr_reg_t status;				/* 0x2e8 - 0x30c */
	volatile intr_reg_t mask;				/* 0x310 - 0x334 */
	volatile intr_reg_t clear;				/* 0x338 - 0x35c */
	volatile uint32_t status_int;			/* 0x360 */
	volatile uint32_t status_int_rsvd;
	/* software handshake registers */
	volatile uint32_t req_src;				/* 0x368 */
	volatile uint32_t req_src_rsvd;
	volatile uint32_t req_dst;				/* 0x370 */
	volatile uint32_t req_dst_rsvd;
	volatile uint32_t sgl_req_src;			/* 0x378 */
	volatile uint32_t sgl_req_src_rsvd;
	volatile uint32_t sgl_req_dst;			/* 0x380 */
	volatile uint32_t sgl_req_dst_rsvd;
	volatile uint32_t lst_req_src;			/* 0x388 */
	volatile uint32_t lst_req_src_rsvd;
	volatile uint32_t lst_req_dst;			/* 0x390 */
	volatile uint32_t lst_req_dst_rsvd;
	/* misc registers */
	volatile dma_cfg_reg_t dma_cfg;			/* 0x398 */
	volatile uint32_t dma_cfg_rsvd;
	volatile uint32_t ch_en;				/* 0x3a0 */
	volatile uint32_t ch_en_rsvd;
	volatile uint32_t dma_id;				/* 0x3a8 */
	volatile uint32_t dma_id_rsvd;
	volatile uint32_t dma_test;				/* 0x3b0 */
	volatile uint32_t dma_test_rsvd;
	volatile uint32_t dma_lpto;				/* 0x3b8 */
	volatile uint32_t dma_lpto_rsvd;		/* 0x3bc */

	volatile uint32_t __gap[2];				/* 0x3c0 - 0x3c4 */
	volatile uint32_t dma_comp_params_6_lo;	/* 0x3c8 */
	volatile uint32_t dma_comp_params_6_hi;	/* 0x3cc */
	volatile uint32_t dma_comp_params_5_lo;	/* 0x3d0 */
	volatile uint32_t dma_comp_params_5_hi;	/* 0x3d4 */
	volatile uint32_t dma_comp_params_4_lo;	/* 0x3d8 */
	volatile uint32_t dma_comp_params_4_hi;	/* 0x3dc */
	volatile uint32_t dma_comp_params_3_lo;	/* 0x3e0 */
	volatile uint32_t dma_comp_params_3_hi;	/* 0x3e4 */
	volatile uint32_t dma_comp_params_2_lo;	/* 0x3e8 */
	volatile uint32_t dma_comp_params_2_hi;	/* 0x3ec */
	volatile uint32_t dma_comp_params_1_lo;	/* 0x3f0 */
	volatile uint32_t dma_comp_params_1_hi;	/* 0x3f4 */
	volatile uint32_t dma_comp_id;			/* 0x3f8 */
	volatile uint32_t dma_comp_id_rsvd;
} aspeed_i3cdma_reg_t;