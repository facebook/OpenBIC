/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "clk_aspeed.h"
#include "common.h"
#include "flash.h"
#include "fmc_spi_aspeed.h"
#include "io.h"
#include "log.h"
#include <stdlib.h>
#include <string.h>
#include "wait.h"


#define OFFSET_INTR_CTRL_STATUS			0x08
#define OFFSET_CE0_CTRL_REG				0x10
#define OFFSET_DMA_CTRL					0x80
#define OFFSET_DMA_FLASH_ADDR_REG		0x84
#define OFFSET_DMA_RAM_ADDR_REG			0x88
#define OFFSET_DMA_LEN_REG				0x8c
#define OFFSET_DMA_CHECKSUM_RESULT		0x90
#define OFFSET_CE0_TIMING_COMPENSATION	0x94

#define CALIBRATION_LEN		0x400
#define SPI_DMA_IRQ_EN		BIT(3)
#define SPI_DAM_REQUEST		BIT(31)
#define SPI_DAM_GRANT		BIT(30)
#define SPI_DMA_CALIB_MODE	BIT(3)
#define SPI_DMA_CALC_CKSUM	BIT(2)
#define SPI_DMA_ENABLE		BIT(0)
#define SPI_DMA_STATUS		BIT(11)
#define DMA_GET_REQ_MAGIC		0xaeed0000
#define DMA_DISCARD_REQ_MAGIC	0xdeea0000

uint32_t ast2600_segment_addr_start(uint32_t reg_val)
{
	return ((reg_val & 0x0ff0) << 16);
}

uint32_t ast2600_segment_addr_end(uint32_t reg_val)
{
	return ((reg_val & 0x0ff00000) | 0x000fffff);
}

uint32_t ast2600_segment_addr_val(uint32_t start, uint32_t end)
{
	return ((((((start) >> 20) << 20) >> 16) & 0xffff) | ((((end) >> 20) << 20) & 0xffff0000));
}

uint32_t ast1030_fmc_segment_addr_start(uint32_t reg_val)
{
	return ((reg_val & 0x0ff8) << 16);
}

uint32_t ast1030_fmc_segment_addr_end(uint32_t reg_val)
{
	return ((reg_val & 0x0ff80000) | 0x0007ffff);
}

uint32_t ast1030_fmc_segment_addr_val(uint32_t start, uint32_t end)
{
	return ((((((start) >> 19) << 19) >> 16) & 0xfff8) | ((((end) >> 19) << 19) & 0xfff80000));
}

uint32_t ast1030_spi_segment_addr_start(uint32_t reg_val)
{
	return ((reg_val & 0x0ff0) << 16);
}

uint32_t ast1030_spi_segment_addr_end(uint32_t reg_val)
{
	return ((reg_val & 0x0ff00000) | 0x000fffff);
}

uint32_t ast1030_spi_segment_addr_val(uint32_t start, uint32_t end)
{
	return ((((((start) >> 20) << 20) >> 16) & 0xffff) | ((((end) >> 20) << 20) & 0xffff0000));
}

uint8_t aspeed_spi_get_io_mode(uint32_t bus_width)
{
	switch (bus_width) {
		case 1:
			return CTRL_IO_SINGLE_DATA;
		case 2:
			return CTRL_IO_DUAL_DATA;
		case 4:
			return CTRL_IO_QUAD_DATA;
		default:
			return CTRL_IO_SINGLE_DATA;
	}
}

static void aspeed_spi_write_enable(flash_t *flash)
{
	fmc_spi_ctrl_t *ctrl_reg = (fmc_spi_ctrl_t *)flash->spi_controller->ctrl_base;

	if (ctrl_reg->spi_ce_type_setting_reg.fields.ce_write_type & BIT(flash->spi_chip->cs))
		return;
	else
		ctrl_reg->spi_ce_type_setting_reg.fields.ce_write_type |= BIT(flash->spi_chip->cs);

	return;
}

static void aspeed_spi_set_io_mode(flash_t *flash, uint32_t io_mode)
{
	fmc_spi_ctrl_t *ctrl_reg = (fmc_spi_ctrl_t *)flash->spi_controller->ctrl_base;

	ctrl_reg->spi_ce_ctrl_reg[flash->spi_chip->cs].fields.io_mode = io_mode;

	return;
}

void aspeed_spi_set_4byte_mode(flash_t *flash)
{
	fmc_spi_ctrl_t *ctrl_reg = (fmc_spi_ctrl_t *)flash->spi_controller->ctrl_base;
	ctrl_reg->spi_addr_mode_ctrl_reg.fields.addr_mode |= 1 << flash->spi_chip->cs;
	ctrl_reg->spi_addr_mode_ctrl_reg.fields.ce_4b_auto_read_cmd |= 1 << flash->spi_chip->cs;

	return;
}

void aspeed_spi_write_data(uint32_t ahb_addr,
		uint32_t write_cnt, uint8_t *write_arr)
{
	int i;
	uint32_t dword;

	if (write_arr) {
#if defined(DEBUG)
		for (i = 0; i < write_cnt; i++)
			log_debug("[%02x]", write_arr[i]);
		log_debug("\n");
#endif
		for (i = 0; i < write_cnt; i += 4) {
			if ((write_cnt - i) < 4)
				break;
			dword = write_arr[i];
			dword |= write_arr[i + 1] << 8;
			dword |= write_arr[i + 2] << 16;
			dword |= write_arr[i + 3] << 24;
			writel(dword, ahb_addr);
		}

		for (; i < write_cnt; i++)
			writeb(write_arr[i], ahb_addr);
	}

	return;
}

void aspeed_spi_read_data(uint32_t ahb_addr,
		uint32_t read_cnt, uint8_t *read_arr)
{
	int i = 0;
	uint32_t dword;
	uint32_t *read_ptr = (uint32_t *)read_arr;

	if (read_arr) {
		if (((uint32_t)read_ptr & 0xf) == 0) {
			for (i = 0; i < read_cnt; i += 4) {
				if (read_cnt - i < 4)
					break;
				*read_ptr = readl(ahb_addr);
				read_ptr += 1;
			}
		}

		for (; i < read_cnt;) {
			dword = readl(ahb_addr);
			if (i < read_cnt)
				read_arr[i] = dword & 0xff;
			i++;
			if (i < read_cnt)
				read_arr[i] = (dword >> 8) & 0xff;
			i++;
			if (i < read_cnt)
				read_arr[i] = (dword >> 16) & 0xff;
			i++;
			if (i < read_cnt)
				read_arr[i] = (dword >> 24) & 0xff;
			i++;
		}
#if defined(DEBUG)
		for (i = 0; i < read_cnt; i++)
			log_debug("[%02x]", read_arr[i]);
		log_debug("\n");
#endif
	}

	return;
}

uint32_t aspeed_spi_xfer(spi_t *spi, uint32_t cs,
		uint32_t read_cnt, uint32_t write_cnt,
		uint8_t *read_arr, uint8_t *write_arr)
{
	fmc_spi_ctrl_t *ctrl_reg = (fmc_spi_ctrl_t *)spi->device->base;
	fmc_spi_priv_t *priv = (fmc_spi_priv_t *)spi->device->private;

	uint32_t ahb_addr = priv->chipes[cs].ahb_start_addr;
	uint32_t ori_ctrl_val = ctrl_reg->spi_ce_ctrl_reg[cs].val;

	log_debug("%s, cmd=0x%02x, writecnt=%d, readcnt=%d\n",
			__func__, *write_arr, write_cnt, read_cnt);
#if 0
	if (priv->flash_component) {
		flash_t *flash = priv->chipes[cs].ctx;
		/* for flash component, always send cmd + address + dummy by single i/o mode */
		ctrl_reg->spi_ce_ctrl_reg[cs].fields.io_mode = \
				aspeed_spi_get_io_mode(SNOR_PROTO_1_1_1);
		ctrl_reg->spi_ce_ctrl_reg[cs].fields.spi_cmd_mode = SPI_USER_MODE;
		ctrl_reg->spi_ce_ctrl_reg[cs].fields.cs_inactive_ctrl = 0;

		if (flash->cmd_addr_buf)
			aspeed_spi_write_data(ahb_addr, flash->cmd_addr_len, flash->cmd_addr_buf);

		if (write_cnt !=0 && write_arr != NULL) {
			/* we take write status cmd 01h into consideration.
			 * when read_cnt is smaller than 4 bytes, use 1-1-1 format.
			 */
			if (4 < write_cnt) {
				ctrl_reg->spi_ce_ctrl_reg[cs].fields.io_mode = \
					aspeed_spi_get_io_mode(priv->chipes[cs].tx_bus_width);
			}
			aspeed_spi_write_data(ahb_addr, write_cnt, write_arr);
		}

		if (read_cnt != 0 && read_arr != NULL) {
			/* we take read status cmd 05h into consideration.
			 * when read_cnt is smaller than 4 bytes, use 1-1-1 format.
			 */
			if (4 < read_cnt) {
				ctrl_reg->spi_ce_ctrl_reg[cs].fields.io_mode = \
					aspeed_spi_get_io_mode(priv->chipes[cs].rx_bus_width);
			}
			aspeed_spi_read_data(ahb_addr, read_cnt, read_arr);
		}

		aspeed_wait_us(5);
	}
#endif
	/* enter user mode */
	ctrl_reg->spi_ce_ctrl_reg[cs].fields.io_mode = \
		aspeed_spi_get_io_mode(priv->chipes[cs].tx_bus_width);
	ctrl_reg->spi_ce_ctrl_reg[cs].fields.spi_cmd_mode = CTRL_CMD_MODE_USER;
	ctrl_reg->spi_ce_ctrl_reg[cs].fields.cs_inactive_ctrl = 0;

	/* transfer data */
	aspeed_spi_write_data(ahb_addr, write_cnt, write_arr);
	aspeed_wait_us(5);

	ctrl_reg->spi_ce_ctrl_reg[cs].fields.io_mode = \
		aspeed_spi_get_io_mode(priv->chipes[cs].rx_bus_width);

	aspeed_spi_read_data(ahb_addr, read_cnt, read_arr);


	/* disable user mode */
	ctrl_reg->spi_ce_ctrl_reg[cs].fields.cs_inactive_ctrl = 1;
	ctrl_reg->spi_ce_ctrl_reg[cs].val = ori_ctrl_val;
	return 0;
}

void aspeed_spi_start_user(flash_t *flash)
{
	fmc_spi_ctrl_t *ctrl_reg = (fmc_spi_ctrl_t *)flash->spi_controller->ctrl_base;

	/*
	 * When the chip is controlled in user mode, we need write
	 * access to send the opcodes to it. So check the config.
	 */
	aspeed_spi_write_enable(flash);

	ctrl_reg->spi_ce_ctrl_reg[flash->spi_chip->cs].fields.spi_cmd_mode = CTRL_CMD_MODE_USER;
	ctrl_reg->spi_ce_ctrl_reg[flash->spi_chip->cs].fields.cs_inactive_ctrl = 1;
	ctrl_reg->spi_ce_ctrl_reg[flash->spi_chip->cs].fields.io_mode = CTRL_IO_SINGLE_DATA;
	ctrl_reg->spi_ce_ctrl_reg[flash->spi_chip->cs].fields.cs_inactive_ctrl = 0;

	return;
}

void aspeed_spi_stop_user(flash_t *flash)
{
	fmc_spi_ctrl_t *ctrl_reg = (fmc_spi_ctrl_t *)flash->spi_controller->ctrl_base;

	ctrl_reg->spi_ce_ctrl_reg[flash->spi_chip->cs].fields.cs_inactive_ctrl = 0;
	ctrl_reg->spi_ce_ctrl_reg[flash->spi_chip->cs].val = flash->spi_chip->ctrl_base_setting;

	return;
}

void aspeed_spi_flash_read_reg(flash_t *flash, uint8_t opcode, uint8_t *buf,
		uint32_t len)
{
	aspeed_spi_start_user(flash);
	aspeed_spi_write_data(flash->spi_chip->ahb_start_addr, 1, &opcode);
	aspeed_spi_read_data(flash->spi_chip->ahb_start_addr, len, buf);
	aspeed_spi_stop_user(flash);

	return;
}

void aspeed_spi_flash_write_reg(flash_t *flash, uint8_t opcode, uint8_t *buf,
		uint32_t len)
{
	aspeed_spi_start_user(flash);
	aspeed_spi_write_data(flash->spi_chip->ahb_start_addr, 1, &opcode);
	aspeed_spi_write_data(flash->spi_chip->ahb_start_addr, len, buf);
	aspeed_spi_stop_user(flash);

	return;
}

void aspeed_spi_send_cmd_addr(flash_t *flash, uint8_t cmd, uint32_t addr)
{
	uint32_t temp;
	uint32_t cmdaddr;

	switch (flash->addr_width) {
	case 3:
		cmdaddr = addr & 0xFFFFFF;
		cmdaddr |= cmd << 24;

		temp = bswap_32(cmdaddr);
		aspeed_spi_write_data(flash->spi_chip->ahb_start_addr , 4, (uint8_t *)&temp);
		break;
	case 4:
		temp = bswap_32(addr);
		aspeed_spi_write_data(flash->spi_chip->ahb_start_addr, 1, &cmd);
		aspeed_spi_write_data(flash->spi_chip->ahb_start_addr, 4, (uint8_t *)&temp);
		break;
	default:
		log_error("Unexpected address width %u, defaulting to 3\n",
				flash->addr_width);
	}

	return;
}

void aspeed_spi_flash_read(flash_t *flash, uint32_t from,
		uint32_t len, uint8_t *read_buf)
{
	uint32_t i;
	uint8_t dummy = 0xFF;
	uint8_t io_mode = aspeed_spi_get_io_mode(flash->spi_chip->rx_bus_width);

	aspeed_spi_start_user(flash);
	aspeed_spi_send_cmd_addr(flash, flash->read_cmd, from);
	for (i = 0; i < flash->read_dummy / 8; i++)
		aspeed_spi_write_data(flash->spi_chip->ahb_start_addr , sizeof(dummy), &dummy);

	/* Set IO mode only for data */
	if ((io_mode == CTRL_IO_DUAL_DATA) || (io_mode == CTRL_IO_QUAD_DATA))
		aspeed_spi_set_io_mode(flash, io_mode);

	aspeed_spi_read_data(flash->spi_chip->ahb_start_addr, len, read_buf);
	aspeed_spi_stop_user(flash);

	return;
}

void aspeed_spi_flash_write_internal(flash_t *flash, uint32_t to,
		uint32_t len, uint8_t *write_buf)
{
	uint8_t io_mode = aspeed_spi_get_io_mode(flash->spi_chip->tx_bus_width);

	aspeed_spi_start_user(flash);
	aspeed_spi_send_cmd_addr(flash, flash->write_cmd, to);

	if ((io_mode == CTRL_IO_DUAL_DATA) || (io_mode == CTRL_IO_QUAD_DATA))
		aspeed_spi_set_io_mode(flash, io_mode);

	aspeed_spi_write_data(flash->spi_chip->ahb_start_addr, len, write_buf);
	aspeed_spi_stop_user(flash);

	return;
}

uint32_t aspeed_get_spi_freq_div(uint32_t max_freq)
{
	uint32_t div_arr[16] = {15, 7, 14, 6, 13, 5, 12, 4, 11, 3, 10, 2, 9, 1, 8, 0};
	uint32_t hclk = aspeed_clk_get_hclk();
	uint32_t i, j;
	bool find = false;

	for (i = 0; i < 0xf; i++) {
		for (j = 0; j < 16; j++) {
			if (i == 0 && j == 0)
				continue;

			if (max_freq >= (hclk / (j + 1 + (i * 16)))) {
				find = true;
				break;
			}
		}

		if (find)
			break;
	}

	if (i == 0xf && j == 16) {
		log_error("%s %d cannot get correct frequence division\n", __func__, __LINE__);
		return 0;
	}

	return ((i << 24) | (div_arr[j] << 8));
}

/*
 * Check whether the data is not all 0 or 1 in order to
 * avoid calibriate umount spi-flash.
 */
static bool aspeed_spi_calibriation_enable(const uint8_t *buf, uint32_t sz)
{
	const uint32_t *buf_32 = (const uint32_t *)buf;
	uint32_t i;
	uint32_t valid_count = 0;

	for (i = 0; i < (sz / 4); i++) {
		if (buf_32[i] != 0 && buf_32[i] != 0xffffffff)
			valid_count++;
		if (valid_count > 100)
			return true;
	}

	return false;
}

static uint32_t
aspeed_spi_dma_checksum(flash_t *flash, uint32_t div, uint32_t delay)
{
	fmc_spi_priv_t *priv = flash->spi_controller;
	uint32_t ctrl_reg = priv->ctrl_base;
	uint32_t ctrl_val;
	uint32_t checksum;

	writel(DMA_GET_REQ_MAGIC, ctrl_reg + OFFSET_DMA_CTRL);
	if (readl(ctrl_reg + OFFSET_DMA_CTRL) & SPI_DAM_REQUEST) {
		while (!(readl(ctrl_reg + OFFSET_DMA_CTRL) &
			 SPI_DAM_GRANT))
			;
	}

	writel(flash->spi_chip->ahb_start_addr,
	       ctrl_reg + OFFSET_DMA_FLASH_ADDR_REG);
	writel(CALIBRATION_LEN, ctrl_reg + OFFSET_DMA_LEN_REG);

	ctrl_val = SPI_DMA_ENABLE | SPI_DMA_CALC_CKSUM | SPI_DMA_CALIB_MODE |
		   (delay << 8) | ((div & 0xf) << 16);
	writel(ctrl_val, ctrl_reg + OFFSET_DMA_CTRL);
	while (!(readl(ctrl_reg + OFFSET_INTR_CTRL_STATUS) &
		 SPI_DMA_STATUS))
		;

	checksum = readl(ctrl_reg + OFFSET_DMA_CHECKSUM_RESULT);

	writel(0x0, ctrl_reg + OFFSET_DMA_CTRL);
	writel(DMA_DISCARD_REQ_MAGIC, ctrl_reg + OFFSET_DMA_CTRL);

	return checksum;
}

static int get_mid_point_of_longest_one(uint8_t *buf, uint32_t len)
{
	int i;
	int start = 0, mid_point = 0;
	int max_cnt = 0, cnt = 0;

	for (i = 0; i < len; i++) {
		if (buf[i] == 1) {
			cnt++;
		} else {
			cnt = 0;
			start = i;
		}

		if (max_cnt < cnt) {
			max_cnt = cnt;
			mid_point = start + (cnt / 2);
		}
	}

	/*
	 * In order to get a stable SPI read timing,
	 * abandon the result if the length of longest
	 * consecutive good points is too short.
	 */
	if (max_cnt < 4)
		return -1;

	return mid_point;
}

void aspeed_spi_timing_calibration(flash_t *flash)
{
	fmc_spi_priv_t *priv = flash->spi_controller;
	uint32_t ctrl_reg = priv->ctrl_base;
	uint32_t cs = flash->spi_chip->cs;
	uint32_t max_freq = flash->spi_chip->max_freq;
	/* HCLK/2, ..., HCKL/5 */
	uint32_t hclk_masks[] = { 7, 14, 6, 13 };
	uint8_t *calib_res = NULL;
	uint8_t *check_buf = NULL;
	uint32_t reg_val;
	uint32_t checksum, gold_checksum;
	uint32_t i, hcycle, delay_ns, final_delay = 0;
	uint32_t hclk_div;
	bool pass;
	int calib_point;

	reg_val =
		readl(ctrl_reg + OFFSET_CE0_TIMING_COMPENSATION + cs * 4);
	if (reg_val != 0) {
		log_debug("has executed calibration.\n");
		goto no_calib;
	}

	log_debug("calculate timing compensation:\n");
	/*
	 * use the related low frequency to get check calibration data
	 * and get golden data.
	 */
	reg_val = readl(ctrl_reg + OFFSET_CE0_CTRL_REG + cs * 4);
	reg_val &= 0xf0fff0ff;
	writel(reg_val, ctrl_reg + OFFSET_CE0_CTRL_REG + cs * 4);

	check_buf = pvPortMalloc(CALIBRATION_LEN);
	if (!check_buf) {
		log_error("Insufficient buffer for calibration.\n");
		goto no_calib;
	}

	log_debug("reg_val = 0x%x\n", reg_val);
	memcpy_fromio(check_buf, flash->spi_chip->ahb_start_addr, CALIBRATION_LEN);

	if (!aspeed_spi_calibriation_enable(check_buf, CALIBRATION_LEN)) {
		log_info("flash data is monotonous, skip calibration.\n");
		goto no_calib;
	}

	gold_checksum = aspeed_spi_dma_checksum(flash, 0, 0);

	/*
	 * allocate a space to record calibration result for
	 * different timing compensation with fixed
	 * HCLK division.
	 */
	calib_res = pvPortMalloc(6 * 17);
	if (!calib_res) {
		log_error("Insufficient buffer for calibration result.\n");
		goto no_calib;;
	}

	/* From HCLK/2 to HCLK/5 */
	for (i = 0; i < ARRAY_SIZE(hclk_masks); i++) {
		if (max_freq < (uint32_t)aspeed_clk_get_hclk() / (i + 2)) {
			log_debug("skipping freq %d\n",
				(uint32_t)aspeed_clk_get_hclk() / (i + 2));
			continue;
		}
		max_freq = (uint32_t)aspeed_clk_get_hclk() / (i + 2);

		checksum = aspeed_spi_dma_checksum(flash, hclk_masks[i], 0);
		pass = (checksum == gold_checksum);
		log_debug("HCLK/%d, no timing compensation: %s\n", i + 2,
			pass ? "PASS" : "FAIL");

		memset(calib_res, 0x0, 6 * 17);

		for (hcycle = 0; hcycle <= 5; hcycle++) {
			/* increase DI delay by the step of 0.5ns */
			log_debug("Delay Enable : hcycle %x\n", hcycle);
			for (delay_ns = 0; delay_ns <= 0xf; delay_ns++) {
				checksum = aspeed_spi_dma_checksum( flash, hclk_masks[i],
					BIT(3) | hcycle | (delay_ns << 4));
				pass = (checksum == gold_checksum);
				calib_res[hcycle * 17 + delay_ns] = pass;
				log_debug("HCLK/%d, %d HCLK cycle, %d delay_ns : %s\n",
					i + 2, hcycle, delay_ns,
					pass ? "PASS" : "FAIL");
			}
		}

		calib_point = get_mid_point_of_longest_one(calib_res, 6 * 17);
		if (calib_point < 0) {
			log_info("cannot get good calibration point.\n");
			continue;
		}

		hcycle = calib_point / 17;
		delay_ns = calib_point % 17;
		log_debug("final hcycle: %d, delay_ns: %d\n", hcycle,
			delay_ns);

		final_delay = (BIT(3) | hcycle | (delay_ns << 4)) << (i * 8);
		writel(final_delay, ctrl_reg +
					    OFFSET_CE0_TIMING_COMPENSATION +
					    cs * 4);
		break;
	}

no_calib:

	hclk_div = aspeed_get_spi_freq_div(max_freq);

	/* configure SPI clock frequency */
	reg_val = readl(ctrl_reg + OFFSET_CE0_CTRL_REG + cs * 4);
	reg_val = (reg_val & 0xf0fff0ff) | hclk_div;
	writel(reg_val, ctrl_reg + OFFSET_CE0_CTRL_REG + cs * 4);

	/* add clock setting info for CE ctrl setting */
	flash->spi_chip->ctrl_base_setting = reg_val;

	log_debug("freq: %dMHz\n", max_freq / 1000000);

	if (check_buf)
		vPortFree(check_buf);
	if (calib_res)
		vPortFree(calib_res);

	return;
}

uint32_t aspeed_spi_decode_range_reinit(flash_t *flash)
{
	uint32_t cs, tmp;
	fmc_spi_priv_t *priv = flash->spi_controller;
	fmc_spi_ctrl_t *ctrl_reg = (fmc_spi_ctrl_t *)priv->ctrl_base;
	uint32_t max_cs = priv->max_cs;
	uint32_t decode_sz_arr[3] = {0};
	uint32_t total_decode_range = 0;
	uint32_t start_addr, end_addr, pre_end_addr = 0;

	/* record original decode range */
	for (cs = 0; cs < max_cs; cs++) {
		tmp = ctrl_reg->spi_ce_addr_decoding_reg[cs].val;
		log_debug("tmp: 0x%08x\n", tmp);
		if (tmp == 0)
			decode_sz_arr[cs] = 0;
		else
			decode_sz_arr[cs] = priv->segment_end(tmp) - priv->segment_start(tmp) + 1;
		total_decode_range += decode_sz_arr[cs];
	}

	/* prepare new decode sz array */
	if (total_decode_range - decode_sz_arr[flash->spi_chip->cs] \
		+ flash->total_sz < 0x10000000) {
		decode_sz_arr[flash->spi_chip->cs] = flash->total_sz;
	} else {
		/* do nothing, otherwise, decode range will be larger than 256MB */
		return 0;
	}

	/* modify decode range */
	for (cs = 0; cs < max_cs; cs++) {
		if (decode_sz_arr[cs] == 0)
			continue;

		if (cs == 0)
			start_addr = flash->spi_controller->ahb_base;
		else
			start_addr = pre_end_addr;

		end_addr = start_addr + decode_sz_arr[cs] - 1;
		log_debug("start: 0x%x end: 0x%x (0x%x)\n", start_addr, end_addr, decode_sz_arr[cs]);

		ctrl_reg->spi_ce_addr_decoding_reg[cs].val = priv->segment_value(start_addr, end_addr);
		flash->spi_controller->chipes[cs].ahb_start_addr = start_addr;
		pre_end_addr = end_addr + 1;
	}

	log_debug("decode reg: <0x%08x, 0x%08x, 0x%08x>\n",
			ctrl_reg->spi_ce_addr_decoding_reg[0].val,
			ctrl_reg->spi_ce_addr_decoding_reg[1].val,
			ctrl_reg->spi_ce_addr_decoding_reg[2].val);

	return 0;
}

uint32_t aspeed_spi_flash_info_deploy(flash_t *flash)
{
	uint32_t ret = 0;
	uint32_t cs = flash->spi_chip->cs;
	fmc_spi_ctrl_t *ctrl_reg = (fmc_spi_ctrl_t *)flash->spi_controller->ctrl_base;

	ctrl_reg->spi_ce_ctrl_reg[cs].fields.io_mode = aspeed_spi_get_io_mode(flash->spi_chip->rx_bus_width);
	ctrl_reg->spi_ce_ctrl_reg[cs].fields.spi_cmd = flash->read_cmd;
	ctrl_reg->spi_ce_ctrl_reg[cs].fields.spi_read_dummy_cycles_low_bits = flash->read_dummy / 8;
	ctrl_reg->spi_ce_ctrl_reg[cs].fields.cs_inactive_ctrl = 0;
	ctrl_reg->spi_ce_ctrl_reg[cs].fields.spi_cmd_mode = CTRL_CMD_MODE_FREAD;
	flash->spi_chip->ctrl_base_setting = ctrl_reg->spi_ce_ctrl_reg[cs].val;

	return ret;
}

void aspeed_decode_range_pre_init(fmc_spi_ctrl_t *ctrl_reg, fmc_spi_priv_t *priv)
{
	uint32_t cs;
	uint32_t unit_sz = 0x200000; /* init 2M for each cs */
	uint32_t start_addr, end_addr, pre_end_addr = 0;

	for (cs = 0; cs < priv->max_cs; cs++) {
		if (priv->chipes[cs].enable == false) {
			priv->chipes[cs].ahb_start_addr = 0;
			ctrl_reg->spi_ce_addr_decoding_reg[cs].val = 0;
			continue;
		}

		if (cs == 0)
			start_addr = priv->ahb_base;
		else
			start_addr = pre_end_addr;

		end_addr = start_addr + unit_sz - 1;

		/* the maximum decode size is 256MB */
		if (priv->ahb_base + 0x10000000 <= end_addr) {
			log_error("%s %d smc decode address overflow\n", __func__, __LINE__);
			continue;
		}

		log_debug("cs: %d start: 0x%08x, end: 0x%08x (%08x)\n",
				cs, start_addr, end_addr, priv->segment_value(start_addr, end_addr));

		ctrl_reg->spi_ce_addr_decoding_reg[cs].val = \
				priv->segment_value(start_addr, end_addr);
		log_debug("cs: %d 0x%08x\n", cs, ctrl_reg->spi_ce_addr_decoding_reg[cs].val);
		priv->chipes[cs].ahb_start_addr = start_addr;
		pre_end_addr = end_addr + 1;
	}

	return;
}

/* only fill SAFS related registers when cs is 0 */
void aspeed_spi_fill_safs_cmd(spi_t *spi, flash_t flash)
{
	uint32_t ctrl_base = spi->device->base;
	uint32_t tmp_val;
	/* rx */
	tmp_val = readl(ctrl_base + OFFSET_HOST_DIRECT_ACCESS_CMD_CTRL4);
	if (flash.addr_width == 4)
		tmp_val = (tmp_val & 0xffff00ff) | (flash.read_cmd << 8);
	else
		tmp_val = (tmp_val & 0xffffff00) | flash.read_cmd;

	tmp_val = (tmp_val & 0x0fffffff) | \
			(aspeed_spi_get_io_mode(flash.spi_chip->rx_bus_width) << 28);
	writel(tmp_val, ctrl_base + OFFSET_HOST_DIRECT_ACCESS_CMD_CTRL4);

	/* tx */
	tmp_val = readl(ctrl_base + OFFSET_HOST_DIRECT_ACCESS_CMD_CTRL4);
	tmp_val = (tmp_val & 0xf0ffffff) |
			(aspeed_spi_get_io_mode(flash.spi_chip->tx_bus_width) << 24);
	writel(tmp_val, ctrl_base + OFFSET_HOST_DIRECT_ACCESS_CMD_CTRL4);
	tmp_val = readl(ctrl_base + OFFSET_HOST_DIRECT_ACCESS_CMD_CTRL2);
	if (flash.addr_width == 4)
		tmp_val = (tmp_val & 0xffff00ff) | (flash.write_cmd << 8);
	else
		tmp_val = (tmp_val & 0xffffff00) | flash.write_cmd;

	writel(tmp_val, ctrl_base + OFFSET_HOST_DIRECT_ACCESS_CMD_CTRL2);
}

void aspeed_spi_ctrl_init(spi_t *spi)
{
	fmc_spi_ctrl_t *ctrl_reg = (fmc_spi_ctrl_t *)spi->device->base;
	fmc_spi_priv_t *priv = spi->device->private;
	uint32_t cs;

	if(spi->device->init)
		return;

	/* copy ctrl base to priv struct in order to simplify source code. */
	priv->ctrl_base = spi->device->base;

	/* set inital decode address for user mode */
	aspeed_decode_range_pre_init(ctrl_reg, priv);

	for (cs = 0; cs < priv->max_cs; cs++) {
		/* get frequency */
		ctrl_reg->spi_ce_ctrl_reg[cs].val = 0;
		//aspeed_get_spi_freq_div(priv->chipes[cs].max_freq);
		/* set into user mode by default */
		ctrl_reg->spi_ce_ctrl_reg[cs].fields.spi_cmd_mode = CTRL_CMD_MODE_USER;
		ctrl_reg->spi_ce_ctrl_reg[cs].fields.cs_inactive_ctrl = 1;

		priv->chipes[cs].ctrl_base_setting = ctrl_reg->spi_ce_ctrl_reg[cs].val;
		/* write enable */
		ctrl_reg->spi_ce_type_setting_reg.fields.ce_write_type |= 1 << cs;

		priv->chipes[cs].cs = cs;
	}

	spi->device->init = 1;

	return;
}

uint32_t aspeed_spi_driver_caps(void)
{
	return SNOR_HWCAPS_READ | SNOR_HWCAPS_READ_FAST | \
		SNOR_HWCAPS_READ_1_1_2 | SNOR_HWCAPS_READ_1_1_4 | \
		SNOR_HWCAPS_PP | SNOR_HWCAPS_PP_1_1_4 | SNOR_HWCAPS_ERASE_MASK;
}

uint32_t aspeed_spi_init(spi_t *spi)
{
	uint32_t ret = 0;

	aspeed_spi_ctrl_init(spi);

	return ret;
}

