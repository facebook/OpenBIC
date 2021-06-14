/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "util.h"
#include "common.h"
#include "flash.h"
#include "flash_api.h"
#include "fmc_spi_aspeed.h"
#include "fmc_spi_err.h"
#include "log.h"
#include "objects.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "wait.h"

static const struct sfdp_bfpt_read sfdp_bfpt_reads[] = {
	/* Fast Read 1-1-2 */
	{
		SNOR_HWCAPS_READ_1_1_2,
		BFPT_DWORD(1), BIT(16),	/* Supported bit */
		BFPT_DWORD(4), 0,	/* Settings */
		SNOR_PROTO_1_1_2,
	},

	/* Fast Read 1-2-2 */
	{
		SNOR_HWCAPS_READ_1_2_2,
		BFPT_DWORD(1), BIT(20),	/* Supported bit */
		BFPT_DWORD(4), 16,	/* Settings */
		SNOR_PROTO_1_2_2,
	},

	/* Fast Read 2-2-2 */
	{
		SNOR_HWCAPS_READ_2_2_2,
		BFPT_DWORD(5),  BIT(0),	/* Supported bit */
		BFPT_DWORD(6), 16,	/* Settings */
		SNOR_PROTO_2_2_2,
	},

	/* Fast Read 1-1-4 */
	{
		SNOR_HWCAPS_READ_1_1_4,
		BFPT_DWORD(1), BIT(22),	/* Supported bit */
		BFPT_DWORD(3), 16,	/* Settings */
		SNOR_PROTO_1_1_4,
	},

	/* Fast Read 1-4-4 */
	{
		SNOR_HWCAPS_READ_1_4_4,
		BFPT_DWORD(1), BIT(21),	/* Supported bit */
		BFPT_DWORD(3), 0,	/* Settings */
		SNOR_PROTO_1_4_4,
	},

	/* Fast Read 4-4-4 */
	{
		SNOR_HWCAPS_READ_4_4_4,
		BFPT_DWORD(5), BIT(4),	/* Supported bit */
		BFPT_DWORD(7), 16,	/* Settings */
		SNOR_PROTO_4_4_4,
	},
};

static const struct sfdp_bfpt_erase sfdp_bfpt_erases[] = {
	/* Erase Type 1 in DWORD8 bits[15:0] */
	{SNOR_HWCAPS_ERASE_TYPE_1, BFPT_DWORD(8), 0},

	/* Erase Type 2 in DWORD8 bits[31:16] */
	{SNOR_HWCAPS_ERASE_TYPE_2, BFPT_DWORD(8), 16},

	/* Erase Type 3 in DWORD9 bits[15:0] */
	{SNOR_HWCAPS_ERASE_TYPE_3, BFPT_DWORD(9), 0},

	/* Erase Type 4 in DWORD9 bits[31:16] */
	{SNOR_HWCAPS_ERASE_TYPE_4, BFPT_DWORD(9), 16},
};

uint32_t aspeed_get_flash_id(flash_t *flash)
{
	uint8_t flash_id[3] __attribute__ ((aligned (8))) = {0};

	aspeed_spi_flash_read_reg(flash, SPINOR_OP_RDID, flash_id, 3);

	return flash_id[0] << 16 | flash_id[1] << 8 | flash_id[2];
}

/*
 * Read the status register, returning its value in the location
 * Return the status register value.
 * Returns negative if error occurred.
 */
uint8_t read_sr(flash_t *flash)
{
	uint8_t val = 0;

	aspeed_spi_flash_read_reg(flash, SPINOR_OP_RDSR, &val, 1);

	return val;
}

/*
 * Read configuration register, returning its value in the
 * location. Return the configuration register value.
 * Returns negative if error occurred.
 */
uint8_t read_cr(flash_t *flash)
{
	uint8_t val;

	aspeed_spi_flash_read_reg(flash, SPINOR_OP_RDCR, &val, 1);

	return val;
}

void spi_nor_wait_ready(flash_t *flash)
{
	uint8_t sr;

	sr = read_sr(flash);

	while (sr & SR_WIP) {
		aspeed_wait_us(20);
		sr = read_sr(flash);
	}

	return;
}

/*
 * Set write enable latch with Write Enable command.
 * Returns negative if error occurred.
 */
void write_enable(flash_t *flash)
{
	aspeed_spi_flash_write_reg(flash, SPINOR_OP_WREN, NULL, 0);
}

/*
 * Send write disable instruction to the chip.
 */
void write_disable(flash_t *flash)
{
	aspeed_spi_flash_write_reg(flash, SPINOR_OP_WRDI, NULL, 0);
}

/*
 * Write status Register and configuration register with 2 bytes
 * The first byte will be written to the status register, while the
 * second byte will be written to the configuration register.
 * Return negative if error occurred.
 */
void write_sr_cr(flash_t *flash, uint8_t *sr_cr)
{
	write_enable(flash);

	aspeed_spi_flash_write_reg(flash, SPINOR_OP_WRSR, sr_cr, 2);

	spi_nor_wait_ready(flash);

	return;
}

/*
 * Write status register 1 byte
 * Returns negative if error occurred.
 */
void write_sr(flash_t *flash, uint8_t val)
{
	aspeed_spi_flash_write_reg(flash, SPINOR_OP_WRSR, &val, 1);
	return;
}

uint32_t spi_nor_detect_flash_exist_roughly(uint32_t flash_id)
{
	uint32_t ret = 0;

	switch (JEDEC_MFR(flash_id)) {
	case SNOR_MFR_SPANSION:
	case SNOR_MFR_MACRONIX:
	case SNOR_MFR_ST:
	case SNOR_MFR_MICRON:
	case SNOR_MFR_WINBOND:
		break;
	default:
		ret = ERR_FLASH_ID_NOT_FOUND;
	}

	return ret;
}

uint32_t set_4byte(flash_t *flash, bool enable_4byte)
{
	uint32_t ret = 0;
	bool need_wren = false;
	uint8_t cmd;

	switch (JEDEC_MFR(flash->id)) {
	case SNOR_MFR_ST:
	case SNOR_MFR_MICRON:
		/* Some Micron need WREN command; all will accept it */
		need_wren = true;
		/* fall through */
	case SNOR_MFR_MACRONIX:
	case SNOR_MFR_WINBOND:
		if (need_wren)
			write_enable(flash);

		if (enable_4byte)
			log_debug("enter 4 byte mode\n");
		else
			log_debug("exit 4 byte mode\n");

		cmd = enable_4byte ? SPINOR_OP_EN4B : SPINOR_OP_EX4B;
		aspeed_spi_flash_write_reg(flash, cmd, NULL, 0);
		if (need_wren)
			write_disable(flash);

		if (!enable_4byte &&
		    JEDEC_MFR(flash->id) == SNOR_MFR_WINBOND) {
			/*
			 * On Winbond W25Q256FV, leaving 4byte mode causes
			 * the Extended Address Register to be set to 1, so all
			 * 3-byte-address reads come from the second 16M.
			 * We must clear the register to enable normal behavior.
			 */
			write_enable(flash);
			flash->cmd_addr_buf[0] = 0;
			aspeed_spi_flash_write_reg(flash, SPINOR_OP_WREAR, flash->cmd_addr_buf, 1);
			write_disable(flash);
		}

		break;
	default:
		/* Spansion style */
		flash->cmd_addr_buf[0] = enable_4byte << 7;
		aspeed_spi_flash_write_reg(flash, SPINOR_OP_BRWR, flash->cmd_addr_buf, 1);
	}

	return ret;
}

/**
 * macronix_quad_enable() - set QE bit in Status Register.
 * @nor:	pointer to a 'struct spi_nor'
 *
 * Set the Quad Enable (QE) bit in the Status Register.
 *
 * bit 6 of the Status Register is the QE bit for Macronix like QSPI memories.
 *
 * Return: 0 on success, -errno otherwise.
 */
uint32_t macronix_quad_enable(flash_t *flash)
{
	uint8_t val;

	val = read_sr(flash);
	if (val & SR_QUAD_EN_MX)
		return 0;

	write_enable(flash);

	write_sr(flash, val | SR_QUAD_EN_MX);

	spi_nor_wait_ready(flash);

	val = read_sr(flash);
	if (!(val > 0 && (val & SR_QUAD_EN_MX))) {
		log_error("Macronix Quad bit not set\n");
	}

	return 0;
}

/**
 * spansion_no_read_cr_quad_enable() - set QE bit in Configuration Register.
 * @nor:	pointer to a 'struct spi_nor'
 *
 * Set the Quad Enable (QE) bit in the Configuration Register.
 * This function should be used with QSPI memories not supporting the Read
 * Configuration Register (35h) instruction.
 *
 * bit 1 of the Configuration Register is the QE bit for Spansion like QSPI
 * memories.
 *
 * Return: 0 on success, -errno otherwise.
 */
uint32_t spansion_no_read_cr_quad_enable(flash_t *flash)
{
	uint8_t sr_cr[2];

	/* Keep the current value of the Status Register. */
	sr_cr[0] = read_sr(flash);
	sr_cr[1] = CR_QUAD_EN_SPAN;

	write_sr_cr(flash, sr_cr);

	return 0;
}

/**
 * spansion_read_cr_quad_enable() - set QE bit in Configuration Register.
 * @nor:	pointer to a 'struct spi_nor'
 *
 * Set the Quad Enable (QE) bit in the Configuration Register.
 * This function should be used with QSPI memories supporting the Read
 * Configuration Register (35h) instruction.
 *
 * bit 1 of the Configuration Register is the QE bit for Spansion like QSPI
 * memories.
 *
 * Return: 0 on success, -errno otherwise.
 */
uint32_t spansion_read_cr_quad_enable(flash_t *flash)
{
	uint8_t sr_cr[2];
	uint8_t ret;

	/* Check current Quad Enable bit value. */
	ret = read_cr(flash);

	if (ret & CR_QUAD_EN_SPAN)
		return 0;

	sr_cr[1] = ret | CR_QUAD_EN_SPAN;

	/* Keep the current value of the Status Register. */
	ret = read_sr(flash);
	sr_cr[0] = ret;

	write_sr_cr(flash, sr_cr);

	/* Read back and check it. */
	ret = read_cr(flash);
	if (!(ret > 0 && (ret & CR_QUAD_EN_SPAN))) {
		log_error("Spansion Quad bit not set\n");
	}

	return 0;
}


static uint32_t spi_nor_hwcaps2cmd(uint32_t hwcaps, const uint32_t table[][2], uint32_t sz)
{
	uint32_t i;

	for (i = 0; i < sz; i++)
		if (table[i][0] == hwcaps)
			return table[i][1];

	/* select default value */
	return 0;
}

static uint32_t spi_nor_hwcaps_read2cmd(uint32_t hwcaps)
{
	static const uint32_t hwcaps_read2cmd[][2] = {
		{ SNOR_HWCAPS_READ,		SNOR_CMD_READ },
		{ SNOR_HWCAPS_READ_FAST,	SNOR_CMD_READ_FAST },
		{ SNOR_HWCAPS_READ_1_1_1_DTR,	SNOR_CMD_READ_1_1_1_DTR },
		{ SNOR_HWCAPS_READ_1_1_2,	SNOR_CMD_READ_1_1_2 },
		{ SNOR_HWCAPS_READ_1_2_2,	SNOR_CMD_READ_1_2_2 },
		{ SNOR_HWCAPS_READ_2_2_2,	SNOR_CMD_READ_2_2_2 },
		{ SNOR_HWCAPS_READ_1_2_2_DTR,	SNOR_CMD_READ_1_2_2_DTR },
		{ SNOR_HWCAPS_READ_1_1_4,	SNOR_CMD_READ_1_1_4 },
		{ SNOR_HWCAPS_READ_1_4_4,	SNOR_CMD_READ_1_4_4 },
		{ SNOR_HWCAPS_READ_4_4_4,	SNOR_CMD_READ_4_4_4 },
		{ SNOR_HWCAPS_READ_1_4_4_DTR,	SNOR_CMD_READ_1_4_4_DTR },
		{ SNOR_HWCAPS_READ_1_1_8,	SNOR_CMD_READ_1_1_8 },
		{ SNOR_HWCAPS_READ_1_8_8,	SNOR_CMD_READ_1_8_8 },
		{ SNOR_HWCAPS_READ_8_8_8,	SNOR_CMD_READ_8_8_8 },
		{ SNOR_HWCAPS_READ_1_8_8_DTR,	SNOR_CMD_READ_1_8_8_DTR },
	};

	return spi_nor_hwcaps2cmd(hwcaps, hwcaps_read2cmd,
			ARRAY_SIZE(hwcaps_read2cmd));
}

uint32_t spi_nor_hwcaps_pp2cmd(uint32_t hwcaps)
{
	static const uint32_t hwcaps_pp2cmd[][2] = {
		{ SNOR_HWCAPS_PP,		SNOR_CMD_PP },
		{ SNOR_HWCAPS_PP_1_1_4,		SNOR_CMD_PP_1_1_4 },
		{ SNOR_HWCAPS_PP_1_4_4,		SNOR_CMD_PP_1_4_4 },
		{ SNOR_HWCAPS_PP_4_4_4,		SNOR_CMD_PP_4_4_4 },
		{ SNOR_HWCAPS_PP_1_1_8,		SNOR_CMD_PP_1_1_8 },
		{ SNOR_HWCAPS_PP_1_8_8,		SNOR_CMD_PP_1_8_8 },
		{ SNOR_HWCAPS_PP_8_8_8,		SNOR_CMD_PP_8_8_8 },
	};

	return spi_nor_hwcaps2cmd(hwcaps, hwcaps_pp2cmd,
			ARRAY_SIZE(hwcaps_pp2cmd));
}

uint32_t spi_nor_hwcaps_erase2cmd(uint32_t hwcaps)
{
	static const uint32_t hwcaps_erase2cmd[][2] = {
		{ SNOR_HWCAPS_ERASE_4K,         SNOR_CMD_ERASE_4K },
		{ SNOR_HWCAPS_ERASE_TYPE_1,     SNOR_CMD_ERASE_TYPE_1 },
		{ SNOR_HWCAPS_ERASE_TYPE_2,     SNOR_CMD_ERASE_TYPE_2 },
		{ SNOR_HWCAPS_ERASE_TYPE_3,     SNOR_CMD_ERASE_TYPE_3 },
		{ SNOR_HWCAPS_ERASE_TYPE_4,     SNOR_CMD_ERASE_TYPE_4 },
	};

	return spi_nor_hwcaps2cmd(hwcaps, hwcaps_erase2cmd,
			ARRAY_SIZE(hwcaps_erase2cmd));
}

static uint8_t spi_nor_convert_opcode(uint8_t opcode, const uint8_t table[][2], uint32_t size)
{
	size_t i;

	for (i = 0; i < size; i++)
		if (table[i][0] == opcode)
			return table[i][1];

	/* No conversion found, keep input op code. */
	return opcode;
}

static uint8_t spi_nor_convert_3to4_read(uint8_t opcode)
{
	static const uint8_t spi_nor_3to4_read[][2] = {
		{ SPINOR_OP_READ,	SPINOR_OP_READ_4B },
		{ SPINOR_OP_READ_FAST,	SPINOR_OP_READ_FAST_4B },
		{ SPINOR_OP_READ_1_1_2,	SPINOR_OP_READ_1_1_2_4B },
		{ SPINOR_OP_READ_1_2_2,	SPINOR_OP_READ_1_2_2_4B },
		{ SPINOR_OP_READ_1_1_4,	SPINOR_OP_READ_1_1_4_4B },
		{ SPINOR_OP_READ_1_4_4,	SPINOR_OP_READ_1_4_4_4B },
		{ SPINOR_OP_READ_1_1_8,	SPINOR_OP_READ_1_1_8_4B },
		{ SPINOR_OP_READ_1_8_8,	SPINOR_OP_READ_1_8_8_4B },

		{ SPINOR_OP_READ_1_1_1_DTR,	SPINOR_OP_READ_1_1_1_DTR_4B },
		{ SPINOR_OP_READ_1_2_2_DTR,	SPINOR_OP_READ_1_2_2_DTR_4B },
		{ SPINOR_OP_READ_1_4_4_DTR,	SPINOR_OP_READ_1_4_4_DTR_4B },
	};

	return spi_nor_convert_opcode(opcode, spi_nor_3to4_read,
			ARRAY_SIZE(spi_nor_3to4_read));
}

uint8_t spi_nor_convert_3to4_program(uint8_t opcode)
{
	static const uint8_t spi_nor_3to4_program[][2] = {
		{ SPINOR_OP_PP,		SPINOR_OP_PP_4B },
		{ SPINOR_OP_PP_1_1_4,	SPINOR_OP_PP_1_1_4_4B },
		{ SPINOR_OP_PP_1_4_4,	SPINOR_OP_PP_1_4_4_4B },
		{ SPINOR_OP_PP_1_1_8,	SPINOR_OP_PP_1_1_8_4B },
		{ SPINOR_OP_PP_1_8_8,	SPINOR_OP_PP_1_8_8_4B },
	};

	return spi_nor_convert_opcode(opcode, spi_nor_3to4_program,
			ARRAY_SIZE(spi_nor_3to4_program));
}


uint32_t spi_nor_select_read(flash_t *flash,
		struct spi_nor_flash_parameter *params,
		uint32_t shared_hwcaps)
{
	uint32_t cmd_idx;
	uint32_t best_match = fls(shared_hwcaps & SNOR_HWCAPS_READ_MASK) - 1;
	struct spi_nor_read_cmd *read;

	cmd_idx = spi_nor_hwcaps_read2cmd(BIT(best_match));

	read = &params->reads[cmd_idx];
	flash->read_cmd = read->opcode;
	flash->read_dummy = read->num_mode_clocks + read->num_wait_states;
	if ((read->proto & SNOR_PROTO_DATA_MASK) == 1) {
		log_info("reset read bus width to single.\n");
		flash->spi_chip->rx_bus_width = 1;
	}

	return 0;
}

uint32_t spi_nor_select_pp(flash_t *flash,
		struct spi_nor_flash_parameter *params,
		uint32_t shared_hwcaps)
{
	uint32_t cmd_idx;
	uint32_t best_match = fls(shared_hwcaps & SNOR_HWCAPS_PP_MASK) - 1;
	const struct spi_nor_pp_cmd *pp;

	cmd_idx = spi_nor_hwcaps_pp2cmd(BIT(best_match));

	pp = &params->page_programs[cmd_idx];
	flash->write_cmd = pp->opcode;
	if ((pp->proto & SNOR_PROTO_DATA_MASK) == 1) {
		log_info("reset write bus width to single.\n");
		flash->spi_chip->tx_bus_width = 1;
	}

	return 0;
}

uint32_t spi_nor_select_erase(flash_t *flash,
		struct spi_nor_flash_parameter *params,
		uint32_t shared_hwcaps)
{
	uint32_t cmd_idx;
	uint32_t best_match = ffs(shared_hwcaps & SNOR_HWCAPS_ERASE_MASK) - 1;
	const struct spi_nor_erase_cmd *erase;

	cmd_idx = spi_nor_hwcaps_erase2cmd(BIT(best_match));

	erase = &params->erases[cmd_idx];
	flash->erase_cmd = erase->opcode;
	flash->sector_sz = erase->erase_sz;
	log_debug("best match: %d, opcode: %x\n", best_match, erase->opcode);

	return 0;
}

static void spi_nor_set_read_settings(struct spi_nor_read_cmd *read,
		uint8_t num_mode_clocks,
		uint8_t num_wait_states,
		uint8_t opcode,
		enum spi_nor_protocol proto)
{
	read->num_mode_clocks = num_mode_clocks;
	read->num_wait_states = num_wait_states;
	read->opcode = opcode;
	read->proto = proto;
}

static void spi_nor_set_pp_settings(struct spi_nor_pp_cmd *pp,
		uint8_t opcode,
		enum spi_nor_protocol proto)
{
	pp->opcode = opcode;
	pp->proto = proto;
}

static void spi_nor_set_erase_settings(struct spi_nor_erase_cmd *erase,
		uint8_t opcode,
		uint32_t erase_sz)
{
	erase->opcode = opcode;
	erase->erase_sz = erase_sz;
}

uint32_t aspeed_get_flash_param_from_ids(flash_t *flash, uint32_t flash_id,
		struct spi_nor_flash_parameter *params)
{

	flash_info_t *info = flash_ids;
	for (; info->name; info++) {
		if (info->id) {
			if ( info->id == flash_id)
				break;
		}
	}

	if (info->name == NULL) {
		log_debug("id (0x%x) not found, get flash flash size from sfdp\n", flash_id);
		flash->total_sz = 0;
	} else {
		flash->total_sz = info->total_sz * 1024; /* transfer to byte*/
	}

	if (!(info->flag & SPI_NOR_NO_FR)) {
		params->hwcaps |= SNOR_HWCAPS_READ_FAST;
		spi_nor_set_read_settings(&params->reads[SNOR_CMD_READ_FAST],
				0, 8, SPINOR_OP_READ_FAST,
				SNOR_PROTO_1_1_1);
	}

	if (info->flag & SPI_NOR_DUAL_READ) {
		params->hwcaps |= SNOR_HWCAPS_READ_1_1_2;
		spi_nor_set_read_settings(&params->reads[SNOR_CMD_READ_1_1_2],
				0, 8, SPINOR_OP_READ_1_1_2,
				SNOR_PROTO_1_1_2);
	}

	if (info->flag & SPI_NOR_QUAD_READ) {
		params->hwcaps |= SNOR_HWCAPS_READ_1_1_4;
		spi_nor_set_read_settings(&params->reads[SNOR_CMD_READ_1_1_4],
				0, 8, SPINOR_OP_READ_1_1_4,
				SNOR_PROTO_1_1_4);
	}

	return 0;
}

static void spi_nor_read_sfdp(flash_t *flash, uint32_t addr,
		uint32_t len, void *buf)
{
	uint8_t addr_width, read_cmd, read_dummy;
	uint32_t rx_bus_width;

	read_cmd = flash->read_cmd;
	addr_width = flash->addr_width;
	read_dummy = flash->read_dummy;
	rx_bus_width = flash->spi_chip->rx_bus_width;

	flash->read_cmd = SPINOR_OP_RDSFDP;
	flash->addr_width = 3;
	flash->read_dummy = 8;
	flash->spi_chip->rx_bus_width = 1;

	aspeed_spi_flash_read(flash, addr, len, buf);

	flash->read_cmd = read_cmd;
	flash->addr_width = addr_width;
	flash->read_dummy = read_dummy;
	flash->spi_chip->rx_bus_width = rx_bus_width;

	return;
}

static void spi_nor_set_read_settings_from_bfpt(
		struct spi_nor_read_cmd *read,
		uint16_t half,
		enum spi_nor_protocol proto)
{
	read->num_mode_clocks = (half >> 5) & 0x07;
	read->num_wait_states = (half >> 0) & 0x1f;
	read->opcode = (half >> 8) & 0xff;
	read->proto = proto;
}


static uint32_t spi_nor_parse_bfpt(flash_t *flash,
		struct sfdp_parameter_header *bfpt_header,
		struct spi_nor_flash_parameter *params)
{
	uint32_t ret = 0;
	struct sfdp_bfpt bfpt;
	uint32_t len;
	uint32_t i, cmd;
	uint32_t addr;
	uint16_t half;

	/* JESD216 Basic Flash Parameter Table length is at least 9 DWORDs. */
	if (bfpt_header->length < BFPT_DWORD_MAX_JESD216)
		return ERR_INVALID_BFPT_LEN;

	/* Read the Basic Flash Parameter Table. */
	len = MIN(sizeof(bfpt), bfpt_header->length * sizeof(uint32_t));
	addr = SFDP_PARAM_HEADER_PTP(bfpt_header);
	memset(&bfpt, 0, sizeof(bfpt));
	spi_nor_read_sfdp(flash, addr, len, &bfpt);

	/* Fix endianness of the BFPT DWORDs. */
	for (i = 0; i < BFPT_DWORD_MAX; i++)
		bfpt.dwords[i] = bfpt.dwords[i];

	/* Number of address bytes. */
	switch (bfpt.dwords[BFPT_DWORD(1)] & BFPT_DWORD1_ADDRESS_BYTES_MASK) {
	case BFPT_DWORD1_ADDRESS_BYTES_3_ONLY:
		flash->addr_width = 3;
		break;
	case BFPT_DWORD1_ADDRESS_BYTES_4_ONLY:
		flash->addr_width = 4;
		break;
	case BFPT_DWORD1_ADDRESS_BYTES_3_OR_4:
		flash->addr_width = 4;
		flash->enter_4byte = true;
	default:
		break;
	}

	/* Flash Memory Density (in bits). */
	params->sz = bfpt.dwords[BFPT_DWORD(2)];
	if (params->sz & BIT(31)) {
		params->sz &= ~BIT(31);

		/*
		 * Prevent overflows on params->sz. Anyway, a NOR of 2^64
		 * bits is unlikely to exist so this error probably means
		 * the BFPT we are reading is corrupted/wrong.
		 */
		if (params->sz > 63)
			return ERR_INVALID_FLASH_SZ;

		params->sz = 1ULL << params->sz;
	} else {
		params->sz++;
	}

	params->sz >>= 3; /* Convert to bytes. */

	if (flash->total_sz == 0)
		flash->total_sz = params->sz;

	/* volatile status register write enable */
	flash->status_wen = 0;
	if (bfpt.dwords[BFPT_DWORD(1)] & BIT(3)) {
		if (bfpt.dwords[BFPT_DWORD(1)] & BIT(4))
			flash->status_wen = 0x06;
		else
			flash->status_wen = 0x50;
	}

	/* check 4k erase status */
	if ((bfpt.dwords[BFPT_DWORD(1)] & GENMASK(1, 0)) == 0x1) {
		params->hwcaps |= SNOR_HWCAPS_ERASE_4K;
		spi_nor_set_erase_settings(&params->erases[SNOR_CMD_ERASE_4K],
				(uint8_t)((bfpt.dwords[BFPT_DWORD(1)] & GENMASK(15, 8)) >> 8),
				4 * 1024);
	} else {
		/* clear default erase info */
		params->hwcaps &= ~SNOR_HWCAPS_ERASE_4K;
		spi_nor_set_erase_settings(&params->erases[SNOR_CMD_ERASE_4K],
				0, 0);
	}

	/* Fast Read settings. */
	for (i = 0; i < ARRAY_SIZE(sfdp_bfpt_reads); i++) {
		const struct sfdp_bfpt_read *rd = &sfdp_bfpt_reads[i];
		struct spi_nor_read_cmd *read;

		if (!(bfpt.dwords[rd->supported_dword] & rd->supported_bit)) {
			params->hwcaps &= ~rd->hwcaps;
			continue;
		}

		params->hwcaps |= rd->hwcaps;
		cmd = spi_nor_hwcaps_read2cmd(rd->hwcaps);
		read = &params->reads[cmd];
		half = bfpt.dwords[rd->settings_dword] >> rd->settings_shift;
		spi_nor_set_read_settings_from_bfpt(read, half, rd->proto);
	}

	/*
	 * Sector Erase settings. Reinitialize the uniform erase map using the
	 * Erase Types defined in the bfpt table.
	 */
	for (i = 0; i < ARRAY_SIZE(sfdp_bfpt_erases); i++) {
		const struct sfdp_bfpt_erase *er = &sfdp_bfpt_erases[i];
		uint32_t erasesize;
		uint8_t opcode;

		half = bfpt.dwords[er->dword] >> er->shift;
		erasesize = half & 0xff;

		/* erasesize == 0 means this Erase Type is not supported. */
		if (!erasesize)
			continue;

		erasesize = 1U << erasesize;
		opcode = (half >> 8) & 0xff;
		params->hwcaps |= er->hwcaps;
		spi_nor_set_erase_settings(&params->erases[i + 1],
				opcode, erasesize);
	}

	/* Stop here if not JESD216 rev A or later. */
	if (bfpt_header->length < BFPT_DWORD_MAX)
		return ret;

	/* Page size: this field specifies 'N' so the page size = 2^N bytes. */
	params->page_sz = bfpt.dwords[BFPT_DWORD(11)];
	params->page_sz &= BFPT_DWORD11_PAGE_SIZE_MASK;
	params->page_sz >>= BFPT_DWORD11_PAGE_SIZE_SHIFT;
	params->page_sz = 1U << params->page_sz;
	log_debug("%s %d\n", __func__, __LINE__);
	log_debug("0x%08x\n", bfpt.dwords[BFPT_DWORD(15)]);
	/* Quad Enable Requirements. */
	switch (bfpt.dwords[BFPT_DWORD(15)] & BFPT_DWORD15_QER_MASK) {
	case BFPT_DWORD15_QER_NONE:
		flash->quad_enable = NULL;
		break;

	case BFPT_DWORD15_QER_SR2_BIT1_BUGGY:
	case BFPT_DWORD15_QER_SR2_BIT1_NO_RD:
		flash->quad_enable = spansion_no_read_cr_quad_enable;
		break;

	case BFPT_DWORD15_QER_SR1_BIT6:
		flash->quad_enable = macronix_quad_enable;
		break;

	case BFPT_DWORD15_QER_SR2_BIT1:
		flash->quad_enable = spansion_read_cr_quad_enable;
		break;

	default:
		flash->quad_enable = NULL;
		log_info("quad enable bit is not enabled.\n");
	}

	return 0;
}


/**
 * spi_nor_parse_4bait() - parse the 4-Byte Address Instruction Table
 * @nor:		pointer to a 'struct spi_nor'.
 * @param_header:	pointer to the 'struct sfdp_parameter_header' describing
 *			the 4-Byte Address Instruction Table length and version.
 * @params:		pointer to the 'struct spi_nor_flash_parameter' to be.
 *
 * Return: 0 on success, -errno otherwise.
 */
uint32_t spi_nor_parse_4bait(flash_t *flash,
		struct sfdp_parameter_header *param_header,
		struct spi_nor_flash_parameter *params)
{
	static const struct sfdp_4bait reads[] = {
		{ SNOR_HWCAPS_READ,		BIT(0) },
		{ SNOR_HWCAPS_READ_FAST,	BIT(1) },
		{ SNOR_HWCAPS_READ_1_1_2,	BIT(2) },
		{ SNOR_HWCAPS_READ_1_2_2,	BIT(3) },
		{ SNOR_HWCAPS_READ_1_1_4,	BIT(4) },
		{ SNOR_HWCAPS_READ_1_4_4,	BIT(5) },
		{ SNOR_HWCAPS_READ_1_1_1_DTR,	BIT(13) },
		{ SNOR_HWCAPS_READ_1_2_2_DTR,	BIT(14) },
		{ SNOR_HWCAPS_READ_1_4_4_DTR,	BIT(15) },
	};
	static const struct sfdp_4bait programs[] = {
		{ SNOR_HWCAPS_PP,		BIT(6) },
		{ SNOR_HWCAPS_PP_1_1_4,		BIT(7) },
		{ SNOR_HWCAPS_PP_1_4_4,		BIT(8) },
	};
	static const struct sfdp_4bait erases[SNOR_ERASE_TYPE_MAX + 1] = {
		{ 0u /* not used */,            0 },
		{ SNOR_HWCAPS_ERASE_TYPE_1,		BIT(9) },
		{ SNOR_HWCAPS_ERASE_TYPE_2,		BIT(10) },
		{ SNOR_HWCAPS_ERASE_TYPE_3,		BIT(11) },
		{ SNOR_HWCAPS_ERASE_TYPE_4,		BIT(12) },
	};
	uint32_t *dwords;
	uint32_t len;
	uint32_t addr, discard_hwcaps, read_hwcaps, pp_hwcaps, erase_hwcaps, erase_mask;
	uint32_t i, ret = 0;

	if (param_header->major != SFDP_JESD216_MAJOR ||
			param_header->length < SFDP_4BAIT_DWORD_MAX) {
		log_info("wrong 4B address table.\n");
		return ERR_INVALID_4B_ADDR_TABLE;
	}

	/* Read the 4-byte Address Instruction Table. */
	len = sizeof(*dwords) * SFDP_4BAIT_DWORD_MAX;

	/* Use a kmalloc'ed bounce buffer to guarantee it is DMA-able. */
	dwords = pvPortMalloc(len);
	if (!dwords)
		return ERR_INVALID_PTR;

	addr = SFDP_PARAM_HEADER_PTP(param_header);
	spi_nor_read_sfdp(flash, addr, len, dwords);

	/* Fix endianness of the 4BAIT DWORDs. */
	/*
	for (i = 0; i < SFDP_4BAIT_DWORD_MAX; i++)
		dwords[i] = dwords[i];
	*/
	/*
	 * Compute the subset of (Fast) Read commands for which the 4-byte
	 * version is supported.
	 */
	discard_hwcaps = 0;
	read_hwcaps = 0;
	for (i = 0; i < ARRAY_SIZE(reads); i++) {
		const struct sfdp_4bait *read = &reads[i];

		discard_hwcaps |= read->hwcaps;
		if ((params->hwcaps & read->hwcaps) &&
				(dwords[0] & read->supported_bit))
			read_hwcaps |= read->hwcaps;
	}

	/*
	 * Compute the subset of Page Program commands for which the 4-byte
	 * version is supported.
	 */
	pp_hwcaps = 0;
	for (i = 0; i < ARRAY_SIZE(programs); i++) {
		const struct sfdp_4bait *program = &programs[i];

		/*
		 * The 4 Byte Address Instruction (Optional) Table is the only
		 * SFDP table that indicates support for Page Program Commands.
		 * Bypass the params->hwcaps.mask and consider 4BAIT the biggest
		 * authority for specifying Page Program support.
		 */
		discard_hwcaps |= program->hwcaps;
		if (dwords[0] & program->supported_bit)
			pp_hwcaps |= program->hwcaps;
	}

	/*
	 * Compute the subset of Sector Erase commands for which the 4-byte
	 * version is supported.
	 */
	erase_mask = 0;
	erase_hwcaps = 0;
	for (i = 0; i < SNOR_ERASE_TYPE_MAX; i++) {
		const struct sfdp_4bait *erase = &erases[i + 1];

		discard_hwcaps |= erase->hwcaps;
		if ((params->hwcaps & erase->hwcaps) && \
				(dwords[0] & erase->supported_bit)) {
			erase_mask |= BIT(i + 1);
			erase_hwcaps |= erase->hwcaps;
		}
	}

	/*
	 * We need at least one 4-byte op code per read, program and erase
	 * operation; the .read(), .write() and .erase() hooks share the
	 * nor->addr_width value.
	 */
	if (!read_hwcaps || !pp_hwcaps || !erase_hwcaps)
		goto end;

	/*
	 * Discard all operations from the 4-byte instruction set which are
	 * not supported by this memory.
	 */
	params->hwcaps &= ~discard_hwcaps;
	params->hwcaps |= (read_hwcaps | pp_hwcaps | erase_hwcaps);

	/* Use the 4-byte address instruction set. */
	for (i = 0; i < SNOR_CMD_READ_MAX; i++) {
		struct spi_nor_read_cmd *read_cmd = &params->reads[i];

		read_cmd->opcode = spi_nor_convert_3to4_read(read_cmd->opcode);
	}

	/* 4BAIT is the only SFDP table that indicates page program support. */
	if (pp_hwcaps & SNOR_HWCAPS_PP)
		spi_nor_set_pp_settings(&params->page_programs[SNOR_CMD_PP],
				SPINOR_OP_PP_4B,
				SNOR_PROTO_1_1_1);

	if (pp_hwcaps & SNOR_HWCAPS_PP_1_1_4)
		spi_nor_set_pp_settings(&params->page_programs[SNOR_CMD_PP_1_1_4],
				SPINOR_OP_PP_1_1_4_4B,
				SNOR_PROTO_1_1_4);

	if (pp_hwcaps & SNOR_HWCAPS_PP_1_4_4)
		spi_nor_set_pp_settings(&params->page_programs[SNOR_CMD_PP_1_4_4],
				SPINOR_OP_PP_1_4_4_4B,
				SNOR_PROTO_1_4_4);

	for (i = 0; i < SNOR_ERASE_TYPE_MAX; i++) {
		uint8_t opcode;
		if (erase_mask & BIT(i + 1)) {
			opcode = (dwords[1] >> (i * 8)) & 0xFF;
			spi_nor_set_erase_settings(&params->erases[i + 1],
					opcode,
					params->erases[i + 1].erase_sz);
		} else {
			spi_nor_set_erase_settings(&params->erases[i + 1], 0, 0);
		}
	}

	flash->addr_width = 4;

end:
	vPortFree(dwords);
	return ret;
}


static uint32_t spi_nor_parse_sfdp(flash_t *flash,
		struct spi_nor_flash_parameter *params)
{
	uint32_t ret = 0;
	fmc_spi_priv_t *priv = flash->spi_controller;
	chip_t *spi_chip = flash->spi_chip;
	struct sfdp_parameter_header *param_header, *bfpt_header;
	struct sfdp_parameter_header *param_headers = NULL;
	struct sfdp_header header;
	uint32_t psize;
	uint32_t i;

	/* Get the SFDP header. */
	spi_nor_read_sfdp(flash, 0x0, sizeof(header), &header);

	/* Check the SFDP header version. */
	if (header.signature != SFDP_SIGNATURE ||
			header.major != SFDP_JESD216_MAJOR) {
		log_warn("Invalid SFDP signature (%x), %s cs: %d\n",
			header.signature, priv->name, spi_chip->cs);
		return ERR_SPI_NOR_INVALID_SFDP_HDR;
	}

	/*
	 * Verify that the first and only mandatory parameter header is a
	 * Basic Flash Parameter Table header as specified in JESD216.
	 */
	bfpt_header = &header.bfpt_header;
	if (SFDP_PARAM_HEADER_ID(bfpt_header) != SFDP_BFPT_ID ||
			bfpt_header->major != SFDP_JESD216_MAJOR) {
		log_warn("Invalid SFDP header version, %s cs: %d\n",
			priv->name, spi_chip->cs);
		return ERR_SPI_NOR_INVALID_SFDP_HDR;
	}

	/*
	 * Allocate memory then read all parameter headers with a single
	 * Read SFDP command. These parameter headers will actually be parsed
	 * twice: a first time to get the latest revision of the basic flash
	 * parameter table, then a second time to handle the supported optional
	 * tables.
	 */
	if (header.nph) {
		psize = header.nph * sizeof(*param_headers);

		param_headers = pvPortMalloc(psize);
		if (!param_headers)
			return ERR_INVALID_PTR;

		memset(param_headers, 0x0, psize);
		spi_nor_read_sfdp(flash, sizeof(header),
				psize, param_headers);
	}

	/*
	 * Check other parameter headers to get the latest revision of
	 * the basic flash parameter table.
	 */
	for (i = 0; i < header.nph; i++) {
		param_header = &param_headers[i];

		if (SFDP_PARAM_HEADER_ID(param_header) == SFDP_BFPT_ID &&
				param_header->major == SFDP_JESD216_MAJOR &&
				(param_header->minor > bfpt_header->minor ||
				 (param_header->minor == bfpt_header->minor &&
				  param_header->length > bfpt_header->length)))
			bfpt_header = param_header;
	}

	spi_nor_parse_bfpt(flash, bfpt_header, params);

	/* Parse optional parameter tables. */
	for (i = 0; i < header.nph; i++) {
		param_header = &param_headers[i];

		switch (SFDP_PARAM_HEADER_ID(param_header)) {
		case SFDP_4BAIT_ID:
			ret = spi_nor_parse_4bait(flash, param_header, params);
			break;

		default:
			break;
		}

		if (ret) {
			log_warn("Failed to parse optional parameter table: %04x\n",
					SFDP_PARAM_HEADER_ID(param_header));
			/*
			 * Let's not drop all information we extracted so far
			 * if optional table parsers fail. In case of failing,
			 * each optional parser is responsible to roll back to
			 * the previously known spi_nor data.
			 */
			ret = 0;
		}
	}

	vPortFree(param_headers);
	return ret;
}

uint32_t aspeed_flash_param_init(flash_t *flash)
{
	uint32_t ret = 0;
	struct spi_nor_flash_parameter params;
	struct spi_nor_flash_parameter sfdp_params;

	memset(&params, 0x0, sizeof(struct spi_nor_flash_parameter));

	flash->id = aspeed_get_flash_id(flash);
	log_debug("cs: %d, id: %06x\n", flash->spi_chip->cs, flash->id);

	ret = spi_nor_detect_flash_exist_roughly(flash->id);
	if (ret) {
		log_error("cs: %d, flash not found or doesn't exit\n", flash->spi_chip->cs);
		flash->total_sz = 0;
		goto end;
	}

	/* normal read support */
	params.hwcaps |= SNOR_HWCAPS_READ;
	spi_nor_set_read_settings(&params.reads[SNOR_CMD_READ],
			0, 0, SPINOR_OP_READ,
			SNOR_PROTO_1_1_1);

	/* Page Program settings. */
	params.hwcaps |= SNOR_HWCAPS_PP;
	spi_nor_set_pp_settings(&params.page_programs[SNOR_CMD_PP],
			SPINOR_OP_PP, SNOR_PROTO_1_1_1);

	/* currently only support 1-1-4 read */
	if (flash->spi_chip->rx_bus_width == 4) {
		params.hwcaps |= SNOR_HWCAPS_READ_1_1_4;
		spi_nor_set_read_settings(&params.reads[SNOR_CMD_READ_1_1_4],
				0, 8, SPINOR_OP_READ_1_1_4,
				SNOR_PROTO_1_1_4);
	}

	/* currently only support 1-1-4 pp */
	if (flash->spi_chip->tx_bus_width == 4) {
		params.hwcaps |= SNOR_HWCAPS_PP_1_1_4;
		spi_nor_set_pp_settings(&params.page_programs[SNOR_CMD_PP_1_1_4],
			SPINOR_OP_PP_1_1_4, SNOR_PROTO_1_1_4);
	}

	/* set default erase type */
	params.hwcaps |= SNOR_HWCAPS_ERASE_4K;
	spi_nor_set_erase_settings(&params.erases[SNOR_CMD_ERASE_4K],
			SPINOR_OP_BE_4K, 4 * 1024);

	aspeed_get_flash_param_from_ids(flash, flash->id, &params);

	flash->page_sz = 256;
	flash->addr_width = 3;
	if ((16 * 1024 * 1024) < flash->total_sz) {
		flash->addr_width = 4;
		flash->enter_4byte = true;
	}

	memcpy(&sfdp_params, &params, sizeof(sfdp_params));

	if (spi_nor_parse_sfdp(flash, &sfdp_params) == 0)
		memcpy(&params, &sfdp_params, sizeof(params));

	params.hwcaps &= aspeed_spi_driver_caps();
	if ((params.hwcaps & SNOR_HWCAPS_READ_MASK) == 0) {
		log_error("cs %d: no read cmd code.\n", flash->spi_chip->cs);
		return ERR_READ_CMD_NOT_FOUND;
	}

	if ((params.hwcaps & SNOR_HWCAPS_PP_MASK) == 0) {
		log_error("cs %d: no write cmd code.\n", flash->spi_chip->cs);
		return ERR_WRITE_CMD_NOT_FOUND;
	}

	if ((params.hwcaps & SNOR_HWCAPS_ERASE_MASK) == 0) {
		log_error("cs %d: no erase cmd code.\n", flash->spi_chip->cs);
		return ERR_ERASE_CMD_NOT_FOUND;
	}

	ret = spi_nor_select_read(flash, &params, params.hwcaps);
	if (ret)
		goto end;

	ret = spi_nor_select_pp(flash, &params, params.hwcaps);
	if (ret)
		goto end;

	ret = spi_nor_select_erase(flash, &params, params.hwcaps);
	if (ret)
		goto end;

end:
	return ret;
}

uint32_t aspeed_flash_probe(spi_t *spi, uint32_t cs)
{
	uint32_t ret = 0;
	fmc_spi_priv_t *priv = spi->device->private;
	flash_t *flash;

	ret = aspeed_spi_init(spi);
	if (ret)
		goto end;

	/* get flash info */
	if (priv->chipes[cs].enable == false ||
		priv->chipes[cs].flash_component == false)
		goto end;

	if (priv->chipes[cs].probed)
		goto end;

	priv->chipes[cs].ctx = pvPortMalloc(sizeof(flash_t));
	memset(priv->chipes[cs].ctx, 0x0, sizeof(flash_t));

	flash = (flash_t *)priv->chipes[cs].ctx;
	flash->spi_controller = priv;
	flash->spi_chip = &priv->chipes[cs];
	ret = aspeed_flash_param_init(flash);
	if (ret) {
		log_error("%s ret = 0x%x\n", __func__, ret);
		goto end;
	}

	log_info("ctrl: 0x%08x, cs: %d\n", flash->spi_controller->ctrl_base, flash->spi_chip->cs);
	log_info("id: %06x, size: %dM, rop: %02x, rdummy: %02x, wop: %02x, eop: %02x\n",
			flash->id, flash->total_sz / (1024 * 1024), flash->read_cmd, flash->read_dummy,
			flash->write_cmd, flash->erase_cmd);
	log_info("sector_sz: %x, pg_sz: %x, width: %x\n",
			flash->sector_sz, flash->page_sz, flash->addr_width);

	ret = aspeed_spi_decode_range_reinit(flash);
	if (ret)
		log_error("%s ret = 0x%x\n", __func__, ret);

	/* config for cmd read mode */
	ret = aspeed_spi_flash_info_deploy(flash);
	if (ret)
		log_error("%s ret = 0x%x\n", __func__, ret);

	if (flash->quad_enable) {
		log_debug("enter quad mode\n");
		flash->quad_enable(flash);
	}

	if (flash->enter_4byte)
		set_4byte(flash, true);

	if (flash->addr_width == 4)
		aspeed_spi_set_4byte_mode(flash);

	if (strcmp(priv->name, "spi1") == 0 && cs == 0)
		aspeed_spi_fill_safs_cmd(spi, *flash);

	aspeed_spi_timing_calibration(flash);

	priv->chipes[cs].probed = true;

end:
	if (ret)
		log_error("%s ret = 0x%x\n", __func__, ret);

	return ret;
}

void aspeed_spi_flash_write(flash_t *flash, uint32_t to,
		uint32_t len, uint8_t *write_buf)
{
	write_enable(flash);

	aspeed_spi_flash_write_internal(flash, to, len, write_buf);

	spi_nor_wait_ready(flash);

	write_disable(flash);

	return;
}

void aspeed_spi_flash_erase(flash_t *flash, uint32_t offset)
{
	write_enable(flash);

	aspeed_spi_start_user(flash);
	aspeed_spi_send_cmd_addr(flash, flash->erase_cmd, offset);
	aspeed_spi_stop_user(flash);

	spi_nor_wait_ready(flash);

	write_disable(flash);

	return;
}

