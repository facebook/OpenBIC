
#ifndef __FLASH_H__
#define __FLASH_H__

#include "common.h"
#include "fmc_spi_aspeed.h"
#include "objects.h"

#define SNOR_MFR_SPANSION   0x0001
#define SNOR_MFR_MACRONIX   0x00C2
#define SNOR_MFR_ST         0x0020 /* STMicroelectronics */
#define SNOR_MFR_MICRON     0x002C /* Micron */
#define SNOR_MFR_WINBOND    0x00ef

#define JEDEC_MFR(id)       ((id & 0xff0000) >> 16)

/* Flash opcodes. */
#define SPINOR_OP_WREN		0x06	/* Write enable */
#define SPINOR_OP_WRDI		0x04	/* Write disable */
#define SPINOR_OP_RDSR		0x05	/* Read status register */
#define SPINOR_OP_WRSR		0x01	/* Write status register 1 byte */
#define SPINOR_OP_RDSR2		0x3f	/* Read status register 2 */
#define SPINOR_OP_WRSR2		0x3e	/* Write status register 2 */
#define SPINOR_OP_READ		0x03	/* Read data bytes (low frequency) */
#define SPINOR_OP_READ_FAST	0x0b	/* Read data bytes (high frequency) */
#define SPINOR_OP_READ_1_1_2	0x3b	/* Read data bytes (Dual Output SPI) */
#define SPINOR_OP_READ_1_2_2	0xbb	/* Read data bytes (Dual I/O SPI) */
#define SPINOR_OP_READ_1_1_4	0x6b	/* Read data bytes (Quad Output SPI) */
#define SPINOR_OP_READ_1_4_4	0xeb	/* Read data bytes (Quad I/O SPI) */
#define SPINOR_OP_READ_1_1_8	0x8b	/* Read data bytes (Octal Output SPI) */
#define SPINOR_OP_READ_1_8_8	0xcb	/* Read data bytes (Octal I/O SPI) */
#define SPINOR_OP_PP		0x02	/* Page program (up to 256 bytes) */
#define SPINOR_OP_PP_1_1_4	0x32	/* Quad page program */
#define SPINOR_OP_PP_1_4_4	0x38	/* Quad page program */
#define SPINOR_OP_PP_1_1_8	0x82	/* Octal page program */
#define SPINOR_OP_PP_1_8_8	0xc2	/* Octal page program */
#define SPINOR_OP_BE_4K		0x20	/* Erase 4KiB block */
#define SPINOR_OP_BE_4K_PMC	0xd7	/* Erase 4KiB block on PMC chips */
#define SPINOR_OP_BE_32K	0x52	/* Erase 32KiB block */
#define SPINOR_OP_CHIP_ERASE	0xc7	/* Erase whole flash chip */
#define SPINOR_OP_SE		0xd8	/* Sector erase (usually 64KiB) */
#define SPINOR_OP_RDID		0x9f	/* Read JEDEC ID */
#define SPINOR_OP_RDSFDP	0x5a	/* Read SFDP */
#define SPINOR_OP_RDCR		0x35	/* Read configuration register */
#define SPINOR_OP_RDFSR		0x70	/* Read flag status register */
#define SPINOR_OP_CLFSR		0x50	/* Clear flag status register */
#define SPINOR_OP_RDEAR		0xc8	/* Read Extended Address Register */
#define SPINOR_OP_WREAR		0xc5	/* Write Extended Address Register */

/* 4-byte address opcodes - used on Spansion and some Macronix flashes. */
#define SPINOR_OP_READ_4B	0x13	/* Read data bytes (low frequency) */
#define SPINOR_OP_READ_FAST_4B	0x0c	/* Read data bytes (high frequency) */
#define SPINOR_OP_READ_1_1_2_4B	0x3c	/* Read data bytes (Dual Output SPI) */
#define SPINOR_OP_READ_1_2_2_4B	0xbc	/* Read data bytes (Dual I/O SPI) */
#define SPINOR_OP_READ_1_1_4_4B	0x6c	/* Read data bytes (Quad Output SPI) */
#define SPINOR_OP_READ_1_4_4_4B	0xec	/* Read data bytes (Quad I/O SPI) */
#define SPINOR_OP_READ_1_1_8_4B	0x7c	/* Read data bytes (Octal Output SPI) */
#define SPINOR_OP_READ_1_8_8_4B	0xcc	/* Read data bytes (Octal I/O SPI) */
#define SPINOR_OP_PP_4B		0x12	/* Page program (up to 256 bytes) */
#define SPINOR_OP_PP_1_1_4_4B	0x34	/* Quad page program */
#define SPINOR_OP_PP_1_4_4_4B	0x3e	/* Quad page program */
#define SPINOR_OP_PP_1_1_8_4B	0x84	/* Octal page program */
#define SPINOR_OP_PP_1_8_8_4B	0x8e	/* Octal page program */
#define SPINOR_OP_BE_4K_4B	0x21	/* Erase 4KiB block */
#define SPINOR_OP_BE_32K_4B	0x5c	/* Erase 32KiB block */
#define SPINOR_OP_SE_4B		0xdc	/* Sector erase (usually 64KiB) */

/* Double Transfer Rate opcodes - defined in JEDEC JESD216B. */
#define SPINOR_OP_READ_1_1_1_DTR	0x0d
#define SPINOR_OP_READ_1_2_2_DTR	0xbd
#define SPINOR_OP_READ_1_4_4_DTR	0xed

#define SPINOR_OP_READ_1_1_1_DTR_4B	0x0e
#define SPINOR_OP_READ_1_2_2_DTR_4B	0xbe
#define SPINOR_OP_READ_1_4_4_DTR_4B	0xee


/* Used for Macronix and Winbond flashes. */
#define SPINOR_OP_EN4B		0xb7	/* Enter 4-byte mode */
#define SPINOR_OP_EX4B		0xe9	/* Exit 4-byte mode */


/* Used for Spansion flashes only. */
#define SPINOR_OP_BRWR		0x17	/* Bank register write */
#define SPINOR_OP_CLSR		0x30	/* Clear status register 1 */


/* Configuration Register bits. */
#define CR_QUAD_EN_SPAN		BIT(1)	/* Spansion Quad I/O */

/* Status Register bits. */
#define SR_WIP			BIT(0)	/* Write in progress */
#define SR_WEL			BIT(1)	/* Write enable latch */

#define SR_QUAD_EN_MX		BIT(6)	/* Macronix Quad I/O */


/* flash info control flags */
#define SECT_4K			BIT(0)	/* SPINOR_OP_BE_4K works uniformly */
#define SPI_NOR_NO_ERASE	BIT(1)	/* No erase command needed */
#define SST_WRITE		BIT(2)	/* use SST byte programming */
#define SPI_NOR_NO_FR		BIT(3)	/* Can't do fastread */
#define SECT_4K_PMC		BIT(4)	/* SPINOR_OP_BE_4K_PMC works uniformly */
#define SPI_NOR_DUAL_READ	BIT(5)	/* Flash supports Dual Read */
#define SPI_NOR_QUAD_READ	BIT(6)	/* Flash supports Quad Read */
#define USE_FSR			BIT(7)	/* use flag status register */
#define SPI_NOR_HAS_LOCK	BIT(8)	/* Flash supports lock/unlock via SR */
#define SPI_NOR_HAS_TB		BIT(9)	/*
									 * Flash SR has Top/Bottom (TB) protect
									 * bit. Must be used with
									 * SPI_NOR_HAS_LOCK.
									 */
#define	SPI_S3AN		BIT(10)	/*
								 * Xilinx Spartan 3AN In-System Flash
								 * (MFR cannot be used for probing
								 * because it has the same value as
								 * ATMEL flashes)
								 */
#define SPI_NOR_4B_OPCODES	BIT(11)	/*
									 * Use dedicated 4byte address op codes
									 * to support memory size above 128Mib.
									 */
#define NO_CHIP_ERASE		BIT(12) /* Chip does not support chip erase */
#define SPI_NOR_SKIP_SFDP	BIT(13)	/* Skip parsing of SFDP tables */
#define USE_CLSR		BIT(14)	/* use CLSR command */
#define SPI_NOR_OCTAL_READ	BIT(15)	/* Flash supports Octal Read */

/* flash capability control */
#define SNOR_HWCAPS_READ_MASK       GENMASK(14, 0)
#define SNOR_HWCAPS_READ            BIT(0)
#define SNOR_HWCAPS_READ_FAST       BIT(1)
#define SNOR_HWCAPS_READ_1_1_1_DTR  BIT(2)

#define SNOR_HWCAPS_READ_DUAL       GENMASK(6, 3)
#define SNOR_HWCAPS_READ_1_1_2      BIT(3)
#define SNOR_HWCAPS_READ_1_2_2      BIT(4)
#define SNOR_HWCAPS_READ_2_2_2      BIT(5)
#define SNOR_HWCAPS_READ_1_2_2_DTR  BIT(6)

#define SNOR_HWCAPS_READ_QUAD       GENMASK(10, 7)
#define SNOR_HWCAPS_READ_1_1_4      BIT(7)
#define SNOR_HWCAPS_READ_1_4_4      BIT(8)
#define SNOR_HWCAPS_READ_4_4_4      BIT(9)
#define SNOR_HWCAPS_READ_1_4_4_DTR  BIT(10)

#define SNOR_HWCAPS_READ_OCTAL      GENMASK(14, 11)
#define SNOR_HWCAPS_READ_1_1_8      BIT(11)
#define SNOR_HWCAPS_READ_1_8_8      BIT(12)
#define SNOR_HWCAPS_READ_8_8_8      BIT(13)
#define SNOR_HWCAPS_READ_1_8_8_DTR  BIT(14)

#define SNOR_HWCAPS_PP_MASK         GENMASK(22, 16)
#define SNOR_HWCAPS_PP              BIT(16)

#define SNOR_HWCAPS_PP_QUAD         GENMASK(19, 17)
#define SNOR_HWCAPS_PP_1_1_4        BIT(17)
#define SNOR_HWCAPS_PP_1_4_4        BIT(18)
#define SNOR_HWCAPS_PP_4_4_4        BIT(19)

#define SNOR_HWCAPS_PP_OCTAL        GENMASK(22, 20)
#define SNOR_HWCAPS_PP_1_1_8        BIT(20)
#define SNOR_HWCAPS_PP_1_8_8        BIT(21)
#define SNOR_HWCAPS_PP_8_8_8        BIT(22)

#define SNOR_HWCAPS_ERASE_MASK      GENMASK(27, 23)
#define SNOR_HWCAPS_ERASE_4K        BIT(23)
#define SNOR_HWCAPS_ERASE_TYPE_1    BIT(24)
#define SNOR_HWCAPS_ERASE_TYPE_2    BIT(25)
#define SNOR_HWCAPS_ERASE_TYPE_3    BIT(26)
#define SNOR_HWCAPS_ERASE_TYPE_4    BIT(27)

/* Supported SPI protocols */
#define SNOR_PROTO_INST_MASK	GENMASK(23, 16)
#define SNOR_PROTO_INST_SHIFT	16
#define SNOR_PROTO_INST(_nbits)	\
	((((unsigned long)(_nbits)) << SNOR_PROTO_INST_SHIFT) & \
	 SNOR_PROTO_INST_MASK)

#define SNOR_PROTO_ADDR_MASK	GENMASK(15, 8)
#define SNOR_PROTO_ADDR_SHIFT	8
#define SNOR_PROTO_ADDR(_nbits)	\
	((((unsigned long)(_nbits)) << SNOR_PROTO_ADDR_SHIFT) & \
	 SNOR_PROTO_ADDR_MASK)

#define SNOR_PROTO_DATA_MASK	GENMASK(7, 0)
#define SNOR_PROTO_DATA_SHIFT	0
#define SNOR_PROTO_DATA(_nbits)	\
	((((unsigned long)(_nbits)) << SNOR_PROTO_DATA_SHIFT) & \
	 SNOR_PROTO_DATA_MASK)

#define SNOR_PROTO_IS_DTR	BIT(24)	/* Double Transfer Rate */

#define SNOR_PROTO_STR(_inst_nbits, _addr_nbits, _data_nbits)	\
	(SNOR_PROTO_INST(_inst_nbits) |				\
	 SNOR_PROTO_ADDR(_addr_nbits) |				\
	 SNOR_PROTO_DATA(_data_nbits))
#define SNOR_PROTO_DTR(_inst_nbits, _addr_nbits, _data_nbits)	\
	(SNOR_PROTO_IS_DTR |					\
	 SNOR_PROTO_STR(_inst_nbits, _addr_nbits, _data_nbits))


enum spi_nor_protocol {
	SNOR_PROTO_1_1_1 = SNOR_PROTO_STR(1, 1, 1),
	SNOR_PROTO_1_1_2 = SNOR_PROTO_STR(1, 1, 2),
	SNOR_PROTO_1_1_4 = SNOR_PROTO_STR(1, 1, 4),
	SNOR_PROTO_1_1_8 = SNOR_PROTO_STR(1, 1, 8),
	SNOR_PROTO_1_2_2 = SNOR_PROTO_STR(1, 2, 2),
	SNOR_PROTO_1_4_4 = SNOR_PROTO_STR(1, 4, 4),
	SNOR_PROTO_1_8_8 = SNOR_PROTO_STR(1, 8, 8),
	SNOR_PROTO_2_2_2 = SNOR_PROTO_STR(2, 2, 2),
	SNOR_PROTO_4_4_4 = SNOR_PROTO_STR(4, 4, 4),
	SNOR_PROTO_8_8_8 = SNOR_PROTO_STR(8, 8, 8),

	SNOR_PROTO_1_1_1_DTR = SNOR_PROTO_DTR(1, 1, 1),
	SNOR_PROTO_1_2_2_DTR = SNOR_PROTO_DTR(1, 2, 2),
	SNOR_PROTO_1_4_4_DTR = SNOR_PROTO_DTR(1, 4, 4),
	SNOR_PROTO_1_8_8_DTR = SNOR_PROTO_DTR(1, 8, 8),
};

enum spi_nor_read_cmd_idx {
	SNOR_CMD_READ,
	SNOR_CMD_READ_FAST,
	SNOR_CMD_READ_1_1_1_DTR,

	/* Dual SPI */
	SNOR_CMD_READ_1_1_2,
	SNOR_CMD_READ_1_2_2,
	SNOR_CMD_READ_2_2_2,
	SNOR_CMD_READ_1_2_2_DTR,

	/* Quad SPI */
	SNOR_CMD_READ_1_1_4,
	SNOR_CMD_READ_1_4_4,
	SNOR_CMD_READ_4_4_4,
	SNOR_CMD_READ_1_4_4_DTR,

	/* Octal SPI */
	SNOR_CMD_READ_1_1_8,
	SNOR_CMD_READ_1_8_8,
	SNOR_CMD_READ_8_8_8,
	SNOR_CMD_READ_1_8_8_DTR,

	SNOR_CMD_READ_MAX
};

enum spi_nor_pp_cmd_idx {
	SNOR_CMD_PP,

	/* Quad SPI */
	SNOR_CMD_PP_1_1_4,
	SNOR_CMD_PP_1_4_4,
	SNOR_CMD_PP_4_4_4,

	/* Octal SPI */
	SNOR_CMD_PP_1_1_8,
	SNOR_CMD_PP_1_8_8,
	SNOR_CMD_PP_8_8_8,

	SNOR_CMD_PP_MAX
};

enum spi_nor_erase_cmd_idx {
	SNOR_CMD_ERASE_4K,

	SNOR_CMD_ERASE_TYPE_1,
	SNOR_CMD_ERASE_TYPE_2,
	SNOR_CMD_ERASE_TYPE_3,
	SNOR_CMD_ERASE_TYPE_4,

	SNOR_CMD_ERASE_MAX
};

struct spi_nor_read_cmd {
	uint8_t num_mode_clocks;
	uint8_t num_wait_states;
	uint8_t opcode;
	enum spi_nor_protocol proto;
};

struct spi_nor_pp_cmd {
	uint8_t opcode;
	enum spi_nor_protocol proto;
};

struct spi_nor_erase_cmd {
	uint8_t opcode;
	uint32_t erase_sz;
};

struct spi_nor_flash_parameter {
	uint32_t sz;
	uint32_t page_sz;

	uint32_t hwcaps;
	struct spi_nor_read_cmd	reads[SNOR_CMD_READ_MAX];
	struct spi_nor_pp_cmd page_programs[SNOR_CMD_PP_MAX];
	struct spi_nor_erase_cmd erases[SNOR_CMD_ERASE_MAX];
};

/* for sfdp parser */
struct sfdp_parameter_header {
	uint8_t id_lsb;
	uint8_t minor;
	uint8_t major;
	uint8_t length; /* in double words */
	uint8_t parameter_table_pointer[3]; /* byte address */
	uint8_t id_msb;
};

#define SFDP_PARAM_HEADER_ID(p)	(((p)->id_msb << 8) | (p)->id_lsb)
#define SFDP_PARAM_HEADER_PTP(p) \
	(((p)->parameter_table_pointer[2] << 16) | \
	 ((p)->parameter_table_pointer[1] <<  8) | \
	 ((p)->parameter_table_pointer[0] <<  0))

#define SFDP_BFPT_ID		0xff00	/* Basic Flash Parameter Table */
#define SFDP_SECTOR_MAP_ID	0xff81	/* Sector Map Table */
#define SFDP_4BAIT_ID		0xff84  /* 4-byte Address Instruction Table */

#define SFDP_SIGNATURE		0x50444653U
#define SFDP_JESD216_MAJOR	1
#define SFDP_JESD216_MINOR	0
#define SFDP_JESD216A_MINOR	5
#define SFDP_JESD216B_MINOR	6

struct sfdp_header {
	uint32_t signature; /* Ox50444653U <=> "SFDP" */
	uint8_t minor;
	uint8_t major;
	uint8_t nph; /* 0-base number of parameter headers */
	uint8_t unused;
	/* Basic Flash Parameter Table. */
	struct sfdp_parameter_header	bfpt_header;
};

/* Basic Flash Parameter Table */

/*
 * JESD216 rev B defines a Basic Flash Parameter Table of 16 DWORDs.
 * They are indexed from 1 but C arrays are indexed from 0.
 */
#define BFPT_DWORD(i)		((i) - 1)
#define BFPT_DWORD_MAX		16

/* The first version of JESB216 defined only 9 DWORDs. */
#define BFPT_DWORD_MAX_JESD216			9

/* 1st DWORD. */
#define BFPT_DWORD1_FAST_READ_1_1_2		BIT(16)
#define BFPT_DWORD1_ADDRESS_BYTES_MASK		GENMASK(18, 17)
#define BFPT_DWORD1_ADDRESS_BYTES_3_ONLY	(0x0UL << 17)
#define BFPT_DWORD1_ADDRESS_BYTES_3_OR_4	(0x1UL << 17)
#define BFPT_DWORD1_ADDRESS_BYTES_4_ONLY	(0x2UL << 17)
#define BFPT_DWORD1_DTR				BIT(19)
#define BFPT_DWORD1_FAST_READ_1_2_2		BIT(20)
#define BFPT_DWORD1_FAST_READ_1_4_4		BIT(21)
#define BFPT_DWORD1_FAST_READ_1_1_4		BIT(22)

/* 5th DWORD. */
#define BFPT_DWORD5_FAST_READ_2_2_2		BIT(0)
#define BFPT_DWORD5_FAST_READ_4_4_4		BIT(4)

/* 11th DWORD. */
#define BFPT_DWORD11_PAGE_SIZE_SHIFT		4
#define BFPT_DWORD11_PAGE_SIZE_MASK		GENMASK(7, 4)

/* 15th DWORD. */

/*
 * (from JESD216 rev B)
 * Quad Enable Requirements (QER):
 * - 000b: Device does not have a QE bit. Device detects 1-1-4 and 1-4-4
 *         reads based on instruction. DQ3/HOLD# functions are hold during
 *         instruction phase.
 * - 001b: QE is bit 1 of status register 2. It is set via Write Status with
 *         two data bytes where bit 1 of the second byte is one.
 *         [...]
 *         Writing only one byte to the status register has the side-effect of
 *         clearing status register 2, including the QE bit. The 100b code is
 *         used if writing one byte to the status register does not modify
 *         status register 2.
 * - 010b: QE is bit 6 of status register 1. It is set via Write Status with
 *         one data byte where bit 6 is one.
 *         [...]
 * - 011b: QE is bit 7 of status register 2. It is set via Write status
 *         register 2 instruction 3Eh with one data byte where bit 7 is one.
 *         [...]
 *         The status register 2 is read using instruction 3Fh.
 * - 100b: QE is bit 1 of status register 2. It is set via Write Status with
 *         two data bytes where bit 1 of the second byte is one.
 *         [...]
 *         In contrast to the 001b code, writing one byte to the status
 *         register does not modify status register 2.
 * - 101b: QE is bit 1 of status register 2. Status register 1 is read using
 *         Read Status instruction 05h. Status register2 is read using
 *         instruction 35h. QE is set via Write Status instruction 01h with
 *         two data bytes where bit 1 of the second byte is one.
 *         [...]
 */
#define BFPT_DWORD15_QER_MASK			GENMASK(22, 20)
#define BFPT_DWORD15_QER_NONE			(0x0UL << 20) /* Micron */
#define BFPT_DWORD15_QER_SR2_BIT1_BUGGY		(0x1UL << 20)
#define BFPT_DWORD15_QER_SR1_BIT6		(0x2UL << 20) /* Macronix */
#define BFPT_DWORD15_QER_SR2_BIT7		(0x3UL << 20)
#define BFPT_DWORD15_QER_SR2_BIT1_NO_RD		(0x4UL << 20)
#define BFPT_DWORD15_QER_SR2_BIT1		(0x5UL << 20) /* Spansion */

struct sfdp_bfpt {
	uint32_t dwords[BFPT_DWORD_MAX]; /* 16 double word */
};

struct sfdp_bfpt_read {
	/* The Fast Read x-y-z hardware capability in params->hwcaps.mask. */
	uint32_t hwcaps;

	/*
	 * The <supported_bit> bit in <supported_dword> BFPT DWORD tells us
	 * whether the Fast Read x-y-z command is supported.
	 */
	uint32_t supported_dword;
	uint32_t supported_bit;

	/*
	 * The half-word at offset <setting_shift> in <setting_dword> BFPT DWORD
	 * encodes the op code, the number of mode clocks and the number of wait
	 * states to be used by Fast Read x-y-z command.
	 */
	uint32_t settings_dword;
	uint32_t settings_shift;

	/* The SPI protocol for this Fast Read x-y-z command. */
	enum spi_nor_protocol	proto;
};

struct sfdp_bfpt_erase {
	/*
	 * The half-word at offset <shift> in DWORD <dwoard> encodes the
	 * op code and erase sector size to be used by Sector Erase commands.
	 */
	uint32_t hwcaps;
	uint32_t dword;
	uint32_t shift;
};

#define SFDP_4BAIT_DWORD_MAX	2

struct sfdp_4bait {
	/* The hardware capability. */
	uint32_t hwcaps;

	/*
	 * The <supported_bit> bit in DWORD1 of the 4BAIT tells us whether
	 * the associated 4-byte address op code is supported.
	 */
	uint32_t supported_bit;
};

#define SNOR_ERASE_TYPE_MAX	4
#define SNOR_ERASE_TYPE_MASK	GENMASK_ULL(SNOR_ERASE_TYPE_MAX - 1, 0)

typedef struct _flash {
	uint32_t id;
	uint32_t total_sz;
	uint32_t sector_sz;
	uint32_t page_sz;
	uint8_t addr_width;
	uint8_t read_cmd;
	uint8_t write_cmd;
	uint8_t read_dummy;
	uint8_t erase_cmd;
	uint8_t status_wen;
	bool enter_4byte;
	uint8_t cmd_addr_buf[16];
	uint8_t cmd_addr_len;

	uint32_t (*quad_enable)(struct _flash *flash);

	fmc_spi_priv_t *spi_controller;
	chip_t *spi_chip;
} flash_t;

typedef struct {
	char *name;
	uint32_t id;
	uint32_t total_sz;
	uint32_t sector_sz;
	uint32_t flag;
} flash_info_t;

extern const flash_info_t flash_ids[];

#endif
