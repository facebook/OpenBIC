#ifndef _ESPI_ASPEED_H_
#define _ESPI_ASPEED_H_
#include "objects.h"

enum espi_chan {
	ESPI_CH_PERI,
	ESPI_CH_VW,
	ESPI_CH_OOB,
	ESPI_CH_FLASH,
	ESPI_CH_NUM
};

enum espi_safs_mode {
	ESPI_SAFS_MIX,
	ESPI_SAFS_SW,
	ESPI_SAFS_HW,
	ESPI_SAFS_MODES,
};

typedef struct aspeed_espi_priv_s {

	uint32_t delay_timing;

	struct {
		uint32_t host_map_addr;
		uint32_t host_map_size;
		bool dma_mode;
	} perif;

	struct {
		bool dma_mode;
	} oob;

	struct {
		uint32_t safs_mode;
		bool dma_mode;
	} flash;

} aspeed_espi_priv_t;

/*
 * eSPI cycle type encoding
 *
 * Section 5.1 Cycle Types and Packet Format,
 * Intel eSPI Interface Base Specification, Rev 1.0, Jan. 2016.
 */
#define ESPI_PERIF_MEMRD32		0x00
#define ESPI_PERIF_MEMRD64		0x02
#define ESPI_PERIF_MEMWR32		0x01
#define ESPI_PERIF_MEMWR64		0x03
#define ESPI_PERIF_MSG			0x10
#define ESPI_PERIF_MSG_D		0x11
#define ESPI_PERIF_SUC_CMPLT		0x06
#define ESPI_PERIF_SUC_CMPLT_D_MIDDLE	0x09
#define ESPI_PERIF_SUC_CMPLT_D_FIRST	0x0b
#define ESPI_PERIF_SUC_CMPLT_D_LAST	0x0d
#define ESPI_PERIF_SUC_CMPLT_D_ONLY	0x0f
#define ESPI_PERIF_UNSUC_CMPLT		0x0c
#define ESPI_OOB_MSG			0x21
#define ESPI_FLASH_READ			0x00
#define ESPI_FLASH_WRITE		0x01
#define ESPI_FLASH_ERASE		0x02
#define ESPI_FLASH_SUC_CMPLT		0x06
#define ESPI_FLASH_SUC_CMPLT_D_MIDDLE	0x09
#define ESPI_FLASH_SUC_CMPLT_D_FIRST	0x0b
#define ESPI_FLASH_SUC_CMPLT_D_LAST	0x0d
#define ESPI_FLASH_SUC_CMPLT_D_ONLY	0x0f
#define ESPI_FLASH_UNSUC_CMPLT		0x0c

/*
 * eSPI packet format structure
 *
 * Section 5.1 Cycle Types and Packet Format,
 * Intel eSPI Interface Base Specification, Rev 1.0, Jan. 2016.
 */
struct espi_comm_hdr {
	uint8_t cyc;
	uint8_t len_h : 4;
	uint8_t tag : 4;
	uint8_t len_l;
} __attribute__((packed));

struct espi_perif_mem32 {
	uint8_t cyc;
	uint8_t len_h : 4;
	uint8_t tag : 4;
	uint8_t len_l;
	uint32_t addr_be;
	uint8_t data[];
} __attribute__((packed));

struct espi_perif_mem64 {
	uint8_t cyc;
	uint8_t len_h : 4;
	uint8_t tag : 4;
	uint8_t len_l;
	uint32_t addr_be;
	uint8_t data[];
} __attribute__((packed));

struct espi_perif_msg {
	uint8_t cyc;
	uint8_t len_h : 4;
	uint8_t tag : 4;
	uint8_t len_l;
	uint8_t msg_code;
	uint8_t msg_byte[4];
	uint8_t data[];
} __attribute__((packed));

struct espi_perif_cmplt {
	uint8_t cyc;
	uint8_t len_h : 4;
	uint8_t tag : 4;
	uint8_t len_l;
	uint8_t data[];
} __attribute__((packed));

struct espi_oob_msg {
	uint8_t cyc;
	uint8_t len_h : 4;
	uint8_t tag : 4;
	uint8_t len_l;
	uint8_t data[];
};

struct espi_flash_rwe {
	uint8_t cyc;
	uint8_t len_h : 4;
	uint8_t tag : 4;
	uint8_t len_l;
	uint32_t addr_be;
	uint8_t data[];
} __attribute__((packed));

struct espi_flash_cmplt {
	uint8_t cyc;
	uint8_t len_h : 4;
	uint8_t tag : 4;
	uint8_t len_l;
	uint8_t data[];
} __attribute__((packed));

struct aspeed_espi_xfer {
	uint32_t pkt_len;
	uint8_t *pkt;
};

/*
 * we choose the longest header and the max payload size
 * based on the Intel specification to define the maximum
 * eSPI packet length
 */
#define ESPI_PLD_LEN_MIN	(1UL << 6)
#define ESPI_PLD_LEN_MAX	(1UL << 12)
#define ESPI_PKT_LEN_MAX	(sizeof(struct espi_perif_msg) + ESPI_PLD_LEN_MAX)

/* NOTE that these exported interfaces are NOT re-entrant */
int aspeed_espi_perif_pc_get_rx(struct aspeed_espi_xfer *xfer);
int aspeed_espi_perif_pc_put_tx(struct aspeed_espi_xfer *xfer);
int aspeed_espi_perif_np_put_tx(struct aspeed_espi_xfer *xfer);
int aspeed_espi_oob_get_rx(struct aspeed_espi_xfer *xfer);
int aspeed_espi_oob_put_tx(struct aspeed_espi_xfer *xfer);
int aspeed_espi_flash_get_rx(struct aspeed_espi_xfer *xfer);
int aspeed_espi_flash_put_tx(struct aspeed_espi_xfer *xfer);

void aspeed_espi_init(struct espi_s *obj);

#endif
