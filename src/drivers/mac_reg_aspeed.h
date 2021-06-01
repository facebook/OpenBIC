#include <stdint.h>
#include "hal_def.h"

typedef union
{
	volatile uint32_t value;
	struct
	{
		volatile uint32_t rpoll_cnt:4; 		/*[3-0]*/ 
		volatile uint32_t rpoll_time_sel:1; /*[4-4]*/ 
		volatile uint32_t :3; 				/*[7-5]*/ 
		volatile uint32_t tpoll_cnt:4; 		/*[11-8]*/ 
		volatile uint32_t tpoll_time_sel:1; /*[12-12]*/  
		volatile uint32_t :19; 				/*[31-13]*/ 
	} fields;
} aptc_t; /* 00000034 */

typedef union
{
	volatile uint32_t value;
	struct
	{
		volatile uint32_t :8; 			/*[7-0]*/  
		volatile uint32_t rbst_size:2; /*[9-8]*/ 
		volatile uint32_t tbst_size:2; /*[11-10]*/ 
		volatile uint32_t rdes_size:4; 	/*[15-12]*/ 
		volatile uint32_t tdes_size:4; 	/*[19-16]*/ 
		volatile uint32_t ifg_cnt:3; 	/*[22-20]*/ 
		volatile uint32_t ifg_inc:1; 	/*[23-23]*/ 
		volatile uint32_t :8; 			/*[31-24]*/ 
	} fields;
} dblac_t; /* 00000038 */

typedef union {
	volatile uint32_t value;
	struct {
		volatile uint32_t txdma_en			: 1;	/* bit[0] */
		volatile uint32_t rxdma_en			: 1;	/* bit[1] */
		volatile uint32_t txmac_en			: 1;	/* bit[2] */
		volatile uint32_t rxmac_en			: 1;	/* bit[3] */
		volatile uint32_t rm_vlan			: 1;	/* bit[4] */
		volatile uint32_t hptxr_en			: 1;	/* bit[5] */
		volatile uint32_t phy_link_sts_dtct	: 1;	/* bit[6] */
		volatile uint32_t enrx_in_halftx	: 1;	/* bit[7] */
		volatile uint32_t fulldup			: 1;	/* bit[8] */
		volatile uint32_t gmac_mode			: 1;	/* bit[9] */
		volatile uint32_t crc_apd			: 1;	/* bit[10] */
		volatile uint32_t reserved_1		: 1;	/* bit[11] */
		volatile uint32_t rx_runt			: 1;	/* bit[12] */
		volatile uint32_t jumbo_lf			: 1;	/* bit[13] */
		volatile uint32_t rx_alladr			: 1;	/* bit[14] */
		volatile uint32_t rx_ht_en			: 1;	/* bit[15] */
		volatile uint32_t rx_multipkt_en	: 1;	/* bit[16] */
		volatile uint32_t rx_broadpkt_en	: 1;	/* bit[17] */
		volatile uint32_t discard_crcerr	: 1;	/* bit[18] */
		volatile uint32_t speed_100			: 1;	/* bit[19] */
		volatile uint32_t reserved_0		: 11;	/* bit[30:20] */
		volatile uint32_t sw_rst			: 1;	/* bit[31] */
	} fields;
} maccr_t;	/* 00000050 */

typedef struct mac_register_s {
	volatile uint32_t isr;			/* 0x00 */
	volatile uint32_t ier;			/* 0x04 */
	volatile uint32_t mac_madr;		/* 0x08 */
	volatile uint32_t mac_ladr;		/* 0x0c */
	volatile uint32_t maht0;		/* 0x10 */
	volatile uint32_t maht1;		/* 0x14 */
	volatile uint32_t txpd;			/* 0x18 */
	volatile uint32_t rxpd;			/* 0x1c */
	volatile uint32_t txr_badr;		/* 0x20 */
	volatile uint32_t rxr_badr;		/* 0x24 */
	volatile uint32_t hptxpd;		/* 0x28 */
	volatile uint32_t hptxpd_badr;	/* 0x2c */
	volatile uint32_t itc;			/* 0x30 */
	volatile aptc_t	aptc;			/* 0x34 */
	volatile dblac_t dblac;			/* 0x38 */
	volatile uint32_t dbgsts;		/* 0x3c */
	volatile uint32_t fear;			/* 0x40 */
	volatile uint32_t resv0;		/* 0x44 */
	volatile uint32_t tpafcr;		/* 0x48 */
	volatile uint32_t rbsr;			/* 0x4c */
	volatile maccr_t maccr;			/* 0x50 */
	volatile uint32_t macsr;		/* 0x54 */
	volatile uint32_t tm;			/* 0x58 */
	volatile uint32_t resv1[3];		/* 0x5c - 0x64 */ /* not defined in spec */
	volatile uint32_t fcr;			/* 0x68 */
	volatile uint32_t bpr;			/* 0x6c */
	volatile uint32_t wolcr;		/* 0x70 */
	volatile uint32_t wolsr;		/* 0x74 */
	volatile uint32_t mac_madr1;	/* 0x78 */
	volatile uint32_t mac_ladr1;	/* 0x7c */
	volatile uint32_t mac_madr2;	/* 0x80 */
	volatile uint32_t mac_ladr2;	/* 0x84 */
	volatile uint32_t mac_madr3;	/* 0x88 */
	volatile uint32_t mac_ladr3;	/* 0x8c */
	volatile uint32_t nptxr_ptr;	/* 0x90 */
	volatile uint32_t rxr_ptr;		/* 0x94 */
	volatile uint32_t resv3;		/* 0x98 */ /* not defined in spec */
	volatile uint32_t hptxr_ptr;	/* 0x9c */
	volatile uint32_t tx_cnt[3];	/* 0xa0 - 0xa8 */
	volatile uint32_t rx_cnt[8];	/* 0xac - 0xc8 */
} mac_register_t;

#define MAC_TXDES0_TXDMA_OWN		BIT(31)
#define MAC_TXDES0_EDOTR			BIT(30)
#define MAC_TXDES0_FTS				BIT(29)
#define MAC_TXDES0_LTS				BIT(28)
#define MAC_TXDES0_CRC_ERR			BIT(19)
#define MAC_TXDES0_TXBUF_SIZE(x)	((x) & 0x3fff)

#define MAC_RXDES0_RXPKT_RDY		BIT(31)
#define MAC_RXDES0_EDORR			BIT(30)
#define MAC_RXDES0_FRS				BIT(29)
#define MAC_RXDES0_LRS				BIT(28)
#define MAC_RXDES0_PAUSE_FRAME		BIT(25)
#define MAC_RXDES0_PAUSE_OPCODE		BIT(24)
#define MAC_RXDES0_FIFO_FULL		BIT(23)
#define MAC_RXDES0_RX_ODD_NB		BIT(22)
#define MAC_RXDES0_RUNT				BIT(21)
#define MAC_RXDES0_FTL				BIT(20)
#define MAC_RXDES0_CRC_ERR			BIT(19)
#define MAC_RXDES0_RX_ERR			BIT(18)
#define MAC_RXDES0_BROADCAST		BIT(17)
#define MAC_RXDES0_MULTICAST		BIT(16)
#define MAC_RXDES0_VDBC(x)			((x) & 0x3fff)