#ifndef _PCC_ASPEED_H_
#define _PCC_ASPEED_H_
#include "objects.h"

#define ASPEED_PCC_DMA_SIZE	0x1000
#define ASPEED_PCC_FIFO_SIZE	0x100

enum aspeed_pcc_record_mode {
	PCC_REC_1B,
	PCC_REC_2B,
	PCC_REC_4B,
	PCC_REC_FULL,
};

enum aspeed_pcc_hbits_select {
	PCC_HBITS_SEL_NONE,
	PCC_HBITS_SEL_45,
	PCC_HBITS_SEL_67,
	PCC_HBITS_SEL_89,
};

typedef struct aspeed_pcc_priv_s {
	uint32_t addr;
	uint32_t addr_xbit;
	uint32_t addr_hbit_sel;
	uint32_t rec_mode;
	bool dma_mode;
} aspeed_pcc_priv_t;

int aspeed_pcc_read(pcc_t *pcc, uint8_t *buf, uint32_t buf_sz);
void aspeed_pcc_init(pcc_t *pcc);

#endif
