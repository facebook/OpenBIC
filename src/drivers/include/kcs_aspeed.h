#ifndef _KCS_ASPEED_H_
#define _KCS_ASPEED_H_
#include "objects.h"

#define ASPEED_KCS_BUF_SIZE	0x100

enum aspeed_kcs_chan {
	KCS_CH1,
	KCS_CH2,
	KCS_CH3,
	KCS_CH4,
	KCS_CH_NUM
};

typedef struct aspeed_kcs_priv_s {
	uint32_t chan;
	uint32_t addr;
} aspeed_kcs_priv_t;

int aspeed_kcs_read(kcs_t *kcs, uint8_t *buf, uint32_t buf_sz);
int aspeed_kcs_write(kcs_t *kcs, uint8_t *buf, uint32_t buf_sz);
void aspeed_kcs_init(kcs_t *kcs);

#endif
