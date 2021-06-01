#ifndef _SNOOP_ASPEED_H_
#define _SNOOP_ASPEED_H_
#include "objects.h"

#define ASPEED_SNOOP_CHANNEL_NUM	2
#define ASPEED_SNOOP_MSG_QUEUE_SIZE	256

typedef struct aspeed_snoop_priv_s {
	struct {
		uint16_t port_addr;
		bool enable;
	} chan[ASPEED_SNOOP_CHANNEL_NUM];
} aspeed_snoop_priv_t;

int aspeed_snoop_read(snoop_t *snoop, uint32_t chan, uint8_t *buf, uint32_t buf_sz);
void aspeed_snoop_init(snoop_t *snoop);

#endif
