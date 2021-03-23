#ifndef _BT_ASPEED_H_
#define _BT_ASPEED_H_
#include "objects.h"

#define ASPEED_BT_BUF_SIZE	0x100

typedef struct aspeed_bt_priv_s {
	uint32_t addr;
	uint32_t sirq;
} aspeed_bt_priv_t;

int aspeed_bt_read(bt_t *bt, uint8_t *buf, uint32_t buf_sz);
int aspeed_bt_write(bt_t *bt, uint8_t *buf, uint32_t buf_sz);
void aspeed_bt_init(bt_t *bt);

#endif
