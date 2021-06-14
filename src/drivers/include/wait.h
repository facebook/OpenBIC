
#ifndef __WAIT__
#define __WAIT__

#include <stdint.h>
int aspeed_wait_ms(uint32_t ms);
int aspeed_wait_us(uint32_t us);
void aspeed_wait_init_timer(uint8_t is_os_start);

#endif
