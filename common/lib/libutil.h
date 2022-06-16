#ifndef LIBUTIL_H
#define LIBUTIL_H

#include <stdint.h>
#include "ipmb.h"
#include "hal_i2c.h"

#define SAFE_FREE(p)                                                                               \
	if (p) {                                                                                   \
		free(p);                                                                           \
		p = NULL;                                                                          \
	}

#define SETBIT(x, y) (x | (1ULL << y))
#define GETBIT(x, y) ((x & (1ULL << y)) > y)
#define CLEARBIT(x, y) (x & (~(1ULL << y)))

ipmi_msg construct_ipmi_message(uint8_t seq_source, uint8_t netFn, uint8_t command,
				uint8_t source_inft, uint8_t target_inft, uint16_t data_len,
				uint8_t *data);

I2C_MSG construct_i2c_message(uint8_t bus_id, uint8_t address, uint8_t tx_len, uint8_t *data,
			      uint8_t rx_len);

#endif
