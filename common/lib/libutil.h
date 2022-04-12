#ifndef LIBUTIL_H
#define LIBUTIL_H

#include <stdint.h>
#include "ipmb.h"

#define SAFE_FREE(p)                                                                               \
	if (p) {                                                                                   \
		free(p);                                                                           \
		p = NULL;                                                                          \
	}

ipmi_msg construct_ipmi_message(uint8_t seq_source, uint8_t netFn, uint8_t command,
				uint8_t source_inft, uint8_t target_inft, uint16_t data_len,
				uint8_t *data);

#endif
