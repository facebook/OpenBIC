#ifndef PLAT_IPMI_H
#define PLAT_IPMI_H

#include <stdbool.h>
#include <stdint.h>

enum REQ_GET_CARD_TYPE {
	GET_1OU_CARD_TYPE = 0x0,
	GET_2OU_CARD_TYPE,
};

#endif
