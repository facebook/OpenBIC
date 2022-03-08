#ifndef APP_HANDLER_H
#define APP_HANDLER_H

#include "ipmi.h"

#define GET_TEST_RESULT 0

typedef struct SELF_TEST_RESULT_STRUCT {
	uint8_t status;

	struct RESULT {
		uint8_t cannotAccessSelDev : 1;
		uint8_t cannotAccessSdrRepo : 1;
		uint8_t cannotAccessBmcFruDev : 1;
		uint8_t ipmbLinesDead : 1;
		uint8_t sdrRepoEmpty : 1;
		uint8_t internalCorrupt : 1;
		uint8_t updateFwCorrupt : 1;
		uint8_t opFwCorrupt : 1;
	} result;
} SELF_TEST_RESULT;

void APP_GET_DEVICE_ID(ipmi_msg *msg);
void APP_COLD_RESET(ipmi_msg *msg);
void APP_WARM_RESET(ipmi_msg *msg);
void APP_GET_SELFTEST_RESULTS(ipmi_msg *msg);
void APP_MASTER_WRITE_READ(ipmi_msg *msg);

#ifdef CONFIG_ESPI
void APP_GET_SYSTEM_GUID(ipmi_msg *msg);
#endif

void IPMI_APP_handler(ipmi_msg *msg);

#endif
