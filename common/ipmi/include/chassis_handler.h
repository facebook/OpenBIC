#ifndef CHASSIS_HANDLER_H
#define CHASSIS_HANDLER_H

#include "ipmi.h"

typedef struct CHASSIS_STATUS_STRUCT {
	struct CURRENT_POWER_STATE {
		uint8_t pwOn : 1;
		uint8_t pwOverload : 1;
		uint8_t interlock : 1;
		uint8_t pwFault : 1;
		uint8_t pwControlFault : 1;
		uint8_t pwRestorePolicy : 2;
		uint8_t reserve : 1;
	} currPwState;

	struct LAST_POWER_EVENT {
		uint8_t acFail : 1;
		uint8_t pwDownOverload : 1;
		uint8_t pwDownInterlock : 1;
		uint8_t pwDownFault : 1;
		uint8_t pwOnIpmi : 1;
		uint8_t reserve : 3;
	} lastPwEvt;

	struct MISC {
		uint8_t intru : 1;
		uint8_t fpcLockout : 1;
		uint8_t driveFault : 1;
		uint8_t fanFault : 1;
		uint8_t idLedState : 2;
		uint8_t idLedSupport : 1;
		uint8_t reserve : 1;
	} misc;

	struct CHASSIS_BUTTON_ENABLES {
		uint8_t powerOffButtonDisable : 1;
		uint8_t resetButtonDisable : 1;
		uint8_t diagnosticDisable : 1;
		uint8_t standbyDisable : 1;
		uint8_t powerOffButtonDisableAllowed : 1;
		uint8_t resetButtonDisableAllowed : 1;
		uint8_t diagnosticDisableAllowed : 1;
		uint8_t standbyDisableAllowed : 1;
	} chassisButtonEnables;
} CHASSIS_STATUS;

#ifdef CONFIG_ESPI
void CHASSIS_GET_CHASSIS_STATUS(ipmi_msg *msg);
#endif

void IPMI_CHASSIS_handler(ipmi_msg *msg);

#endif
