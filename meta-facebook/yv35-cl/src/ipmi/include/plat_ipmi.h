#ifndef PLAT_IPMI_H
#define PLAT_IPMI_H

#include "ipmi.h"
#include "plat_ipmb.h"

enum {
	CPNT_CPLD = 1,
	CPNT_BIC,
	CPNT_ME,
	CPNT_PVCCIN = 5,
	CPNT_PVCCFA_EHV_FIVRA,
	CPNT_PVCCD_HV,
	CPNT_PVCCINFAON,
	CPNT_PVCCFA_EHV
};

typedef struct CHASSIS_STATUS_STRUCT {
	struct CURRENT_POWER_STATE {
		uint8_t pwOn;
		uint8_t pwOverload;
		uint8_t interlock;
		uint8_t pwFault;
		uint8_t pwControlFault;
		uint8_t pwRestorePolicy;
		uint8_t reserve;
	} currPwState;

	struct LAST_POWER_EVENT {
		uint8_t acFail;
		uint8_t pwDownOverload;
		uint8_t pwDownInterlock;
		uint8_t pwDownFault;
		uint8_t pwOnIpmi;
		uint8_t reserve;
	} lastPwEvt;

	struct MISC {
		uint8_t intru;
		uint8_t fpcLockout;
		uint8_t driveFault;
		uint8_t fanFault;
		uint8_t idLedState;
		uint8_t idLedSupport;
		uint8_t reserve;
	} misc;

	struct CHASSIS_BUTTON_ENABLES {
		uint8_t powerOffButtonDisable;
		uint8_t resetButtonDisable;
		uint8_t diagnosticDisable;
		uint8_t standbyDisable;
		uint8_t powerOffButtonDisableAllowed;
		uint8_t resetButtonDisableAllowed;
		uint8_t diagnosticDisableAllowed;
		uint8_t standbyDisableAllowed;
	} chassisButtonEnables;
} CHASSIS_STATUS;

typedef struct addsel_msg_t {
	uint8_t sensor_type;
	uint8_t sensor_number;
	uint8_t evt_type;
	uint8_t evt_data1;
	uint8_t evt_data2;
	uint8_t evt_data3;
} addsel_msg_t;

bool add_sel_evt_record(addsel_msg_t *sel_msg);

#endif
