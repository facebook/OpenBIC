#include "chassis_handler.h"

#include "power_status.h"

#ifdef CONFIG_ESPI
__weak void CHASSIS_GET_CHASSIS_STATUS(ipmi_msg *msg)
{
	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	CHASSIS_STATUS chassis_status = { 0 };

	// CurrPwState
	chassis_status.currPwState.pwOn = get_DC_status();
	chassis_status.currPwState.pwFault = get_DC_status();
	chassis_status.currPwState.interlock = 0;
	chassis_status.currPwState.pwFault = 0;
	chassis_status.currPwState.pwControlFault = 0;
	chassis_status.currPwState.pwRestorePolicy = 0x2; // Always On
	chassis_status.currPwState.reserve = 0;

	// LastPwEvt
	chassis_status.lastPwEvt.acFail = 0;
	chassis_status.lastPwEvt.pwDownOverload = 0;
	chassis_status.lastPwEvt.pwDownInterlock = 0;
	chassis_status.lastPwEvt.pwDownFault = 0;
	chassis_status.lastPwEvt.pwOnIpmi = 0;
	chassis_status.lastPwEvt.reserve = 0;

	// Misc
	chassis_status.misc.intru = 0;
	chassis_status.misc.fpcLockout = 0;
	chassis_status.misc.driveFault = 0;
	chassis_status.misc.fanFault = 0;
	chassis_status.misc.idLedState = 0;
	chassis_status.misc.idLedSupport = 0;
	chassis_status.misc.reserve = 0;

	// ChassisButtonEnables
	chassis_status.chassisButtonEnables.powerOffButtonDisable = 0;
	chassis_status.chassisButtonEnables.resetButtonDisable = 0;
	chassis_status.chassisButtonEnables.diagnosticDisable = 0;
	chassis_status.chassisButtonEnables.standbyDisable = 0;
	chassis_status.chassisButtonEnables.powerOffButtonDisableAllowed = 0;
	chassis_status.chassisButtonEnables.resetButtonDisableAllowed = 0;
	chassis_status.chassisButtonEnables.diagnosticDisableAllowed = 0;
	chassis_status.chassisButtonEnables.standbyDisableAllowed = 0;

	memcpy(msg->data, &chassis_status, sizeof(CHASSIS_STATUS));
	msg->data_len = sizeof(CHASSIS_STATUS);
	msg->completion_code = CC_SUCCESS;
	return;
}
#endif

void IPMI_CHASSIS_handler(ipmi_msg *msg)
{
	switch (msg->cmd) {
#ifdef CONFIG_ESPI
	case CMD_CHASSIS_GET_CHASSIS_STATUS:
		CHASSIS_GET_CHASSIS_STATUS(msg);
		break;
#endif
	default:
		printf("invalid chassis msg netfn: %x, cmd: %x\n", msg->netfn, msg->cmd);
		msg->data_len = 0;
		msg->completion_code = CC_INVALID_CMD;
		break;
	}
	return;
}
