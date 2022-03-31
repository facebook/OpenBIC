#include "chassis_handler.h"

#include "power_status.h"

#ifdef CONFIG_ESPI
__weak void CHASSIS_GET_CHASSIS_STATUS(ipmi_msg *msg)
{
	if (msg == NULL) {
		return;
	}

	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	CHASSIS_STATUS chassis_status = { 0 };

	chassis_status.currPwState.pwOn = get_DC_status();
	chassis_status.currPwState.pwFault = get_DC_status();
	chassis_status.currPwState.pwRestorePolicy = 0x2; // Always On

	memcpy(msg->data, &chassis_status, sizeof(CHASSIS_STATUS));
	msg->data_len = sizeof(CHASSIS_STATUS);
	msg->completion_code = CC_SUCCESS;
	return;
}
#endif

void IPMI_CHASSIS_handler(ipmi_msg *msg)
{
	if (msg == NULL) {
		return;
	}

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
