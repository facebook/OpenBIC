#include "sensor_handler.h"

#include "sensor.h"

void IPMI_SENSOR_handler(ipmi_msg *msg)
{
	if (msg == NULL) {
		return;
	}

	switch (msg->cmd) {
	case CMD_SENSOR_GET_SENSOR_READING:
		// For Meta's projects, the sensor reading command should be used the OEM_1S_ACCURACY_SENSOR_READING.
		msg->data_len = 0;
		msg->completion_code = CC_NOT_SUPP_IN_CURR_STATE;
		break;
	default:
		printf("invalid sensor msg netfn: %x, cmd: %x\n", msg->netfn, msg->cmd);
		msg->data_len = 0;
		msg->completion_code = CC_INVALID_CMD;
		break;
	}
	return;
}
