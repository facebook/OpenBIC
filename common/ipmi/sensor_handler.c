#include "sensor_handler.h"

#include "sensor.h"

__weak void SENSOR_GET_SENSOR_READING(ipmi_msg *msg)
{
	uint8_t status, snr_num;
	int reading;

	if (msg->data_len != 1) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	if (!enable_sensor_poll) {
		printf("Reading sensor cache while sensor polling disable\n");
		msg->completion_code = CC_CAN_NOT_RESPOND;
		return;
	}

	snr_num = msg->data[0];
	// Fix to get_from_cache. As need real time reading, use OEM command to get_from_sensor.
	status = get_sensor_reading(snr_num, &reading, get_from_cache);

	switch (status) {
	case SNR_READ_SUCCESS:
		msg->data[0] = reading & 0xff;
		// SDR sensor initialization bit6 enable scan, bit5 enable event
		// retunr data 1 bit 7 set to 0 to disable all event msg. bit 6 set to 0 disable sensor scan
		msg->data[1] =
			((full_sensor_table[SnrNum_SDR_map[snr_num]].sensor_init & 0x60) << 1);
		// fix to threshold deassert status, BMC will compare with UCR/UNR itself
		msg->data[2] = 0xc0;
		msg->data_len = 3;
		msg->completion_code = CC_SUCCESS;
		break;
	case SNR_READ_ACUR_SUCCESS:
		// In accurate read case, scale reading to one byte
		msg->data[0] = (reading >> 8) & 0xff;
		// SDR sensor initialization bit6 enable scan, bit5 enable event
		// retunr data 1 bit 7 set to 0 to disable all event msg. bit 6 set to 0 disable sensor scan
		msg->data[1] =
			((full_sensor_table[SnrNum_SDR_map[snr_num]].sensor_init & 0x60) << 1);
		// fix to threshold deassert status, BMC will compare with UCR/UNR itself
		msg->data[2] = 0xc0;
		msg->data_len = 3;
		msg->completion_code = CC_SUCCESS;
		break;
	case SNR_FAIL_TO_ACCESS:
		// transection error
		msg->completion_code = CC_NODE_BUSY;
		break;
	case SNR_NOT_ACCESSIBLE:
		// DC off
		msg->completion_code = CC_NOT_SUPP_IN_CURR_STATE;
		break;
	case SNR_NOT_FOUND:
		// request sensor number not found
		msg->completion_code = CC_INVALID_DATA_FIELD;
		break;
	case SNR_UNSPECIFIED_ERROR:
	default:
		// unknown error
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		break;
	}

	return;
}

void IPMI_SENSOR_handler(ipmi_msg *msg)
{
	switch (msg->cmd) {
	case CMD_SENSOR_GET_SENSOR_READING:
		SENSOR_GET_SENSOR_READING(msg);
		break;
	default:
		printf("invalid sensor msg netfn: %x, cmd: %x\n", msg->netfn, msg->cmd);
		msg->data_len = 0;
		break;
	}
	return;
}
