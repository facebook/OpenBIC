#include "oem_handler.h"

#include "sensor.h"
#include "plat_sensor.h"
#include "guid.h"
#include "plat_guid.h"

#ifdef CONFIG_ESPI
__weak void OEM_NM_SENSOR_READ(ipmi_msg *msg)
{
	if (msg == NULL) {
		return;
	}

	uint8_t status, snr_num;
	int reading;

	// only input enable status
	if (msg->data_len < 3) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	// Follow INTEL NM SPEC, read platform pwr from HSC
	if (msg->data[0] == 0x00) {
		snr_num = SENSOR_NUM_PWR_HSCIN;
		status = get_sensor_reading(snr_num, &reading, get_from_cache);
		// scale down to one byte and times SDR to get original reading
		reading = (reading >> 8) * SDR_M(snr_num);
	} else {
		msg->completion_code = CC_INVALID_DATA_FIELD;
		return;
	}

	switch (status) {
	case SNR_READ_ACUR_SUCCESS:
		msg->data[1] = reading & 0xFF;
		msg->data[2] = (reading >> 8) & 0xFF;
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
	default:
		// unknown error
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		break;
	}
	return;
}

__weak void OEM_SET_SYSTEM_GUID(ipmi_msg *msg)
{
	if (msg == NULL) {
		return;
	}

	if (msg->data_len != 16) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t status;
	EEPROM_ENTRY guid_entry;

	guid_entry.offset = 0;
	guid_entry.data_len = msg->data_len;
	guid_entry.config.dev_id = MB_SYS_GUID_ID;
	memcpy(&guid_entry.data[0], &msg->data, guid_entry.data_len);
	status = GUID_write(&guid_entry);

	switch (status) {
	case GUID_WRITE_SUCCESS:
		msg->completion_code = CC_SUCCESS;
		break;
	case GUID_INVALID_ID:
		msg->completion_code = CC_INVALID_PARAM;
		break;
	case GUID_OUT_OF_RANGE:
		msg->completion_code = CC_PARAM_OUT_OF_RANGE;
		break;
	case GUID_FAIL_TO_ACCESS:
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		break;
	default:
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		break;
	}

	msg->data_len = 0;
	return;
}
#endif

void IPMI_OEM_handler(ipmi_msg *msg)
{
	if (msg == NULL) {
		return;
	}

	switch (msg->cmd) {
#ifdef CONFIG_ESPI
	case CMD_OEM_NM_SENSOR_READ:
		OEM_NM_SENSOR_READ(msg);
		break;
	case CMD_OEM_SET_SYSTEM_GUID:
		OEM_SET_SYSTEM_GUID(msg);
		break;
#endif
	default:
		printf("invalid OEM msg netfn: %x, cmd: %x\n", msg->netfn, msg->cmd);
		msg->data_len = 0;
		msg->completion_code = CC_INVALID_CMD;
		break;
	}
	return;
}
