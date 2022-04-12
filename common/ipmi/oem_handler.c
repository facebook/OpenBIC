#include "oem_handler.h"

#include "sensor.h"
#include "plat_sensor_table.h"
#include "guid.h"
#include "plat_guid.h"
#include "plat_ipmi.h"
#ifdef ENABLE_FAN
#include "plat_fan.h"
#endif

#ifdef CONFIG_ESPI
__weak void OEM_NM_SENSOR_READ(ipmi_msg *msg)
{
	if (msg == NULL) {
		return;
	}

	uint8_t status, sensor_num;
	int reading;
	int val;

	// only input enable status
	if (msg->data_len < 3) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	// Follow INTEL NM SPEC, read platform pwr from HSC
	if (msg->data[0] == 0x00) {
		sensor_num = SENSOR_NUM_PWR_HSCIN;
		status = get_sensor_reading(sensor_num, &reading, GET_FROM_CACHE);

		val = (calculate_accurate_MBR(sensor_num, (int)reading) / 1000) & 0xffff;

		// scale down to one byte and times SDR to get original reading
		val = (val >> 8) * SDR_M(sensor_num);
	} else {
		msg->completion_code = CC_INVALID_DATA_FIELD;
		return;
	}

	switch (status) {
	case SENSOR_READ_SUCCESS:
		msg->data[1] = val & 0xFF;
		msg->data[2] = (val >> 8) & 0xFF;
		msg->data_len = 3;
		msg->completion_code = CC_SUCCESS;
		break;
	case SENSOR_FAIL_TO_ACCESS:
		// transection error
		msg->completion_code = CC_NODE_BUSY;
		break;
	case SENSOR_NOT_ACCESSIBLE:
		// DC off
		msg->completion_code = CC_NOT_SUPP_IN_CURR_STATE;
		break;
	case SENSOR_NOT_FOUND:
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

#ifdef ENABLE_FAN
__weak void OEM_SET_FAN_DUTY_MANUAL(ipmi_msg *msg)
{
	/*********************************
	Request
	data 0: fan pwm index
	data 1: duty
	Response
	data 0: completion code
	***********************************/
	if (msg == NULL) {
		printf("%s failed due to parameter *msg is NULL\n", __func__);
		return;
	}

	if (msg->data_len != 2) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t pwm_id = msg->data[0];
	uint8_t duty = msg->data[1];
	uint8_t current_fan_mode = FAN_AUTO_MODE, slot_index = 0;
	int ret = 0, i = 0;

	msg->data_len = 0;
	msg->completion_code = CC_SUCCESS;

	if (msg->InF_source == SLOT1_BIC) {
		slot_index = INDEX_SLOT1;
	} else if (msg->InF_source == SLOT3_BIC) {
		slot_index = INDEX_SLOT3;
	} else {
		msg->completion_code = CC_PARAM_OUT_OF_RANGE;
		return;
	}

	if ((duty > MAX_FAN_DUTY_VALUE) ||
	    ((pwm_id >= MAX_FAN_PWM_INDEX_COUNT) && (pwm_id != INDEX_ALL_PWM))) {
		msg->completion_code = CC_PARAM_OUT_OF_RANGE;
		return;
	}

	ret = pal_get_fan_ctrl_mode(&current_fan_mode);
	if (ret < 0) {
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}

	if (current_fan_mode != FAN_MANUAL_MODE) {
		printf("%s() is called when it's not at manual mode\n", __func__);
		return;
	}

	if (pwm_id == INDEX_ALL_PWM) {
		for (i = 0; i < MAX_FAN_PWM_INDEX_COUNT; i++) {
			ret = pal_set_fan_duty(i, duty, slot_index);
			if (ret < 0) {
				msg->completion_code = CC_UNSPECIFIED_ERROR;
				break;
			}
		}
	} else {
		ret = pal_set_fan_duty(pwm_id, duty, slot_index);
		if (ret < 0) {
			msg->completion_code = CC_UNSPECIFIED_ERROR;
		}
	}

	return;
}

__weak void OEM_GET_SET_FAN_CTRL_MODE(ipmi_msg *msg)
{
	/*********************************
	Request -
	data 0: Target
	  0x00 Set fan mode manual
	  0x01 Set fan mode auto
	  0x02 Get fan mode
	Response -
	data 0: Completion code
	if request data 0 == 0x02
	data 1: Current fan mode
	  0x00 manual fan mode
	  0x01 auto fan mode
	***********************************/
	if (msg == NULL) {
		printf("%s failed due to parameter *msg is NULL\n", __func__);
		return;
	}

	if (msg->data_len != 1) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t ctrl_cmd = msg->data[0];
	uint8_t ctrl_mode_get = 0;
	int ret = 0;

	msg->data_len = 0;
	msg->completion_code = CC_SUCCESS;

	if (ctrl_cmd == FAN_SET_MANUAL_MODE) {
		pal_set_fan_ctrl_mode(FAN_MANUAL_MODE);

	} else if (ctrl_cmd == FAN_SET_AUTO_MODE) {
		pal_set_fan_ctrl_mode(FAN_AUTO_MODE);

	} else if (ctrl_cmd == FAN_GET_MODE) {
		ret = pal_get_fan_ctrl_mode(&ctrl_mode_get);
		if (ret < 0) {
			msg->completion_code = CC_UNSPECIFIED_ERROR;

		} else {
			msg->data_len = 1;
			msg->data[0] = ctrl_mode_get;
		}

	} else {
		msg->completion_code = CC_PARAM_OUT_OF_RANGE;
	}

	return;
}
#endif

__weak void OEM_GET_MB_INDEX(ipmi_msg *msg)
{
	if (msg == NULL) {
		printf("%s failed due to parameter *msg is NULL\n", __func__);
		return;
	}

	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	if (msg->InF_source == SLOT1_BIC) {
		msg->data[0] = INDEX_SLOT1;
	} else if (msg->InF_source == SLOT3_BIC) {
		msg->data[0] = INDEX_SLOT3;
	} else {
		msg->data_len = 0;
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}

	msg->data_len = 1;
	msg->completion_code = CC_SUCCESS;
	return;
}

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
#ifdef ENABLE_FAN
	case CMD_OEM_SET_FAN_DUTY_MANUAL:
		OEM_SET_FAN_DUTY_MANUAL(msg);
		break;
	case CMD_OEM_GET_SET_FAN_CTRL_MODE:
		OEM_GET_SET_FAN_CTRL_MODE(msg);
		break;
#endif
	case CMD_OEM_GET_MB_INDEX:
		OEM_GET_MB_INDEX(msg);
		break;
	default:
		printf("invalid OEM msg netfn: %x, cmd: %x\n", msg->netfn, msg->cmd);
		msg->data_len = 0;
		msg->completion_code = CC_INVALID_CMD;
		break;
	}
	return;
}
