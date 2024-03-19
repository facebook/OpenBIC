/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "oem_handler.h"

#include "sensor.h"
#include "plat_sensor_table.h"
#include "guid.h"
#include <logging/log.h>
#include "libutil.h"
#ifdef ENABLE_FAN
#include "plat_fan.h"
#endif

LOG_MODULE_DECLARE(ipmi);

__weak uint8_t get_hsc_pwr_reading(int *reading)
{
	LOG_WRN("HSC Power Reading Not Supported");
	return SENSOR_NOT_FOUND;
}

#ifdef CONFIG_ESPI
__weak void OEM_NM_SENSOR_READ(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	uint8_t status;
	int reading;

	// only input enable status
	if (msg->data_len < 3) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	// Follow INTEL NM SPEC, read platform pwr from HSC
	if (msg->data[0] == 0x00) {
		status = get_hsc_pwr_reading(&reading);
	} else {
		msg->completion_code = CC_INVALID_DATA_FIELD;
		return;
	}

	sensor_val *sval = (sensor_val *)(&reading);
	switch (status) {
	case SENSOR_READ_SUCCESS:
	case SENSOR_READ_ACUR_SUCCESS:
	case SENSOR_READ_4BYTE_ACUR_SUCCESS:
		msg->data[1] = sval->integer & 0xFF;
		msg->data[2] = (sval->integer >> 8) & 0xFF;
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
#endif

__weak void OEM_SET_SYSTEM_GUID(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 16) {
		msg->completion_code = CC_INVALID_LENGTH;
		LOG_ERR("Message Data invalid length");
		return;
	}

	uint8_t status = set_system_guid(&msg->data_len, &msg->data[0]);

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
	CHECK_NULL_ARG(msg);

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
		LOG_WRN("Fan must be in Manual Mode to set fan.");
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
	CHECK_NULL_ARG(msg);

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
	CHECK_NULL_ARG(msg);

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

__weak void OEM_CABLE_DETECTION(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	msg->data_len = 0;
	msg->completion_code = CC_NOT_SUPP_IN_CURR_STATE;
	LOG_WRN("Cable Detection IPMI command not supported");
	return;
}

__weak void OEM_GET_CHASSIS_POSITION(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	msg->data_len = 0;
	msg->completion_code = CC_NOT_SUPP_IN_CURR_STATE;
	LOG_WRN("OEM_GET_CHASSIS_POSITION not supported");
	return;
}

void IPMI_OEM_handler(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	switch (msg->cmd) {
	case CMD_OEM_CABLE_DETECTION:
		LOG_DBG("Received Cable Detect command");
		OEM_CABLE_DETECTION(msg);
		break;
#ifdef CONFIG_ESPI
	case CMD_OEM_NM_SENSOR_READ:
		LOG_DBG("Received NM Sensor Read command");
		OEM_NM_SENSOR_READ(msg);
		break;
#endif
	case CMD_OEM_SET_SYSTEM_GUID:
		LOG_DBG("Received Set System GUID command");
		OEM_SET_SYSTEM_GUID(msg);
		break;
#ifdef ENABLE_FAN
	case CMD_OEM_SET_FAN_DUTY_MANUAL:
		LOG_DBG("Received Set Fan Duty (manual) command");
		OEM_SET_FAN_DUTY_MANUAL(msg);
		break;
	case CMD_OEM_GET_SET_FAN_CTRL_MODE:
		LOG_DBG("Received Set Fan Control Mode command");
		OEM_GET_SET_FAN_CTRL_MODE(msg);
		break;
#endif
	case CMD_OEM_GET_MB_INDEX:
		LOG_DBG("Received Get MB Index command");
		OEM_GET_MB_INDEX(msg);
		break;
	case CMD_OEM_GET_CHASSIS_POSITION:
		LOG_DBG("Received Get Chassis Position command");
		OEM_GET_CHASSIS_POSITION(msg);
		break;
	default:
		LOG_ERR("invalid OEM msg netfn: %x, cmd: %x", msg->netfn, msg->cmd);
		msg->data_len = 0;
		msg->completion_code = CC_INVALID_CMD;
		break;
	}
	return;
}
