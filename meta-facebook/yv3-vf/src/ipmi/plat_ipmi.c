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

#include <stdio.h>
#include <stdlib.h>

#include "ipmi.h"
#include "libutil.h"

#include "plat_class.h"
#include "plat_m2.h"
#include "plat_led.h"
#include "plat_power_seq.h"

#include <logging/log.h>

LOG_MODULE_REGISTER(plat_ipmi);

// for GET_SET_M2
#define DRIVE_PWR_OFF 0 /* normal device power off/on */
#define DRIVE_PWR_ON 1
#define DRIVE_GET_STATE 3
#define DRIVE_PWR_OFF_BY_PWRDIS 4 /* power off/on device by PWRDIS pin */
#define DRIVE_PWR_ON_BY_PWRDIS 5
#define DRIVE_PWR_OFF_ALL 6 /* power off/on device by PWRDIS/12V/3V3 */
#define DRIVE_PWR_ON_ALL 7
#define DRIVE_PWR_OFF_BY_12V_3V3 8 /* power off/on device by 12V/3V3 */
#define DRIVE_PWR_ON_BY_12V_3V3 9

static uint32_t iana_list[] = {
	IANA_ID,
	IANA_ID2,
};

// IANA 0 is reserved so it should never be a valid IANA
uint32_t get_iana(uint8_t *iana_buf)
{
	CHECK_NULL_ARG_WITH_RETURN(iana_buf, 0);

	uint32_t recieved_iana = iana_buf[2];
	recieved_iana = (recieved_iana << 8) | (uint32_t)iana_buf[1];
	recieved_iana = (recieved_iana << 8) | (uint32_t)iana_buf[0];

	for (uint8_t iana_idx = 0; iana_idx < ARRAY_SIZE(iana_list); iana_idx++) {
		if (recieved_iana == iana_list[iana_idx])
			return iana_list[iana_idx];
	}

	return 0;
}

void OEM_1S_GET_BOARD_ID(ipmi_msg *msg)
{
	if (msg == NULL) {
		LOG_ERR("Failed due to parameter *msg is NULL");
		return;
	}

	if (msg->data_len != 0) {
		msg->data_len = 0;
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	msg->data_len = 1;
	msg->data[0] = get_board_id();
	msg->completion_code = CC_SUCCESS;
	return;
}

void OEM_1S_SET_SSD_LED(ipmi_msg *msg)
{
	if (msg == NULL) {
		LOG_ERR("Failed due to parameter *msg is NULL");
		return;
	}

	if (msg->data_len != 2) {
		msg->data_len = 0;
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t dev = msg->data[0];

	if (!m2_prsnt(dev)) {
		msg->data_len = 0;
		msg->completion_code = 0x80; //ssd not present response complete code
		return;
	}

	if (SSDLEDCtrl(dev, msg->data[1])) {
		msg->data_len = 0;
		msg->completion_code = CC_INVALID_DATA_FIELD;
		return;
	}

	msg->data_len = 0;
	msg->completion_code = CC_SUCCESS;
}

void OEM_1S_GET_SSD_STATUS(ipmi_msg *msg)
{
	if (msg == NULL) {
		LOG_ERR("Failed due to parameter *msg is NULL");
		return;
	}

	if (msg->data_len != 1) {
		msg->data_len = 0;
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t dev = msg->data[0];
	if (!m2_prsnt(dev)) {
		msg->data_len = 0;
		msg->completion_code = 0x80; //ssd not present response complete code
		return;
	}

	msg->data_len = 1;
	msg->data[0] = GetAmberLEDStat(dev);
	msg->completion_code = CC_SUCCESS;
}

void OEM_1S_GET_SET_M2(ipmi_msg *msg)
{
	if (msg == NULL) {
		LOG_ERR("Failed due to parameter *msg is NULL");
		return;
	}

	if (msg->data_len != 2) {
		msg->data_len = 0;
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	msg->completion_code = CC_INVALID_DATA_FIELD;
	msg->data_len = 0;
	uint8_t val, dev, is_on;
	uint8_t func = msg->data[1];
	uint8_t change_dev = msg->data[0] - 1; // Shift for 0 based
	change_dev = exchange_m2_idx(change_dev); // exchange m2 idx

	switch (func) {
	case DRIVE_PWR_OFF:
	case DRIVE_PWR_ON:
		dev = change_dev;
		val = DEV_CHK_DISABLE | DEV_PWR_CTRL | DEV_PCIE_RST |
		      ((func == 1) ? DEV_PWR_ON : 0);

		if (dev >= M2_IDX_E_MAX)
			return;

		device_all_power_set(dev, val);

		msg->completion_code = CC_SUCCESS;
		break;

	case DRIVE_GET_STATE:
		dev = msg->data[0];

		if (!dev) {
			/* get all m2 device status */
			uint16_t *tmp = (uint16_t *)(msg->data + 1);
			uint8_t i;

			*tmp = 0;
			for (i = 0; i < M2_IDX_E_MAX; i++)
				*tmp |= (m2_pwrgd(i) && m2_prsnt(i)) << i;

			msg->data_len += 2;
			msg->completion_code = CC_SUCCESS;
		} else {
			dev = change_dev;
			if (dev >= M2_IDX_E_MAX)
				return;

			/* get specific m2 device status */
			//buf[1] = m2_pwrgd((M2_IDX_E)dev) && m2_prsnt((M2_IDX_E)dev);    //remove for now, will add back
			msg->data[0] = m2_pwrgd(dev);
			msg->data_len += 1;
			msg->completion_code = CC_SUCCESS;
		}
		break;

	case DRIVE_PWR_OFF_BY_PWRDIS:
	case DRIVE_PWR_ON_BY_PWRDIS:
	case DRIVE_PWR_OFF_ALL:
	case DRIVE_PWR_ON_ALL:
		dev = change_dev;
		is_on = (func == DRIVE_PWR_ON_BY_PWRDIS || func == DRIVE_PWR_ON_ALL);

		if (dev >= M2_IDX_E_MAX)
			return;

		val = DEV_PRSNT_SET | DEV_PWRDIS_EN | DEV_FORCE_3V3 | (is_on ? DEV_PWR_ON : 0);
		if (func == DRIVE_PWR_OFF_ALL || func == DRIVE_PWR_ON_ALL)
			val |= DEV_PWR_CTRL;

		device_all_power_set(dev, val);

		msg->completion_code = CC_SUCCESS;
		break;

	case DRIVE_PWR_OFF_BY_12V_3V3:
	case DRIVE_PWR_ON_BY_12V_3V3:
		dev = change_dev;
		is_on = (func == DRIVE_PWR_ON_BY_12V_3V3);

		if (dev >= M2_IDX_E_MAX)
			return;

		val = DEV_PWR_CTRL | DEV_PRSNT_SET | DEV_FORCE_3V3 | DEV_PCIE_RST |
		      (is_on ? DEV_PWR_ON : 0);
		device_all_power_set(dev, val);

		msg->completion_code = CC_SUCCESS;
		break;
	}
}
