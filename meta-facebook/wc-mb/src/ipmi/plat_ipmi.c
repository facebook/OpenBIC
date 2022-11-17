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

#include "plat_ipmi.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <logging/log.h>

#include "libutil.h"
#include "ipmi.h"
#include "plat_ipmb.h"
#include "fru.h"
#include "eeprom.h"
#include "plat_fru.h"

LOG_MODULE_REGISTER(plat_ipmi);

int pal_record_bios_fw_version(uint8_t *buf, uint8_t size)
{
	CHECK_NULL_ARG_WITH_RETURN(buf, -1);

	int ret = -1;
	EEPROM_ENTRY set_bios_ver = { 0 };
	EEPROM_ENTRY get_bios_ver = { 0 };

	ret = get_bios_version(&get_bios_ver);
	if (ret == -1) {
		LOG_ERR("Get version fail");
		return -1;
	}

	set_bios_ver.data_len = size - 3; // skip netfn, cmd and command code
	memcpy(&set_bios_ver.data[0], &buf[3], set_bios_ver.data_len);

	// Check the written BIOS version is the same with the stored
	ret = memcmp(&get_bios_ver.data[0], &set_bios_ver.data[0],
		     BIOS_FW_VERSION_MAX_SIZE * sizeof(uint8_t));
	if (ret == 0) {
		LOG_DBG("The Written bios version is the same with the stored bios version in EEPROM");
	} else {
		LOG_DBG("Set bios version");

		ret = set_bios_version(&set_bios_ver);
		if (ret == -1) {
			LOG_ERR("Set version fail");
			return -1;
		}
	}

	return 0;
}

void OEM_1S_GET_BIOS_VERSION(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	int ret = -1;
	EEPROM_ENTRY get_bios_ver = { 0 };

	ret = get_bios_version(&get_bios_ver);
	if (ret == -1) {
		LOG_ERR("Get version fail");
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}

	memcpy(&msg->data[0], &get_bios_ver.data[0], get_bios_ver.data_len);
	msg->data_len = get_bios_ver.data_len;
	msg->completion_code = CC_SUCCESS;
	return;
}
