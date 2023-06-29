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
#include <logging/log.h>
#include "sensor.h"
#include "pmbus.h"
#include "libutil.h"
#include "hal_i2c.h"
#include "plat_dev.h"
#include "plat_hook.h"
#include "plat_class.h"
#include "common_i2c_mux.h"
#include "plat_sensor_table.h"
#include "nvme.h"
#include "plat_ipmi.h"

LOG_MODULE_REGISTER(plat_dev);

freya_info accl_freya_info[] = {
	[0] = { .is_cache_freya1_info = false, .is_cache_freya2_info = false },
	[1] = { .is_cache_freya1_info = false, .is_cache_freya2_info = false },
	[2] = { .is_cache_freya1_info = false, .is_cache_freya2_info = false },
	[3] = { .is_cache_freya1_info = false, .is_cache_freya2_info = false },
	[4] = { .is_cache_freya1_info = false, .is_cache_freya2_info = false },
	[5] = { .is_cache_freya1_info = false, .is_cache_freya2_info = false },
	[6] = { .is_cache_freya1_info = false, .is_cache_freya2_info = false },
	[7] = { .is_cache_freya1_info = false, .is_cache_freya2_info = false },
	[8] = { .is_cache_freya1_info = false, .is_cache_freya2_info = false },
	[9] = { .is_cache_freya1_info = false, .is_cache_freya2_info = false },
	[10] = { .is_cache_freya1_info = false, .is_cache_freya2_info = false },
	[11] = { .is_cache_freya1_info = false, .is_cache_freya2_info = false },
};

void clear_freya_cache_flag()
{
	uint8_t index = 0;

	for (index = 0; index < ARRAY_SIZE(accl_freya_info); ++index) {
		accl_freya_info[index].is_cache_freya1_info = false;
		accl_freya_info[index].is_cache_freya2_info = false;
		memset(&accl_freya_info[index].freya1_fw_info, 0, FREYA_FW_VERSION_LENGTH);
		memset(&accl_freya_info[index].freya2_fw_info, 0, FREYA_FW_VERSION_LENGTH);
	}
}

int get_freya_fw_info(uint8_t bus, uint8_t addr, freya_fw_info *fw_info)
{
	CHECK_NULL_ARG_WITH_RETURN(fw_info, -1);

	int ret = 0;
	uint8_t read_buf[FREYA_MODULE_IDENTIFIER_BLOCK_LENGTH] = { 0 };

	ret = read_nvme_info(bus, addr, FREYA_STATUS_BLOCK_OFFSET, FREYA_STATUS_BLOCK_LENGTH,
			     read_buf);
	if (ret != 0) {
		LOG_ERR("Read freya ready status fail, bus: 0x%x, addr: 0x%x", bus, addr);

		// Set freya not ready value to avoid returning freya ready value
		fw_info->is_freya_ready = FREYA_NOT_READY;
		return -1;
	}

	if ((read_buf[FREYA_READY_STATUS_OFFSET] & FREYA_READY_STATUS_BIT) != 0) {
		fw_info->is_freya_ready = FREYA_NOT_READY;
		return FREYA_NOT_READY_RET_CODE;
	}

	fw_info->is_freya_ready = FREYA_READY;

	memset(read_buf, 0, sizeof(read_buf));
	ret = read_nvme_info(bus, addr, FREYA_MODULE_IDENTIFIER_BLOCK_OFFSET,
			     FREYA_MODULE_IDENTIFIER_BLOCK_LENGTH, read_buf);
	if (ret != 0) {
		LOG_ERR("Read freya module identifier fail, bus: 0x%x, addr: 0x%x", bus, addr);
		return -1;
	}

	fw_info->is_module_identifier_support =
		((read_buf[FREYA_MODULE_IDENTIFIER_OFFSET] != IS_FREYA_MODULE_IDENTIFIER_SUPPORT) ?
			 FREYA_NOT_SUPPORT_MODULE_IDENTIFIER :
			       FREYA_SUPPORT_MODULE_IDENTIFIER);
	if (fw_info->is_module_identifier_support == FREYA_NOT_SUPPORT_MODULE_IDENTIFIER) {
		return FREYA_NOT_SUPPORT_MODULE_IDENTIFIER_RET_CODE;
	}

	fw_info->form_factor_info = read_buf[FREYA_FFI_OFFSET];

	memset(read_buf, 0, sizeof(read_buf));
	ret = read_nvme_info(bus, addr, FREYA_FIRMWARE_VERSION_BLOCK_OFFSET,
			     FREYA_FIRMWARE_VERSION_BLOCK_LENGTH, read_buf);
	if (ret != 0) {
		LOG_ERR("Read freya firmware version fail, bus: 0x%x, addr: 0x%x", bus, addr);
		return -1;
	}

	memcpy(&fw_info->major_version, &read_buf[FREYA_FIRMWARE_VERSION_OFFSET],
	       FREYA_FIRMWARE_VERSION_LENGTH);
	return 0;
}
