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
#include <stdlib.h>
#include <stdio.h>
#include <drivers/flash.h>
#include <logging/log.h>
#include <device.h>
#include "util_spi.h"
#include "plat_spi.h"
// CXL flash JEDEC ID
#define IS25WP256D_ID 0x9D7019

LOG_MODULE_REGISTER(plat_spi);

uint8_t pal_init_cxl_flash_device(uint8_t cxl_comp_id, const struct device **flash_dev)
{
	if (flash_dev == NULL) {
		LOG_ERR("Invalid parameter");
		return 1;
	}
	int flash_position = pal_get_cxl_flash_position_by_id(cxl_comp_id);
	if (flash_position == 0xFF) {
		LOG_ERR("Invalid flash position for CXL component ID %d", cxl_comp_id);
		return 1;
	}
	*flash_dev = device_get_binding(get_flash_device_string_by_index(flash_position));
	if (!(*flash_dev)) {
		LOG_ERR("device_get_binding() failed");
		return 1;
	}

	int ret = ckeck_flash_device_isinit(*flash_dev, flash_position);
	if (ret != 0) {
		LOG_ERR("Failed to re-init flash");
		return ret;
	}
	//IS25WP256D need to set 4byte address mode to read  flash
	uint8_t jedec_id[3] = { 0 };
	flash_read_jedec_id(*flash_dev, jedec_id);
	if ((jedec_id[0] << 16 | jedec_id[1] << 8 | jedec_id[2]) == IS25WP256D_ID) {
		spi_nor_config_4byte_mode(*flash_dev, true);
	}

	return 0;
}

int pal_get_cxl_flash_position_by_id(uint8_t cxl_comp_id)
{
	int flash_position = 0xFF;
	switch (cxl_comp_id) {
	case WF_COMPNT_CXL1:
		flash_position = DEVSPI_SPI1_CS0;
		break;
	case WF_COMPNT_CXL2:
		flash_position = DEVSPI_SPI2_CS0;
		break;
	default:
		LOG_ERR("Unknown CXL component ID %d", cxl_comp_id);
		return 0xFF;
	}
	return flash_position;
}

int pal_dump_cxl_flash_data(uint8_t *read_buf, uint32_t offset, size_t length,
			    const struct device *flash_dev, uint8_t cxl_comp_id)
{
	if (read_buf == NULL || flash_dev == NULL) {
		LOG_ERR("Invalid parameter");
		return 1;
	}
	uint8_t ret = 0;

	ret = flash_read(flash_dev, offset, read_buf, length);
	if (ret != 0) {
		LOG_ERR("Failed to read CXL flash at offset 0x%x (ret = %d)", offset, ret);
		return 1;
	}
	return 0;
}