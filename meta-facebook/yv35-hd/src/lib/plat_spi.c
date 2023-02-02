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
#include <logging/log.h>
#include <drivers/flash.h>
#include "plat_i2c.h"
#include "hal_i2c.h"
#include "util_spi.h"
#include "hal_gpio.h"
#include "plat_spi.h"
#include "libutil.h"

LOG_MODULE_REGISTER(plat_spi);

K_THREAD_STACK_DEFINE(flash_copy_thread, FLASH_COPY_STACK_SIZE);
k_tid_t flash_copy_tid;
struct k_thread flash_copy_thread_handler;
static FLASH_COPY_INFO flash_copy_info;

int pal_get_bios_flash_position()
{
	return DEVSPI_SPI1_CS0;
}

int pal_get_prot_flash_position()
{
	return DEVSPI_SPI2_CS0;
}

bool pal_switch_bios_spi_mux(int gpio_status)
{
	uint8_t retry = 5;
	I2C_MSG msg;

	msg.bus = I2C_BUS1;
	msg.target_addr = CPLD_ADDR;
	msg.tx_len = 2;
	msg.data[0] = CPLD_SPI_OOB_CONTROL_REG;
	if (gpio_status == GPIO_HIGH) {
		msg.data[1] = CPLD_SPI_OOB_FROM_BIC;
	} else {
		msg.data[1] = CPLD_SPI_OOB_FROM_CPU;
	}

	if (i2c_master_write(&msg, retry)) {
		return false;
	}
	return true;
}

void do_flash_copy()
{
	const struct device *src_flash_dev, *dest_flash_dev;
	uint8_t *data = NULL;

	if (flash_copy_info.copy_type == 0) {
		src_flash_dev = device_get_binding("spi1_cs0");
		dest_flash_dev = device_get_binding("spi2_cs0");
	} else if (flash_copy_info.copy_type == 1) {
		src_flash_dev = device_get_binding("spi2_cs0");
		dest_flash_dev = device_get_binding("spi1_cs0");
	} else {
		flash_copy_info.completion_code = COPY_FLASH_UNSUPPORTED_TYPE;
		goto exit;
	}

	if (spi_nor_re_init(src_flash_dev)) {
		flash_copy_info.completion_code = COPY_FLASH_INIT_FAIL;
		goto exit;
	}
	if (spi_nor_re_init(dest_flash_dev)) {
		flash_copy_info.completion_code = COPY_FLASH_INIT_FAIL;
		goto exit;
	}

	uint32_t flash_sz = flash_get_flash_size(src_flash_dev);
	if (flash_sz < (flash_copy_info.src_offset + flash_copy_info.total_length)) {
		flash_copy_info.completion_code = COPY_FLASH_OUT_OF_RANGE;
		goto exit;
	}
	flash_sz = flash_get_flash_size(dest_flash_dev);
	if (flash_sz < (flash_copy_info.dest_offset + flash_copy_info.total_length)) {
		flash_copy_info.completion_code = COPY_FLASH_OUT_OF_RANGE;
		goto exit;
	}

	data = (uint8_t *)malloc(SECTOR_SZ_4K * sizeof(uint8_t));
	if (data == NULL) {
		LOG_ERR("Failed to malloc data buffer.");
		flash_copy_info.completion_code = COPY_FLASH_MALLOC_FAIL;
		goto exit;
	}

	uint32_t read_offset, write_offset;
	for (flash_copy_info.current_len = 0;
	     flash_copy_info.current_len < flash_copy_info.total_length;
	     flash_copy_info.current_len += SECTOR_SZ_4K) {
		read_offset = flash_copy_info.src_offset + flash_copy_info.current_len;
		if (flash_read(src_flash_dev, read_offset, data, SECTOR_SZ_4K)) {
			flash_copy_info.completion_code = COPY_FLASH_READ_ERROR;
			goto exit;
		}
		write_offset = flash_copy_info.dest_offset + flash_copy_info.current_len;
		if (do_update(dest_flash_dev, write_offset, data, SECTOR_SZ_4K)) {
			flash_copy_info.completion_code = COPY_FLASH_WRITE_ERROR;
			goto exit;
		}
		k_yield();
	}
	flash_copy_info.completion_code = COPY_FLASH_SUCCESS;
exit:
	flash_copy_info.status = COPY_STATUS_IDEL;
	SAFE_FREE(data);
	return;
}

uint8_t start_flash_copy(uint8_t copy_type, uint32_t src_offset, uint32_t dest_offset, uint32_t length)
{
	if (flash_copy_tid != NULL && strcmp(k_thread_state_str(flash_copy_tid), "dead") != 0) {
		LOG_ERR("Flash copy thread running.");
		return -1;
	}
	memset(&flash_copy_info, 0, sizeof(FLASH_COPY_INFO));

	flash_copy_info.copy_type = copy_type;
	flash_copy_info.src_offset = src_offset;
	flash_copy_info.dest_offset = dest_offset;
	flash_copy_info.total_length = length;
	flash_copy_info.status = COPY_STATUS_BUSY;

	flash_copy_tid =
		k_thread_create(&flash_copy_thread_handler, flash_copy_thread,
				K_THREAD_STACK_SIZEOF(flash_copy_thread), do_flash_copy, NULL, NULL,
				NULL, CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&flash_copy_thread_handler, "flash_copy_thread");
	return 0;
}

void get_flash_copy_info(FLASH_COPY_INFO *copy_info)
{
	CHECK_NULL_ARG(copy_info);
	memcpy(copy_info, &flash_copy_info, sizeof(FLASH_COPY_INFO));
	return;
}
