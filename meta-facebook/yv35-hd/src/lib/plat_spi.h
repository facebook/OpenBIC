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

#ifndef PLAT_SPI_H
#define PLAT_SPI_H

#define CPLD_ADDR 0x21
#define CPLD_SPI_OOB_CONTROL_REG 0x0B
#define CPLD_SPI_OOB_FROM_CPU 0x02
#define CPLD_SPI_OOB_FROM_BIC 0x0B
#define FLASH_COPY_STACK_SIZE 1024

enum flash_copy_status {
	COPY_STATUS_IDEL = 0,
	COPY_STATUS_BUSY,
};

enum flash_copy_completion_code {
	COPY_FLASH_SUCCESS = 0,
	COPY_FLASH_UNSUPPORTED_TYPE = 0x80,
	COPY_FLASH_INIT_FAIL,
	COPY_FLASH_OUT_OF_RANGE,
	COPY_FLASH_MALLOC_FAIL,
	COPY_FLASH_WRITE_ERROR,
	COPY_FLASH_READ_ERROR,
};

typedef struct _flash_copy_info_ {
	uint8_t copy_type;
	uint32_t src_offset;
	uint32_t dest_offset;
	uint32_t total_length;
	uint8_t status;
	uint8_t completion_code;
	uint32_t current_len;
} FLASH_COPY_INFO;

uint8_t start_flash_copy(uint8_t copy_type, uint32_t src_offset, uint32_t dest_offset,
		      uint32_t length);
void get_flash_copy_info(FLASH_COPY_INFO *copy_info);

#endif
