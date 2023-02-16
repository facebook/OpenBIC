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

#ifndef PM8702_H
#define PM8702_H
#include "mctp.h"
#include "plat_def.h"

#ifdef ENABLE_PM8702

/*CCI (pm8702 vendor CMD) */
#define pm8702_I2C_OFFSET_READ 0xc401

/*CCI (pm8702 vendor CMD) Request paypload length */
#define I2C_OFFSET_READ_REQ_PL_LEN 20 /*Size Bytes*/

/*CCI (pm8702 vendor CMD) Response paypload length */
#define DIMM_TEMP_READ_RESP_PL_LEN 2 /*Size Bytes*/

/*PM8702_I2C_OFFSET_READ parameters */
#define ADDR_SIZE_7_BIT 0x00
#define ADDR_SIZE_10_BIT 0x01

#define OFFSET_SIZE_8_BIT 0x01
#define OFFSET_SIZE_16_BIT 0x02

#define I2C_READ_TIMEOUT_MS 1000
#define DIMM_TEMP_REG_OFFSET 0x0005 /*Refer to JEDEC SPD*/

typedef struct __attribute__((__packed__)) {
	uint8_t addr_size;
	uint8_t rsvd_0;
	uint16_t address;
	uint8_t offset_size;
	uint8_t rsvd_1;
	uint16_t offset;
	uint32_t timeout_offset;
	uint32_t read_bytes;
	uint32_t timeout_ms;
} i2c_offset_read_req;

typedef enum _pm8702_access {
	chip_temp,
	dimm_temp,
} pm8702_access;

bool pm8702_get_dimm_temp(void *mctp_p, mctp_ext_params ext_params, uint16_t address,
			  int16_t *interger, int16_t *fraction);
#endif

#endif
