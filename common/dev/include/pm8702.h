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
#define PM8702_HBO_STATUS 0xCD00
#define PM8702_HBO_TRANSFER_FW 0xCD01
#define PM8702_HBO_ACTIVATE_FW 0xCD02
#define PM8702_READ_DIMM_TEMP 0xC531

/*CCI (pm8702 vendor CMD) Request paypload length */
#define I2C_OFFSET_READ_REQ_PL_LEN 20 /*Size Bytes*/
#define HBO_STATUS_REQ_PL_LEN 0
#define HBO_TRANSFER_FW_REQ_PL_LEN 256
#define HBO_ACTIVATE_FW_REQ_PL_LEN 2
#define READ_DIMM_TEMP_REQ_PL_LEN 0

/*CCI (pm8702 vendor CMD) Response paypload length */
#define DIMM_TEMP_READ_RESP_PL_LEN 2 /*Size Bytes*/
#define DIMM_TEMP_RESP_PL_LEN 84

/*PM8702_I2C_OFFSET_READ parameters */
#define ADDR_SIZE_7_BIT 0x00
#define ADDR_SIZE_10_BIT 0x01

#define OFFSET_SIZE_8_BIT 0x01
#define OFFSET_SIZE_16_BIT 0x02

#define I2C_READ_TIMEOUT_MS 1000
#define DIMM_TEMP_REG_OFFSET 0x0005 /*Refer to JEDEC SPD*/

/*PM8702 0xC531 return code */
#define DIMM_ERROR_VALUE 0x80

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

typedef struct _pm8702_hbo_status_resp {
	uint16_t cmd_opcode;
	uint8_t percent_complete : 7;
	uint8_t bo_run : 1;
	uint8_t reserved;
	uint16_t return_code;
	uint16_t vendor_status;
} pm8702_hbo_status_resp;

typedef struct _pm8702_command_info {
	uint16_t cmd_opcode;
	int payload_len;
	int response_len;
} pm8702_command_info;

typedef struct __attribute__((__packed__)) {
	uint8_t compnt_type;
	uint8_t dimm_id;
	uint8_t temp_sensor_id;
	uint8_t rsvd_0;
	uint32_t rsvd_1;
	uint32_t rsvd_2;
	uint32_t rsvd_3;
	uint8_t temp_int;
	uint8_t temp_dec;
	uint16_t rsvd_4;
} dimm_slot_info; //0xc531

typedef struct __attribute__((__packed__)) {
	uint8_t dimm_num;
	uint8_t flag;
	uint16_t rsvd;
} dimm_info_header; //0xc531

typedef enum _pm8702_access {
	chip_temp,
	dimm_temp,
	dimm_temp_from_pioneer,
} pm8702_access;

bool pm8702_get_dimm_temp(void *mctp_p, mctp_ext_params ext_params, uint16_t address,
			  int16_t *interger, int16_t *fraction);
bool pm8702_cmd_handler(void *mctp_inst, mctp_ext_params ext_params, uint16_t opcode,
			uint8_t *data_buf, uint8_t data_len, uint8_t *response,
			uint8_t *response_len);
bool pm8702_read_dimm_temp_from_pioneer(void *mctp_p, mctp_ext_params ext_params, int dimm_id,
					int16_t *temp_int, int16_t *temp_dec);

#endif

#endif
