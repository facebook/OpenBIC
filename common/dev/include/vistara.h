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

#ifndef VISTARA_H
#define VISTARA_H

#ifdef ENABLE_VISTARA
#include "cci.h"

#define READ_DDR_TEMP_REQ_LEN 0
#define READ_DDR_TEMP_RESP_LEN 32

enum VISTARA_CCI_CMD_OEM_OPCODE {
	CCI_OEM_OP_READ_DDR_TEMP = 0xC531,
};

enum VISTARA_SENSOR_TYPE {
	DDR_TEMP,
};

enum VISTARA_DIMM_ID {
	DIMMA_ID = 0,
	DIMMB_ID,
	DIMMC_ID,
	DIMMD_ID,
};

typedef struct {
	uint8_t temp_valid;
	uint8_t dimm_id;
	uint8_t spd_id;
	uint8_t reserved;
	uint8_t dimm_temp[4];
} read_ddr_temp_resp;

uint8_t plat_get_cxl_eid(uint8_t cxl_id);
bool vistara_cci_command(uint8_t cxl_eid, mctp_cci_msg cci_msg, uint8_t *resp, uint8_t resp_len);
bool vistara_read_ddr_temp(uint8_t cxl_eid, uint8_t *resp);

#endif
#endif
