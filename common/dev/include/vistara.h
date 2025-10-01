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
#include <sys/byteorder.h>

#define READ_DDR_TEMP_REQ_LEN 0
#define READ_DDR_TEMP_RESP_LEN 32

#define VISTARA_SPD_DDR4_TOTAL_BYTES 512
#define VISTARA_SPD_CHUNK_DEFAULT 64
#define VISTARA_SPD_MODULE_SN_OFF 325
#define VISTARA_SPD_MODULE_SN_LEN 4
#define VISTARA_CCI_BUFFER_SIZE 256

enum VISTARA_CCI_CMD_OEM_OPCODE {
	CCI_OEM_OP_READ_DDR_TEMP = 0xC531,
	CCI_OEM_OP_DIMM_SPD_READ = 0xC510,
};

enum VISTARA_SENSOR_TYPE {
	DDR_TEMP,
};

enum VISTARA_DIMM_ID {
	DIMMA_ID = 0,
	DIMMB_ID,
	DIMMC_ID,
	DIMMD_ID,
	MAX_DIMM_ID,
};

typedef struct {
	uint8_t temp_valid;
	uint8_t dimm_id;
	uint8_t spd_id;
	uint8_t reserved;
	uint8_t dimm_temp[4];
} read_ddr_temp_resp;

struct vistara_dimm_spd_read_args {
	uint32_t spd_id;
	uint32_t offset;
	uint32_t num_bytes;
};

static inline size_t vistara_encode_dimm_spd_read(uint8_t dst[12],
						  const struct vistara_dimm_spd_read_args *a)
{
	sys_put_le32(a->spd_id, &dst[0]);
	sys_put_le32(a->offset, &dst[4]);
	sys_put_le32(a->num_bytes, &dst[8]);
	return 12;
}

uint8_t plat_get_cxl_eid(uint8_t cxl_id);
void plat_set_dimm_cache(uint8_t *resp_buf, uint8_t cxl_id, uint8_t status);
float plat_get_dimm_cache(uint8_t cxl_id, uint8_t dimm_id);
bool vistara_cci_command(uint8_t cxl_eid, mctp_cci_msg cci_msg, uint8_t *resp, uint8_t resp_len);
bool vistara_read_ddr_temp(uint8_t cxl_eid, uint8_t *resp);
int vistara_read_dimm_spd_chunk_eid(uint8_t cxl_eid, uint8_t dimm_idx, uint16_t offset,
				    uint8_t length, uint8_t *out);
bool vistara_read_dimm_spd_ddr4(uint8_t cxl_eid, uint8_t dimm_idx,
				uint8_t out512[VISTARA_SPD_DDR4_TOTAL_BYTES]);
#endif
#endif
