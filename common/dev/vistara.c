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

#include <zephyr.h>
#include <logging/log.h>
#include "libutil.h"
#include "mctp.h"
#include "sensor.h"
#ifdef ENABLE_VISTARA
#include "cci.h"
#include "vistara.h"

LOG_MODULE_REGISTER(vistara);

ddr_slot_info_cache_t g_ddr_slot_cache = { 0 };

__weak uint8_t plat_get_cxl_eid(uint8_t cxl_id)
{
	return 0;
}

__weak void plat_set_dimm_cache(uint8_t *resp_buf, uint8_t cxl_id, uint8_t status)
{
	return;
}

__weak float plat_get_dimm_cache(uint8_t cxl_id, uint8_t dimm_id)
{
	return 0;
}

bool vistara_read_ddr_temp(uint8_t cxl_eid, uint8_t *resp)
{
	CHECK_NULL_ARG_WITH_RETURN(resp, false);

	mctp_cci_msg cci_msg = { 0 };
	cci_msg.hdr.op = CCI_OEM_OP_READ_DDR_TEMP;
	cci_msg.hdr.pl_len = READ_DDR_TEMP_REQ_LEN;

	if (!vistara_cci_command(cxl_eid, cci_msg, resp, READ_DDR_TEMP_RESP_LEN)) {
		LOG_DBG("Failed to read Vistara DDR temperature from EID %d", cxl_eid);
		return false;
	}

	return true;
}

bool vistara_read_ddr_slot_info(uint8_t cxl_eid, uint8_t *resp)
{
	CHECK_NULL_ARG_WITH_RETURN(resp, false);

	mctp_cci_msg cci_msg = { 0 };
	cci_msg.hdr.op = CCI_OEM_OP_DIMM_SLOT_INFO;
	cci_msg.hdr.pl_len = READ_DDR_SLOT_INFO_REQ_LEN;

	if (!vistara_cci_command(cxl_eid, cci_msg, resp, READ_DDR_SLOT_INFO_RESP_LEN)) {
		LOG_DBG("Failed to read Vistara DDR slot info from EID %d", cxl_eid);
		return false;
	}

	return true;
}

int vistara_init_ddr_slot_info(void)
{
	uint8_t resp_buf[READ_DDR_SLOT_INFO_RESP_LEN];
	int success_count = 0;

	// Clear cache
	memset(&g_ddr_slot_cache, 0, sizeof(ddr_slot_info_cache_t));

	// Init master DIMM ID
	for (uint8_t i = 0; i < MAX_CXL_COUNT; i++) {
		g_ddr_slot_cache.master_dimm_id[i] = 0xFF;
	}

	for (uint8_t cxl_id = 0; cxl_id < MAX_CXL_COUNT; cxl_id++) {
		uint8_t cxl_eid = plat_get_cxl_eid(cxl_id);

		if (vistara_read_ddr_slot_info(cxl_eid, resp_buf)) {
			uint32_t *num_slots = (uint32_t *)resp_buf;
			g_ddr_slot_cache.slot_info[cxl_id].num_dimm_slots = *num_slots;

			// 4 byte(slot count) + 16 byte(per DIMM) * 4
			uint8_t *dimm_data = resp_buf + 4;  // skip 4 byte(slot count)

			for (int dimm_idx = 0; dimm_idx < MAX_DIMM_PER_CXL && dimm_idx < *num_slots; dimm_idx++) {
				dimm_slot_info_t *dimm_info = &g_ddr_slot_cache.slot_info[cxl_id].dimm_info[dimm_idx];
				uint8_t *current_dimm = dimm_data + (dimm_idx * 16);

				dimm_info->spd_i2c_addr = current_dimm[0];
				dimm_info->channel_id = current_dimm[1];
				dimm_info->dimm_silk_screen = current_dimm[2];
				dimm_info->dimm_present = current_dimm[3];

				if (dimm_info->dimm_present &&
				    g_ddr_slot_cache.master_dimm_id[cxl_id] == 0xFF) {
					g_ddr_slot_cache.master_dimm_id[cxl_id] = dimm_idx;
					LOG_INF("CXL_%d: Master DIMM selected as DIMM_%d", cxl_id, dimm_idx);
				}
			}

			if (g_ddr_slot_cache.master_dimm_id[cxl_id] == 0xFF) {
				LOG_WRN("CXL_%d: No DIMM present, no master selected", cxl_id);
			}

			success_count++;
		} else {
			LOG_ERR("Failed to read DDR slot info from CXL %d", cxl_id);
		}
	}

	if (success_count > 0) {
		g_ddr_slot_cache.valid = true;
		g_ddr_slot_cache.timestamp = k_uptime_get_32();
	}

	return success_count;
}

bool vistara_get_dimm_present_from_cache(uint8_t dimm_id)
{
	if (!g_ddr_slot_cache.valid) {
		LOG_WRN("DDR slot info cache is not valid");
		return false;
	}

	uint8_t cxl_id = dimm_id / MAX_DIMM_PER_CXL;
	uint8_t dimm_idx = dimm_id % MAX_DIMM_PER_CXL;

	if (cxl_id >= MAX_CXL_COUNT || dimm_idx >= MAX_DIMM_PER_CXL) {
		LOG_DBG("Invalid DIMM ID %d", dimm_id);
		return false;
	}

	if (dimm_idx >= g_ddr_slot_cache.slot_info[cxl_id].num_dimm_slots) {
		LOG_DBG("DIMM index %d exceeds available slots %d for CXL %d",
			dimm_idx, g_ddr_slot_cache.slot_info[cxl_id].num_dimm_slots, cxl_id);
		return false;
	}

	bool present = g_ddr_slot_cache.slot_info[cxl_id].dimm_info[dimm_idx].dimm_present;

	return present;
}

bool vistara_cci_command(uint8_t cxl_eid, mctp_cci_msg cci_msg, uint8_t *resp, uint8_t resp_len)
{
	CHECK_NULL_ARG_WITH_RETURN(resp, false);

	mctp *mctp_inst = NULL;
	mctp_ext_params ext_params = { 0 };

	if (!get_mctp_info_by_eid(cxl_eid, &mctp_inst, &ext_params)) {
		LOG_ERR("Fail to get mctp info via eid: %d", cxl_eid);
		return false;
	}

	memcpy(&cci_msg.ext_params, &ext_params, sizeof(mctp_ext_params));
	if (mctp_cci_read(mctp_inst, &cci_msg, resp, resp_len) != resp_len) {
		LOG_ERR("CCI command 0x%x read fail", cci_msg.hdr.op);
		return false;
	}

	k_msleep(50);

	return true;
}

uint8_t vistara_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	uint8_t cxl_id = cfg->port;
	uint8_t cxl_eid = plat_get_cxl_eid(cxl_id);
	uint8_t dimm_id = cfg->target_addr;
	uint8_t sensor_type = cfg->arg0;
	vistara_init_arg *init_arg = (vistara_init_arg *)cfg->init_args;

	sensor_val *sval = (sensor_val *)reading;
	float f_val = 0;

	switch (sensor_type) {
	case DDR_TEMP: {
		uint8_t resp_buf[READ_DDR_TEMP_RESP_LEN];
		if (init_arg->is_cached) {
			// Get master dimm id from slot info cache
			if (!g_ddr_slot_cache.valid) {
				return SENSOR_FAIL_TO_ACCESS;
			}

			uint8_t master_dimm = g_ddr_slot_cache.master_dimm_id[cxl_id];

			if (master_dimm == 0xFF) {
				return SENSOR_FAIL_TO_ACCESS;
			}

			if (dimm_id == master_dimm) {
				if (!vistara_read_ddr_temp(cxl_eid, resp_buf)) {
					plat_set_dimm_cache(resp_buf, cxl_id,
							    SENSOR_FAIL_TO_ACCESS);
					return SENSOR_FAIL_TO_ACCESS;
				} else {
					plat_set_dimm_cache(resp_buf, cxl_id, SENSOR_READ_SUCCESS);
				}
			}

			f_val = plat_get_dimm_cache(cxl_id, dimm_id);
			if (f_val < 0) {
				return SENSOR_FAIL_TO_ACCESS;
			}

		} else {
			if (!vistara_read_ddr_temp(cxl_eid, resp_buf)) {
				return SENSOR_FAIL_TO_ACCESS;
			} else {
				read_ddr_temp_resp *ddr_temp =
					(read_ddr_temp_resp *)(resp_buf + dimm_id * 8);
				f_val = *((float *)&ddr_temp->dimm_temp);
				LOG_HEXDUMP_INF(ddr_temp->dimm_temp, sizeof(float), "ddr temp");
			}
		}
	} break;
	default:
		LOG_ERR("Invalid vistara sensor type 0x%x", sensor_type);
		return SENSOR_FAIL_TO_ACCESS;
		break;
	}

	sval->integer = (int)f_val & 0xFFFF;
	sval->fraction = (f_val - sval->integer) * 1000;

	return SENSOR_READ_SUCCESS;
}

uint8_t vistara_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	cfg->read = vistara_read;
	return SENSOR_INIT_SUCCESS;
}

static inline int validate_spd_read_params(uint8_t *out, uint8_t dimm_idx, uint16_t offset,
					   uint8_t length)
{
	if (!out || length == 0 || length > VISTARA_SPD_CHUNK_DEFAULT || dimm_idx > 3 ||
	    (offset + length) > VISTARA_SPD_DDR4_TOTAL_BYTES) {
		return -EINVAL;
	}
	return 0;
}

int vistara_read_dimm_spd_chunk_eid(uint8_t cxl_eid, uint8_t dimm_idx, uint16_t offset,
				    uint8_t length, uint8_t *out)
{
	int ret = validate_spd_read_params(out, dimm_idx, offset, length);
	if (ret != 0)
		return ret;

	mctp *inst = NULL;
	mctp_ext_params xp = (mctp_ext_params){ 0 };
	if (!get_mctp_info_by_eid(cxl_eid, &inst, &xp))
		return -EIO;

	uint8_t req[12];
	struct vistara_dimm_spd_read_args a = {
		.spd_id = dimm_idx,
		.offset = offset,
		.num_bytes = length,
	};
	vistara_encode_dimm_spd_read(req, &a);

	mctp_cci_msg msg = { 0 };
	msg.hdr.cci_msg_req_resp = 0;
	msg.hdr.op = CCI_OEM_OP_DIMM_SPD_READ;
	msg.hdr.pl_len = sizeof(req);
	msg.pl_data = req;
	msg.ext_params = xp;
	msg.timeout_ms = pal_get_cci_timeout_ms();

	uint8_t rbuf[VISTARA_CCI_BUFFER_SIZE] = { 0 };
	int rlen = mctp_cci_read(inst, &msg, rbuf, sizeof(rbuf));
	if (rlen < (int)length)
		return -EIO;

	memcpy(out, rbuf, length);
	return 0;
}

bool vistara_read_dimm_spd_ddr4(uint8_t cxl_eid, uint8_t dimm_idx,
				uint8_t out512[VISTARA_SPD_DDR4_TOTAL_BYTES])
{
	uint16_t got = 0;
	while (got < VISTARA_SPD_DDR4_TOTAL_BYTES) {
		uint16_t remain = (uint16_t)(VISTARA_SPD_DDR4_TOTAL_BYTES - got);
		uint8_t chunk = (remain < VISTARA_SPD_CHUNK_DEFAULT) ?
					(uint8_t)remain :
					(uint8_t)VISTARA_SPD_CHUNK_DEFAULT;
		int rc = vistara_read_dimm_spd_chunk_eid(cxl_eid, dimm_idx, got, chunk,
							 &out512[got]);
		if (rc != 0) {
			LOG_ERR("SPD chunk fail (eid=%u dimm=%u off=0x%03X len=%u rc=%d)", cxl_eid,
				dimm_idx, got, chunk, rc);
			return false;
		}
		got += chunk;
	}
	return true;
}

#endif
