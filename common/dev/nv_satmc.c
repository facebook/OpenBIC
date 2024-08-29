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

#include "plat_def.h"
#ifdef ENABLE_NVIDIA

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <logging/log.h>
#include "libutil.h"
#include "sensor.h"
#include "pldm.h"
#include "hal_i2c.h"
#include "nvidia.h"
#include "pdr.h"

LOG_MODULE_REGISTER(nv_satmc);

struct nv_satmc_sensor_parm satmc_sensor_cfg_list[] = {
	{ NV_SATMC_SENSOR_NUM_PWR_VDD_CPU, { 1, 0, -3 } },
	{ NV_SATMC_SENSOR_NUM_VOL_VDD_CPU, { 1, 0, -3 } },
	{ NV_SATMC_SENSOR_NUM_PWR_VDD_SOC, { 1, 0, -3 } },
	{ NV_SATMC_SENSOR_NUM_VOL_VDD_SOC, { 1, 0, -3 } },
	{ NV_SATMC_SENSOR_NUM_PWR_MODULE, { 1, 0, -3 } },
	{ NV_SATMC_SENSOR_NUM_ENG_MODULE, { 1, 0, 0 } },
	{ NV_SATMC_SENSOR_NUM_PWR_GRACE, { 1, 0, -3 } },
	{ NV_SATMC_SENSOR_NUM_ENG_GRACE, { 1, 0, 0 } },
	{ NV_SATMC_SENSOR_NUM_PWR_TOTAL_MODULE, { 1, 0, -3 } },
	{ NV_SATMC_SENSOR_NUM_ENG_TOTAL_MODULE, { 1, 0, 0 } },
	{ NV_SATMC_SENSOR_NUM_CNT_PAGE_RETIRE, { 1, 0, 0 } },
	{ NV_SATMC_SENSOR_NUM_TMP_GRACE, { 1, 0, -3 } },
	{ NV_SATMC_SENSOR_NUM_TMP_GRACE_LIMIT, { 1, 0, -3 } },
	{ NV_SATMC_SENSOR_NUM_FRQ_MEMORY, { 1, 0, 3 } },
	{ NV_SATMC_SENSOR_NUM_FRQ_MAX_CPU, { 1, 0, 0 } },
};

const int SATMC_SENSOR_CFG_LIST_SIZE = ARRAY_SIZE(satmc_sensor_cfg_list);

#ifdef ENABLE_SBMR
#include "sbmr.h"
sbmr_boot_progress_code_t nv_sbmr_postcode[] = {
	/* POSTCODE: EFI_NV_FW_BOOT_PC_MB1_START */
	[0] = {	.type = NV_STATUS_CODE_TYPE_PROG_CODE,
		.rsvd = 0x0000,
		.severity = 0x00,
		.operation = 0x0005,
		.sub_class = 0x01,
		.class = 0xC1,
		.inst = 0x00,
	},
	/* ERRORCODE: EFI_NV_HW_DRAM_EC_CHANNEL_LOW_COUNT */
	[1] = {	.type = NV_STATUS_CODE_TYPE_ERROR_CODE,
		.rsvd = 0x0000,
		.severity = 0x90,
		.operation = 0x0001,
		.sub_class = 0x05,
		.class = 0xC0,
		.inst = 0x00,
	},
	/* DEBUGCODE: EFI_NV_HW_DRAM_EC_RETIRED_PAGE_OVERFLOW */
	[2] = {	.type = NV_STATUS_CODE_TYPE_DEBUG_CODE,
		.rsvd = 0x0000,
		.severity = 0x01,
		.operation = 0x0004,
		.sub_class = 0x05,
		.class = 0xC0,
		.inst = 0x00,
	},
};
#endif

pldm_sensor_pdr_parm *find_sensor_parm_by_id(uint16_t sensor_id)
{
	for (int i = 0; i < SATMC_SENSOR_CFG_LIST_SIZE; i++) {
		if (satmc_sensor_cfg_list[i].nv_satmc_sensor_id == sensor_id)
			return &satmc_sensor_cfg_list[i].cal_parm;
	}

	return NULL;
}

uint8_t nv_satmc_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	nv_satmc_init_arg *init_arg = (nv_satmc_init_arg *)cfg->init_args;

	sensor_val *sval = (sensor_val *)reading;
	uint8_t resp_buf[256] = { 0 };
	uint8_t req_len = 0;
	float val = 0;

	mctp *mctp_inst = NULL;
	mctp_ext_params ext_params = { 0 };
	if (get_mctp_info_by_eid(init_arg->endpoint, &mctp_inst, &ext_params) == false) {
		LOG_ERR("Failed to get mctp info by eid 0x%x", init_arg->endpoint);
		return SENSOR_FAIL_TO_ACCESS;
	}

	if ((init_arg->sensor_id == NV_SATMC_SENSOR_NUM_CPU_THROT_STATE) ||
	    (init_arg->sensor_id == NV_SATMC_SENSOR_NUM_POWER_BREAK) ||
	    (init_arg->sensor_id == NV_SATMC_SENSOR_NUM_SPARE_CH_PRESENCE)) {
		req_len = sizeof(struct pldm_get_state_sensor_reading_req);
		struct pldm_get_state_sensor_reading_req req = { 0 };
		req.sensor_id = init_arg->sensor_id;

		uint16_t resp_len = pldm_platform_monitor_read(
			mctp_inst, ext_params, PLDM_MONITOR_CMD_CODE_GET_STATE_SENSOR_READING,
			(uint8_t *)&req, req_len, resp_buf, sizeof(resp_buf));

		if (resp_len == 0) {
			LOG_ERR("Failed to get SatMC sensor #0x%x reading", init_arg->sensor_id);
			return SENSOR_FAIL_TO_ACCESS;
		}

		struct pldm_get_state_sensor_reading_resp *res =
			(struct pldm_get_state_sensor_reading_resp *)resp_buf;

		if (res->completion_code != PLDM_SUCCESS) {
			LOG_ERR("Get SatMC sensor #%04x with bad cc 0x%x", init_arg->sensor_id,
				res->completion_code);
			return SENSOR_FAIL_TO_ACCESS;
		}

		if (init_arg->state_sensor_idx >= res->composite_sensor_count) {
			LOG_ERR("Failed to get SatMC sensor #0x%x state with index %d",
				init_arg->sensor_id, init_arg->state_sensor_idx);
			return SENSOR_UNSPECIFIED_ERROR;
		}

		if (res->field[init_arg->state_sensor_idx].sensor_op_state != PLDM_SENSOR_ENABLED) {
			LOG_WRN("SatMC sensor #%04x in abnormal op state 0x%x", init_arg->sensor_id,
				res->field[init_arg->state_sensor_idx].sensor_op_state);
			return SENSOR_NOT_ACCESSIBLE;
		}

		val = (float)res->field[init_arg->state_sensor_idx].present_state;
		goto exit;
	}

	if (init_arg->is_init == false) {
		pldm_sensor_pdr_parm *parm_cfg = find_sensor_parm_by_id(init_arg->sensor_id);
		if (!parm_cfg) {
			LOG_WRN("SatMC sensor #0x%x PDR not ready", init_arg->sensor_id);
			return SENSOR_UNSPECIFIED_ERROR;
		}
		init_arg->parm = *parm_cfg;
		init_arg->is_init = true;
	}

	req_len = sizeof(struct pldm_get_sensor_reading_req);
	struct pldm_get_sensor_reading_req req = { 0 };
	req.sensor_id = init_arg->sensor_id;
	req.rearm_event_state = 0;

	uint16_t resp_len =
		pldm_platform_monitor_read(mctp_inst, ext_params,
					   PLDM_MONITOR_CMD_CODE_GET_SENSOR_READING,
					   (uint8_t *)&req, req_len, resp_buf, sizeof(resp_buf));

	if (resp_len == 0) {
		LOG_ERR("Failed to get SatMC sensor #0x%x reading", init_arg->sensor_id);
		return SENSOR_FAIL_TO_ACCESS;
	}

	struct pldm_get_sensor_reading_resp *res = (struct pldm_get_sensor_reading_resp *)resp_buf;

	if (res->completion_code != PLDM_SUCCESS) {
		LOG_ERR("Get SatMC sensor #%04x with bad cc 0x%x", init_arg->sensor_id,
			res->completion_code);
		return SENSOR_FAIL_TO_ACCESS;
	}

	if (res->sensor_operational_state != PLDM_SENSOR_ENABLED) {
		LOG_WRN("SatMC sensor #%04x in abnormal op state 0x%x", init_arg->sensor_id,
			res->sensor_operational_state);
		return SENSOR_NOT_ACCESSIBLE;
	}

	LOG_DBG("SatMC sensor#0x%04x", init_arg->sensor_id);
	LOG_HEXDUMP_DBG(res->present_reading, resp_len - 7, "");

	val = pldm_sensor_cal(res->present_reading, resp_len - 7, res->sensor_data_size,
			      init_arg->parm);

exit:
	sval->integer = (int)val & 0xFFFF;
	sval->fraction = (val - sval->integer) * 1000;

	LOG_DBG("SatMC sensor #0x%04x --> %d.%03d", init_arg->sensor_id, sval->integer,
		sval->fraction);

	return SENSOR_READ_SUCCESS;
}

uint8_t nv_satmc_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	cfg->read = nv_satmc_read;
	return SENSOR_INIT_SUCCESS;
}

#endif
