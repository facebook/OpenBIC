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

#include "pldm.h"
#include <zephyr/logging/log.h>
#include <string.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/slist.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/crc.h>
#include <zephyr/kernel.h>
#include "libutil.h"

LOG_MODULE_DECLARE(pldm);

__weak uint8_t plat_pldm_get_tid()
{
	return DEFAULT_TID;
}

/* TID assigned by a bus owner via SetTID. 0 (and 0xFF) mean "unassigned"
 * per DSP0240 - until then GetTID reports the board default. */
static uint8_t assigned_tid;

uint8_t set_tid(void *mctp_inst, uint8_t *buf, uint16_t len, uint8_t instance_id, uint8_t *resp,
		uint16_t *resp_len, void *ext_params)
{
	if (!mctp_inst || !buf || !resp || !resp_len)
		return PLDM_ERROR;

	struct _set_tid_req *req_p = (struct _set_tid_req *)buf;
	struct _set_tid_resp *resp_p = (struct _set_tid_resp *)resp;

	*resp_len = 1;

	if (sizeof(*req_p) != len) {
		resp_p->completion_code = PLDM_ERROR_INVALID_LENGTH;
		return PLDM_SUCCESS;
	}

	/* Persist so GetTID reflects it. 0x00/0xFF are the reserved
	 * "null" values - accept but treat as clearing the assignment. */
	assigned_tid = (req_p->tid == 0x00 || req_p->tid == 0xFF) ? 0 : req_p->tid;
	resp_p->completion_code = PLDM_SUCCESS;
	return PLDM_SUCCESS;
}

uint8_t get_tid(void *mctp_inst, uint8_t *buf, uint16_t len, uint8_t instance_id, uint8_t *resp,
		uint16_t *resp_len, void *ext_params)
{
	if (!mctp_inst || !buf || !resp || !resp_len)
		return PLDM_ERROR;

	struct _get_tid_resp *p = (struct _get_tid_resp *)resp;
	p->completion_code = PLDM_SUCCESS;
	p->tid = assigned_tid ? assigned_tid : plat_pldm_get_tid();
	*resp_len = sizeof(*p);
	return PLDM_SUCCESS;
}

/* GetPLDMVersion (DSP0240 0x03). Single-part response: this terminus
 * implements PLDM 1.0.0 for every type it advertises in GetPLDMTypes.
 * transferCRC is CRC-32 over the version data. */
uint8_t get_pldm_version(void *mctp_inst, uint8_t *buf, uint16_t len, uint8_t instance_id,
			 uint8_t *resp, uint16_t *resp_len, void *ext_params)
{
	if (!mctp_inst || !buf || !resp || !resp_len)
		return PLDM_ERROR;

	struct _get_pldm_version_req *req = (struct _get_pldm_version_req *)buf;
	struct _get_pldm_version_resp *r = (struct _get_pldm_version_resp *)resp;

	if (len != sizeof(*req)) {
		resp[0] = PLDM_ERROR_INVALID_LENGTH;
		*resp_len = 1;
		return PLDM_SUCCESS;
	}

	/* Is this type one we actually support? Reuse the GetPLDMTypes
	 * bitmap so the two answers can never disagree. */
	uint8_t types[GET_PLDM_TYPE_BUF_SIZE] = { 0 };

	(void)get_supported_pldm_type(types, sizeof(types));
	if (req->pldm_type >= (GET_PLDM_TYPE_BUF_SIZE * 8) ||
	    !(types[req->pldm_type / 8] & BIT(req->pldm_type % 8))) {
		resp[0] = INVALID_PLDM_TYPE_IN_REQUEST_DATA;
		*resp_len = 1;
		return PLDM_SUCCESS;
	}

	r->completion_code = PLDM_SUCCESS;
	r->next_data_transfer_handle = 0;
	r->transfer_flag = PLDM_START_AND_END;
	r->version = 0xF1F0F000; /* 1.0.0 */
	r->crc = crc32_ieee((const uint8_t *)&r->version, sizeof(r->version));
	*resp_len = sizeof(*r);
	return PLDM_SUCCESS;
}

uint8_t get_pldm_types(void *mctp_inst, uint8_t *buf, uint16_t len, uint8_t instance_id,
		       uint8_t *resp, uint16_t *resp_len, void *ext_params)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(buf, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp_len, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(ext_params, PLDM_ERROR);

	struct _get_pldm_types_resp *p = (struct _get_pldm_types_resp *)resp;

	if (len > 0) {
		p->completion_code = PLDM_ERROR_INVALID_LENGTH;
		*resp_len = 1;
		return PLDM_SUCCESS;
	}

	uint8_t rc = get_supported_pldm_type(p->pldm_types, sizeof(p->pldm_types));

	*resp_len = (rc == PLDM_SUCCESS) ? sizeof(*p) : 1;
	p->completion_code = rc;
	return PLDM_SUCCESS;
}

uint8_t get_pldm_commands(void *mctp_inst, uint8_t *buf, uint16_t len, uint8_t instance_id,
			  uint8_t *resp, uint16_t *resp_len, void *ext_params)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(buf, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp_len, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(ext_params, PLDM_ERROR);

	struct _get_pldm_commands_req *req_p = (struct _get_pldm_commands_req *)buf;
	struct _get_pldm_commands_resp *resp_p = (struct _get_pldm_commands_resp *)resp;

	if (len != sizeof(struct _get_pldm_commands_req)) {
		resp_p->completion_code = PLDM_ERROR_INVALID_LENGTH;
		*resp_len = 1;
		return PLDM_SUCCESS;
	}

	uint8_t rc = get_supported_pldm_commands(req_p->type, resp_p->pldm_commands,
						 sizeof(resp_p->pldm_commands));

	switch (rc) {
	case PLDM_ERROR_INVALID_PLDM_TYPE:
		resp_p->completion_code = INVALID_PLDM_TYPE_IN_REQUEST_DATA;
		break;

	case PLDM_ERROR:
	case PLDM_ERROR_INVALID_LENGTH:
		resp_p->completion_code = PLDM_ERROR;
		break;

	case PLDM_SUCCESS:
		resp_p->completion_code = PLDM_SUCCESS;
		break;

	default:
		resp_p->completion_code = PLDM_ERROR;
		break;
	}

	*resp_len = sizeof(*resp_p);
	return rc;
}

static pldm_cmd_handler pldm_base_cmd_tbl[] = {
	{ PLDM_BASE_CMD_CODE_SETTID, set_tid },
	{ PLDM_BASE_CMD_CODE_GETTID, get_tid },
	{ PLDM_BASE_CMD_CODE_GET_PLDM_VER, get_pldm_version },
	{ PLDM_BASE_CMD_CODE_GET_PLDM_TYPE, get_pldm_types },
	{ PLDM_BASE_CMD_CODE_GET_PLDM_CMDS, get_pldm_commands },
};

uint8_t pldm_base_handler_query(uint8_t code, void **ret_fn)
{
	if (!ret_fn)
		return PLDM_ERROR;

	pldm_cmd_proc_fn fn = NULL;
	uint8_t i;

	for (i = 0; i < ARRAY_SIZE(pldm_base_cmd_tbl); i++) {
		if (pldm_base_cmd_tbl[i].cmd_code == code) {
			fn = pldm_base_cmd_tbl[i].fn;
			break;
		}
	}

	*ret_fn = (void *)fn;
	return fn ? PLDM_SUCCESS : PLDM_ERROR;
}
