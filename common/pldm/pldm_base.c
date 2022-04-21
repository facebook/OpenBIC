#include "pldm.h"
#include <logging/log.h>
#include <string.h>
#include <sys/printk.h>
#include <sys/slist.h>
#include <sys/util.h>
#include <zephyr.h>

LOG_MODULE_DECLARE(pldm);

uint8_t set_tid(void *mctp_inst, uint8_t *buf, uint16_t len, uint8_t *resp, uint16_t *resp_len,
		void *ext_params)
{
	if (!mctp_inst || !buf || !resp || !resp_len)
		return PLDM_ERROR;

	struct _set_tid_req *req_p = (struct _set_tid_req *)buf;
	struct _set_tid_resp *resp_p = (struct _set_tid_resp *)resp;

	*resp_len = 1;
	resp_p->completion_code = (sizeof(*req_p) != len) ? PLDM_BASE_CODES_ERROR_INVALID_LENGTH :
								  PLDM_BASE_CODES_SUCCESS;
	return PLDM_SUCCESS;
}

uint8_t get_tid(void *mctp_inst, uint8_t *buf, uint16_t len, uint8_t *resp, uint16_t *resp_len,
		void *ext_params)
{
	if (!mctp_inst || !buf || !resp || !resp_len)
		return PLDM_ERROR;

	struct _get_tid_resp *p = (struct _get_tid_resp *)resp;
	p->completion_code = PLDM_BASE_CODES_SUCCESS;
	p->tid = DEFAULT_TID;
	*resp_len = sizeof(*p);
	return PLDM_SUCCESS;
}

static pldm_cmd_handler pldm_base_cmd_tbl[] = { { PLDM_BASE_CMD_CODE_SETTID, set_tid },
						{ PLDM_BASE_CMD_CODE_GETTID, get_tid } };

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