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

#include <stdint.h>
#include "sbmr.h"
#include "pldm.h"
#include "plat_mctp.h"
#include "libutil.h"
#include "util_worker.h"
#include "storage_handler.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(plat_sbmr);

static void sent_pc_err_event_work_handler(void *arg1, uint32_t arg2)
{
	CHECK_NULL_ARG(arg1);
	ARG_UNUSED(arg2);

	struct ipmi_storage_add_sel_req *add_sel_msg = (struct ipmi_storage_add_sel_req *)arg1;

	if (!mctp_add_sel_to_ipmi(add_sel_msg, ADD_OEM_SEL)) {
		LOG_ERR("Failed to add POSTCODE error sel.");
	}
}

void pal_sbmr_postcode_handler(sbmr_boot_progress_code_t boot_progress_code)
{
	bool need_add_sel = false;
	uint8_t failure_event_type = 0xFF;

	if (!memcmp(&boot_progress_code, &nv_sbmr_postcode[0], sizeof(sbmr_boot_progress_code_t))) {
		sbmr_frb3_flag_clr = true; // Clear FRB3 flag while MB1 starts
	} else if (!memcmp(&boot_progress_code, &nv_sbmr_postcode[1],
			   sizeof(sbmr_boot_progress_code_t))) {
		failure_event_type = 0x0B; // DRAM page retired
		need_add_sel = true;
	} else if (!memcmp(&boot_progress_code, &nv_sbmr_postcode[2],
			   sizeof(sbmr_boot_progress_code_t))) {
		failure_event_type = 0x0C; // DRAM channel retired
		need_add_sel = true;
	}

	if (need_add_sel == false)
		return;

	struct ipmi_storage_add_sel_req add_sel_event_req = { 0 };
	add_sel_event_req.oem_event.general_info = 0x28; // System post event
	add_sel_event_req.oem_event.failure_event_type = failure_event_type;

	worker_job job = { 0 };
	job.delay_ms = 0;
	job.fn = sent_pc_err_event_work_handler;
	job.ptr_arg = &add_sel_event_req;
	add_work(&job);

	return;
}
