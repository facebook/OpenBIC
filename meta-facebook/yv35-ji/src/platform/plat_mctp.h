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

#ifndef _PLAT_MCTP_h
#define _PLAT_MCTP_h
#include <kernel.h>
#include "storage_handler.h"
#include "pldm.h"

/* mctp endpoint */
#define MCTP_EID_BMC 0x08
#define MCTP_EID_SATMC 0xF0

struct mctp_to_ipmi_header_req {
	uint8_t iana[IANA_LEN];
	uint8_t netfn_lun;
	uint8_t ipmi_cmd;
} __attribute__((__packed__));
;

struct mctp_to_ipmi_header_resp {
	uint8_t completion_code;
	uint8_t netfn_lun;
	uint8_t ipmi_cmd;
	uint8_t ipmi_comp_code;
} __attribute__((__packed__));
;

enum {
	ADD_COMMON_SEL = 0x01,
	ADD_OEM_SEL = 0x02,
};

struct mctp_to_ipmi_sel_req {
	struct mctp_to_ipmi_header_req header;
	struct ipmi_storage_add_sel_req req_data;
} __attribute__((__packed__));

struct mctp_to_ipmi_sel_resp {
	struct mctp_to_ipmi_header_resp header;
	struct ipmi_storage_add_sel_resp resp_data;
} __attribute__((__packed__));

/* init the mctp moduel for platform */
void plat_mctp_init(void);
void send_cmd_to_dev(struct k_timer *timer);
void send_cmd_to_dev_handler(struct k_work *work);
bool mctp_add_sel_to_ipmi(struct ipmi_storage_add_sel_req *sel_msg, uint8_t sel_type);
uint8_t plat_get_mctp_port_count();
mctp_port *plat_get_mctp_port(uint8_t index);
void satmc_status_update();

#endif /* _PLAT_MCTP_h */
