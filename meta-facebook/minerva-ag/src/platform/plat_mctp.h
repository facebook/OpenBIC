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

#include "storage_handler.h"
#include "pldm_oem.h"
#include "plat_i2c.h"

/* i2c 8 bit address */
#define I2C_ADDR_BIC 0x40
#define I2C_ADDR_BMC 0x20

/* i2c dev bus */
#define I2C_BUS_BMC I2C_BUS4

/* mctp endpoint */
#define MCTP_EID_BMC 0x08

/* I3C related defines */ //Not used in this platform
#define I3C_BUS_BMC I2C_BUS4
#define I3C_STATIC_ADDR_BMC 0x20

struct mctp_to_ipmi_header_req {
	uint8_t iana[IANA_LEN];
	uint8_t netfn_lun;
	uint8_t ipmi_cmd;
} __attribute__((__packed__));
;

struct mctp_to_ipmi_header_resp {
	uint8_t completion_code;
	uint8_t iana[IANA_LEN];
	uint8_t netfn_lun;
	uint8_t ipmi_cmd;
	uint8_t ipmi_comp_code;
} __attribute__((__packed__));
;

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
bool mctp_add_sel_to_ipmi(common_addsel_msg_t *sel_msg);

/* find mctp instance by bus */
mctp *find_mctp_by_bus(uint8_t bus);

extern struct pldm_variable_field nic_vesion[];

#endif /* _PLAT_MCTP_h */
