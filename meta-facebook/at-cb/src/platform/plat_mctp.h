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

/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _PLAT_MCTP_H
#define _PLAT_MCTP_H

#include "ipmi.h"
#include "plat_i2c.h"

#define I2C_BUS_BMC I2C_BUS9
#define I3C_BUS_BMC I3C_BUS1
#define I2C_ADDR_BIC 0x40
#define I2C_ADDR_BMC 0x20

/* mctp endpoint */
#define MCTP_EID_BMC 0x08

#define MCTP_RESP_DATA_INDEX 4
/* MCTP CC, Netfn, cmd, IPMI CC */
#define MCTP_RESP_HEADER_COUNT 4

void plat_mctp_init(void);
int pal_pldm_send_ipmi_request(ipmi_msg *msg, uint8_t eid);

#endif
