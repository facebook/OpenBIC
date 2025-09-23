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

#define MCTP_MSG_TYPE_SHIFT 0
#define MCTP_MSG_TYPE_MASK 0x7F
#define MCTP_IC_SHIFT 7
#define MCTP_IC_MASK 0x80

/* i2c dev bus*/
#define I2C_BUS_BMC 0
#define I2C_BUS_CXL1 0x01
#define I2C_BUS_CXL2 0x03

// i2c dev address
#define I2C_ADDR_BIC 0x40
#define I2C_ADDR_BMC 0x20
#define I2C_ADDR_CXL1 0x64
#define I2C_ADDR_CXL2 0x64

// i3c dev bus
#define I3C_BUS_SD_BIC 0
#define I3C_BUS_BMC I3C_BUS_SD_BIC

// i3c dev address
#define I3C_ADDR_SD_BIC 0x8
#define I3C_STATIC_ADDR_BMC I3C_ADDR_SD_BIC

// mctp endpoint
#define MCTP_EID_BMC 0x08

// dynamic allocate eid
#define MCTP_EID_SD_BIC 0
#define MCTP_EID_CXL1 0
#define MCTP_EID_CXL2 0

#define UNKNOWN_CXL_EID 0xFF

#define SET_DEV_ENDPOINT_STACK_SIZE 1024

/* init the mctp moduel for platform */
void plat_mctp_init(void);
uint8_t plat_get_mctp_port_count();
mctp_port *plat_get_mctp_port(uint8_t index);
void create_set_dev_endpoint_thread();
uint8_t plat_get_eid();
mctp *find_mctp_by_bus(uint8_t bus);
bool check_cxl_eid(uint8_t cxl_id);
bool set_cxl_eid(uint8_t cxl_id);
uint8_t plat_get_cxl_eid(uint8_t cxl_id);

#endif /* _PLAT_MCTP_h */
