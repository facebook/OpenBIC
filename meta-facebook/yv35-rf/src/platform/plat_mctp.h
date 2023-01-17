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
#define MCTP_MSG_TYPE_SHIFT 0
#define MCTP_MSG_TYPE_MASK 0x7F
#define MCTP_IC_SHIFT 7
#define MCTP_IC_MASK 0x80

/* i2c 8 bit address */
#define I2C_ADDR_BIC 0x20
#define I2C_ADDR_CXL0 0x74
#define I2C_ADDR_CXL1 0xC2
/* i2c dev bus */
#define I2C_BUS_CXL 0x01
/* mctp endpoint */
#define CXL_EID 0x2E

typedef struct _mctp_smbus_port {
	mctp *mctp_inst;
	mctp_medium_conf conf;
	uint8_t user_idx;
} mctp_smbus_port;

/* mctp route entry struct */
typedef struct _mctp_route_entry {
	uint8_t endpoint;
	uint8_t bus; /* TODO: only consider smbus/i3c */
	uint8_t addr; /* TODO: only consider smbus/i3c */
	uint8_t dev_present_pin;
} mctp_route_entry;

typedef struct _mctp_msg_handler {
	MCTP_MSG_TYPE type;
	mctp_fn_cb msg_handler_cb;
} mctp_msg_handler;

/* init the mctp moduel for platform */
void send_cmd_to_dev(struct k_timer *timer);
void send_cmd_to_dev_handler(struct k_work *work);
void plat_mctp_init(void);
uint8_t get_mctp_route_info(uint8_t dest_endpoint, void **mctp_inst, mctp_ext_params *ext_params);
mctp *find_mctp_by_smbus(uint8_t bus);
mctp *get_mctp_init();
uint8_t get_mctp_info(uint8_t dest_endpoint, mctp **mctp_inst, mctp_ext_params *ext_params);

#endif /* _PLAT_MCTP_h */