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

#ifndef _MCTP_H
#define _MCTP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <sys/printk.h>
#include <zephyr.h>

#define MCTP_DEBUG 1

#define MCTP_SUCCESS 0
#define MCTP_ERROR 1

#define MCTP_RESPONSE 0
#define MCTP_REQUEST 1

#define MCTP_TX_QUEUE_SIZE 16

#define MSG_ASSEMBLY_BUF_SIZE 1024

#define MCTP_RX_TASK_STACK_SIZE 4096
#define MCTP_TX_TASK_STACK_SIZE 2048
#define MCTP_TASK_NAME_LEN 32

#define MCTP_DEFAULT_ENDPOINT 0x0A
#define MCTP_NULL_EID 0x00

#define MCTP_DEFAULT_MSG_MAX_SIZE 64
#define MCTP_TRANSPORT_HEADER_SIZE 4
#define MCTP_MEDIUM_META_SIZE_SMBUS 3
#define MCTP_PEC_SIZE 1 /* SMBUS/I3C */
#define MCTP_META_INFO_SIZE (MCTP_TRANSPORT_HEADER_SIZE + MCTP_PEC_SIZE)

#define MCTP_MAX_MSG_TAG_NUM 8

#define MCTP_HDR_HDR_VER 0x01
#define MCTP_HDR_SEQ_MASK 0x03
#define MCTP_HDR_TAG_MASK 0x07

#define MCTP_POLL_TIME_MS 1

#define MCTP_MSG_TYPE_SHIFT 0
#define MCTP_MSG_TYPE_MASK 0x7F

#define MCTP_IC_SHIFT 7
#define MCTP_IC_MASK 0x80

typedef enum {
	MCTP_MSG_TYPE_CTRL = 0x00,
	MCTP_MSG_TYPE_PLDM,
	MCTP_MSG_TYPE_NCSI,
	MCTP_MSG_TYPE_ETH,
	MCTP_MSG_TYPE_NVME,
	MCTP_MSG_TYPE_CCI = 0x08,
	MCTP_MSG_TYPE_VEN_DEF_PCI = 0x7E,
	MCTP_MSG_TYPE_VEN_DEF_IANA = 0x7F
} MCTP_MSG_TYPE;

typedef enum {
	MCTP_MEDIUM_TYPE_UNKNOWN = 0,
	MCTP_MEDIUM_TYPE_SMBUS,
	MCTP_MEDIUM_TYPE_CONTROLLER_I3C,
	MCTP_MEDIUM_TYPE_TARGET_I3C,
	MCTP_MEDIUM_TYPE_MAX
} MCTP_MEDIUM_TYPE;

/* smbus extra medium data of endpoint */
typedef struct _mctp_i3c_ext_params {
	uint8_t addr; /* 8 bit address */
	uint32_t dummy; // TODO: test only
} mctp_i3c_ext_params;

/* smbus extra medium data of endpoint */
typedef struct _mctp_smbus_ext_params {
	uint8_t addr; /* 8 bit address */
} mctp_smbus_ext_params;

/* mctp extra parameters prototype */
typedef struct _mctp_ext_params {
	/* mctp transport layer parameters */
	uint8_t tag_owner;
	uint8_t msg_tag;
	uint8_t ep;

	/* medium parameters */
	MCTP_MEDIUM_TYPE type;
	union {
		mctp_smbus_ext_params smbus_ext_params;
		mctp_i3c_ext_params i3c_ext_params;
	};
} mctp_ext_params;

/* mctp recevice data callback function prototype */
/* ext_params shoule be bypass to mctp_send_msg if need */
typedef uint8_t (*mctp_fn_cb)(void *mctp_p, uint8_t *buf, uint32_t len, mctp_ext_params ext_params);

/* medium write/read function prototype */
typedef uint16_t (*medium_tx)(void *mctp_p, uint8_t *buf, uint32_t len, mctp_ext_params ext_params);
typedef uint16_t (*medium_rx)(void *mctp_p, uint8_t *buf, uint32_t len,
			      mctp_ext_params *ext_params);

/* prototype for destitation endpoint resloved */
typedef uint8_t (*endpoint_resolve)(uint8_t dest_endpoint, void **mctp_inst,
				    mctp_ext_params *ext_params);

/* i3c config for mctp medium_conf */
typedef struct _mctp_i3c_conf {
	uint8_t bus;
	uint8_t addr;
} mctp_i3c_conf;

/* smbus config for mctp medium_conf */
typedef struct _mctp_smbus_conf {
	uint8_t bus;
	uint8_t addr;
} mctp_smbus_conf;

/* mctp medium conf */
typedef union {
	mctp_smbus_conf smbus_conf;
	mctp_i3c_conf i3c_conf;
} mctp_medium_conf;

/* mctp tx message struct */
typedef struct __attribute__((aligned(4))) {
	uint8_t is_bridge_packet;
	uint8_t *buf;
	uint16_t len;
	mctp_ext_params ext_params;
	struct k_msgq *evt_msgq;
} mctp_tx_msg;

/* mctp main struct */
typedef struct _mctp {
	uint8_t is_servcie_start;
	MCTP_MEDIUM_TYPE medium_type;
	uint8_t endpoint;
	uint16_t max_msg_size;

	/* medium related */
	mctp_medium_conf medium_conf;
	medium_rx read_data;
	medium_tx write_data;

	/* get mctp route information by application layer */
	endpoint_resolve ep_resolve;

	/* read/write task */
	k_tid_t mctp_rx_task_tid;
	k_tid_t mctp_tx_task_tid;
	struct k_thread rx_task_thread_data;
	struct k_thread tx_task_thread_data;
	K_KERNEL_STACK_MEMBER(rx_task_stack_area, MCTP_RX_TASK_STACK_SIZE);
	K_KERNEL_STACK_MEMBER(tx_task_stack_area, MCTP_TX_TASK_STACK_SIZE);
	uint8_t mctp_rx_task_name[MCTP_TASK_NAME_LEN];
	uint8_t mctp_tx_task_name[MCTP_TASK_NAME_LEN];

	/* write queue */
	struct k_msgq mctp_tx_queue;

	/* point to the rx message buffer that is assembling request/response */
	struct {
		uint8_t *buf;
		uint16_t offset;
	} temp_msg_buf[MCTP_MAX_MSG_TAG_NUM][2];

	/* the callback when recevie mctp data */
	mctp_fn_cb rx_cb;

	/* for pldm instance id */
	uint8_t pldm_inst_id;
	uint32_t pldm_inst_table; // 32 bits field for instance id

	/* for cci_msg_tag */
	uint8_t cci_msg_tag;

	/* for ncsi instance id */
	uint8_t ncsi_inst_id;
	uint32_t ncsi_inst_table[8]; // 256 bits field for instance id

} mctp;

typedef struct _mctp_smbus_port {
	mctp *mctp_inst;
	uint8_t channel_target;
	MCTP_MEDIUM_TYPE medium_type;
	mctp_medium_conf conf;
} mctp_port;

/* mctp route entry struct */
typedef struct _mctp_route_entry {
	uint8_t endpoint;
	uint8_t bus; /* TODO: only consider smbus/i3c */
	uint8_t addr; /* TODO: only consider smbus/i3c */
	uint8_t dev_present_pin;
	bool set_endpoint;
} mctp_route_entry;

typedef struct _mctp_msg_handler {
	MCTP_MSG_TYPE type;
	mctp_fn_cb msg_handler_cb;
} mctp_msg_handler;

uint8_t plat_get_mctp_port_count();
mctp_port *plat_get_mctp_port(uint8_t index);

/* public function */
mctp *mctp_init(void);

uint8_t mctp_deinit(mctp *mctp_inst);

uint8_t mctp_set_medium_configure(mctp *mctp_inst, MCTP_MEDIUM_TYPE medium_type,
				  mctp_medium_conf medium_conf);

/* medium_conf should be freed by application */
uint8_t mctp_get_medium_configure(mctp *mctp_inst, MCTP_MEDIUM_TYPE *medium_type,
				  mctp_medium_conf *medium_conf);

/* mctp service start */
uint8_t mctp_start(mctp *mctp_inst);

/* mctp service stop */
uint8_t mctp_stop(mctp *mctp_inst);

/* send message to destination endpoint */
uint8_t mctp_send_msg(mctp *mctp_inst, uint8_t *buf, uint16_t len, mctp_ext_params ext_params);

/* bridge message to destination endpoint */
uint8_t mctp_bridge_msg(mctp *mctp_inst, uint8_t *buf, uint16_t len, mctp_ext_params ext_params);

/* medium init/deinit */
uint8_t mctp_smbus_init(mctp *mctp_inst, mctp_medium_conf medium_conf);
uint8_t mctp_smbus_deinit(mctp *mctp_inst);
uint8_t mctp_i3c_controller_init(mctp *mctp_instance, mctp_medium_conf medium_conf);
uint8_t mctp_i3c_target_init(mctp *mctp_instance, mctp_medium_conf medium_conf);
uint8_t mctp_i3c_deinit(mctp *mctp_instance);

/* register endpoint resolve function */
uint8_t mctp_reg_endpoint_resolve_func(mctp *mctp_inst, endpoint_resolve resolve_fn);

bool get_mctp_info_by_eid(uint8_t port, mctp **mctp_inst, mctp_ext_params *ext_params);
uint8_t get_mctp_info(uint8_t dest_endpoint, mctp **mctp_inst, mctp_ext_params *ext_params);

/* register callback function when the mctp message is received */
uint8_t mctp_reg_msg_rx_func(mctp *mctp_inst, mctp_fn_cb rx_cb);

mctp *pal_get_mctp(uint8_t mctp_medium_type, uint8_t bus);
int pal_get_target(uint8_t interface);
int pal_get_medium_type(uint8_t interface);
uint8_t plat_get_eid();
int pal_find_bus_in_mctp_port(mctp_port *p);
mctp *pal_find_mctp_by_bus(uint8_t bus);
mctp_port *pal_find_mctp_port_by_channel_target(uint8_t target);

#ifdef __cplusplus
}
#endif

#endif /* _MCTP_H */
