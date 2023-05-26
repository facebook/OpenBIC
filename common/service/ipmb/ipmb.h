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

#ifndef IPMB_H
#define IPMB_H

#include <devicetree.h>
#include <stdio.h>

#if DT_NODE_EXISTS(DT_NODELABEL(ipmb0))
#define DEV_IPMB_0
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(ipmb1))
#define DEV_IPMB_1
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(ipmb2))
#define DEV_IPMB_2
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(ipmb3))
#define DEV_IPMB_3
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(ipmb4))
#define DEV_IPMB_4
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(ipmb5))
#define DEV_IPMB_5
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(ipmb6))
#define DEV_IPMB_6
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(ipmb7))
#define DEV_IPMB_7
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(ipmb8))
#define DEV_IPMB_8
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(ipmb9))
#define DEV_IPMB_9
#endif

#define DEBUG_IPMB 0

#define SEQ_NUM 64
#define MAX_SEQ_QUENE 15
#define MEM_ALLOCATE_RETRY_TIME 2
#define IPMI_DATA_MAX_LENGTH 520
#define IPMB_REQ_HEADER_LENGTH 6
#define IPMB_RESP_HEADER_LENGTH 7
#define IPMI_MSG_MAX_LENGTH (IPMI_DATA_MAX_LENGTH + IPMB_RESP_HEADER_LENGTH)
#define IPMB_TX_RETRY_TIME 5
#define IPMB_TXQUEUE_LEN 1
#define IPMB_RXQUEUE_LEN 2
#define IPMB_TX_STACK_SIZE 3072
#define IPMB_RX_STACK_SIZE 3072
#define IPMI_HEADER_CHECKSUM_POSITION 2
#define IPMB_NETFN_MASK 0xFC
#define IPMB_DEST_LUN_MASK 0x03
#define IPMB_SEQ_MASK 0xFC
#define IPMB_SRC_LUN_MASK 0x03
#define IPMB_RETRY_DELAY_MS 500
#define IPMB_POLLING_TIME_MS 1
#define IPMB_SEQ_TIMEOUT_MS 3000
#define IPMB_SEQ_TIMEOUT_STACK_SIZE 512
#define I2C_RETRY_TIME 5

#define RESERVED_IDX 0xFF
#define RESERVED_BUS 0xFF
#define RESERVED_ADDRESS 0xFF

#define ENABLE 1
#define DISABLE 0

#define IS_RESPONSE(msg) (msg.netfn & 0x01)

enum Channel_Target {
	SELF = 0x0,
	ME_IPMB = 0x01,
	BMC_IPMB = 0x02,
	SERVER_IPMB = 0x04,
	EXP1_IPMB = 0x05,
	SLOT1_BIC = 0x07,
	SLOT3_BIC = 0x08,
	BB_IPMB = 0x10,
	EXP2_IPMB = 0x15,
	CL_BIC_IPMB = 0x16, // Rainbow falls IPMB channel
	HD_BIC_IPMB = 0x17,
	GL_BIC_IPMB = 0x18, // for NF IPMB channel
	/* 19h reserved. */
	EXP3_IPMB = 0x25,
	EXP4_IPMB = 0x30,
	PEER_BMC_IPMB = 0x1A,
	/* 1Bh-1Fh reserved. */
	BMC_USB = 0x20,
	/* 21h-39h reserved. */
	PLDM = 0x40,
	MCTP = 0x41,
	/* 41h-4Fh reserved. */
	HOST_KCS_1 = 0x50,
	HOST_KCS_2 = 0x51,
	HOST_KCS_3 = 0x52,
	HOST_KCS_4 = 0x53,
	/* 54h-5Fh are reserved for KCS */
	HOST_SSIF_1 = 0x60,
	/* 61h-6Fh are reserved for SSIF */
	RESERVED,
};

enum Interface {
	I2C_IF,
	I3C_IF,
	RESERVED_IF,
};

typedef struct IPMB_config {
	uint8_t index;
	uint8_t interface;
	uint8_t channel;
	uint8_t bus;
	uint8_t channel_target_address;
	bool enable_status;
	uint8_t self_address;
	char *rx_thread_name;
	char *tx_thread_name;
} IPMB_config;

extern IPMB_config *IPMB_config_table;

typedef enum ipmb_error {
	IPMB_ERROR_SUCCESS = 0, /**< Generic no-error flag  */
	IPMB_ERROR_UNKNOWN, /**< Unknown error */
	IPMB_ERROR_FAILURE, /**< Generic failure on IPMB */
	IPMB_ERROR_TIMEOUT, /**< Error raised when a message takes too long to be responded */
	IPMB_ERROR_INVALID_REQ, /**< A invalid request was received */
	IPMB_ERROR_HDR_CHECKSUM, /**< Invalid header checksum from incoming message */
	IPMB_ERROR_MSG_CHECKSUM, /**< Invalid message checksum from incoming message */
	IPMB_ERROR_QUEUE_CREATION, /**< Client queue couldn't be created. Invalid pointer to handler was given */
	IPMB_ERROR_GET_MESSAGE_QUEUE, /**< Failure on getting queue message */
	IPMB_ERROR_MUTEX_LOCK, /**< Fail to lock mutex in time */
} ipmb_error;

typedef struct ipmi_msg {
	uint8_t dest_addr; /**< Destination target address */
	uint8_t netfn; /**< Net Function */
	uint8_t dest_LUN; /**< Destination LUN (Logical Unit Number) */
	uint8_t hdr_chksum; /**< Connection Header Checksum */
	uint8_t src_addr; /**< Source target address */
	uint8_t seq_source; /**< Source sequence Number */
	uint8_t seq_target; /**< Target sequence Number */
	uint8_t seq; /**< Sequence Number */
	uint8_t pldm_inst_id; /**< Sequence Number */
	uint8_t InF_source; /**< Source bridge interface */
	uint8_t InF_target; /**< Target bridge interface */
	uint8_t src_LUN; /**< Source LUN (Logical Unit Number) */
	uint8_t cmd; /**< Command */
	uint8_t completion_code; /**< Completion Code*/
	uint16_t data_len; /**< Amount of valid bytes in #data buffer */
	uint8_t data[IPMI_MSG_MAX_LENGTH]; /**< Data buffer >
                                         * Data field has 24 bytes:
                                         * 32 (Max IPMI msg len) - 7 header bytes - 1 final chksum byte
                                         */
	uint32_t timestamp; /**< Tick count at the beginning of the process */
	uint8_t msg_chksum; /**< Message checksum */
} __attribute__((packed, aligned(4))) ipmi_msg;

typedef struct ipmi_msg_cfg {
	ipmi_msg buffer; /**< IPMI Message */
	uint8_t retries; /**< Current retry counter */
	struct ipmi_msg_cfg *next;
} __attribute__((packed, aligned(4))) ipmi_msg_cfg;

bool pal_load_ipmb_config(void);
bool pal_is_interface_use_ipmb(uint8_t interface_index);
void ipmb_init(void);
ipmb_error ipmb_send_request(ipmi_msg *req, uint8_t index);
ipmb_error ipmb_send_response(ipmi_msg *resp, uint8_t index);
ipmb_error ipmb_read(ipmi_msg *msg, uint8_t bus);
void ipmb_tx_suspend(uint8_t index);
void ipmb_tx_resume(uint8_t index);

void pal_encode_response_bridge_cmd(ipmi_msg *bridge_msg, ipmi_msg_cfg *current_msg_rx,
				    IPMB_config *ipmb_cfg, IPMB_config *IPMB_config_tables);

#endif
