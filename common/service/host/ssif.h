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

#ifndef SSIF_H
#define SSIF_H

#include "plat_def.h"
#ifdef ENABLE_SSIF

#include <stdbool.h>
#include <stdint.h>
#include <zephyr.h>
#include "ipmb.h"
#include "hal_i2c_target.h"

#define SSIF_THREAD_STACK_SIZE 4096
#define SSIF_POLLING_INTERVAL 100
#define SSIF_MAX_IPMI_DATA_SIZE 32
#define SSIF_BUFF_SIZE 50

#define SSIF_TASK_NAME_LEN 32

#define SSIF_MULTI_RD_KEY 0x0001

#define CMD_SYS_INFO_FW_VERSION 0x01

#define SSIF_ERR_RCD_SIZE 100

typedef enum ssif_status {
	SSIF_STATUS_WAIT_FOR_WR_START,
	SSIF_STATUS_WAIT_FOR_WR_NEXT,
	SSIF_STATUS_WAIT_FOR_RD_START,
	SSIF_STATUS_WAIT_FOR_RD_NEXT,

	SSIF_STATUS_WR_SINGLE_START = 0x10,
	SSIF_STATUS_WR_MULTI_START,
	SSIF_STATUS_WR_MIDDLE,
	SSIF_STATUS_WR_END,

	SSIF_STATUS_RD_START = 0x20,
	SSIF_STATUS_RD_MIDDLE,
	SSIF_STATUS_RD_RETRY,
	SSIF_STATUS_RD_END,
} ssif_status_t;

typedef enum ssif_err_status {
	SSIF_STATUS_NO_ERR,
	SSIF_STATUS_INVALID_CMD,
	SSIF_STATUS_INVALID_CMD_IN_CUR_STATUS,
	SSIF_STATUS_INVALID_PEC,
	SSIF_STATUS_INVALID_LEN,
	SSIF_STATUS_ADDR_LCK_TIMEOUT, // Address lock timeout
	SSIF_STATUS_ADDR_LOCK_ERR, // Address lock lock/unlock error
	SSIF_STATUS_MUTEX_ERR, // Mutex error in ssif set data
	SSIF_STATUS_RSP_MSG_TIMEOUT, // Failed to send out msg or receive response msg
	SSIF_STATUS_RSP_NOT_READY, // Can't get response msg while data collect
	SSIF_STATUS_TARGET_WR_RD_ERROR, // I2C target write read error
	SSIF_STATUS_UNKNOWN_ERR = 0xFF,
} ssif_err_status_t;

enum ssif_cmd {
	SSIF_WR_SINGLE = 0x02,
	SSIF_WR_MULTI_START = 0x06,
	SSIF_WR_MULTI_MIDDLE = 0x07,
	SSIF_WR_MULTI_END = 0x08,

	SSIF_RD_START = 0x03, // SINGLE/MULTI
	SSIF_RD_NEXT = 0x09, // MIDDLE/END
	SSIF_RD_RETRY = 0x0A,
};

typedef enum ssif_action {
	SSIF_SEND_IPMI,
	SSIF_COLLECT_DATA,
} ssif_action_t;

struct ssif_init_cfg {
	uint8_t i2c_bus;
	uint8_t addr; // bic itself, 7bit
	uint8_t target_msgq_cnt; // maximum msg count for target msg queue
};

typedef struct _ssif_dev {
	uint8_t index;
	uint8_t i2c_bus;
	uint8_t addr; // bic itself, 7bit
	bool addr_lock;
	int64_t exp_to_ms;
	k_tid_t ssif_task_tid;
	K_KERNEL_STACK_MEMBER(ssif_task_stack, SSIF_THREAD_STACK_SIZE);
	uint8_t task_name[SSIF_TASK_NAME_LEN];
	struct k_thread task_thread;
	struct k_mutex rsp_buff_mutex;
	struct k_sem rsp_buff_sem;
	uint8_t rsp_buff[IPMI_MSG_MAX_LENGTH];
	uint16_t rsp_buf_len; // Length of collected data
	uint16_t remain_data_len; // Length of remain data
	ipmi_msg_cfg current_ipmi_msg;
	uint16_t cur_rd_blck; // for multi-read middle/end

	ssif_status_t cur_status;
	ssif_err_status_t err_status_lst[SSIF_ERR_RCD_SIZE]; // history error status
	uint8_t err_idx; //
	ssif_err_status_t err_status; // last error status
} ssif_dev;

struct ssif_wr_start {
	uint8_t len;
	uint8_t netfn; // netfn(6bit) + lun(2bit)
	uint8_t cmd;
	uint8_t data[SSIF_MAX_IPMI_DATA_SIZE - 2]; // -netfn -cmd
} __attribute__((packed));

struct ssif_wr_middle {
	uint8_t len;
	uint8_t data[SSIF_MAX_IPMI_DATA_SIZE];
} __attribute__((packed));

struct ssif_rd_single {
	uint8_t len;
	uint8_t netfn; // netfn(6bit) + lun(2bit)
	uint8_t cmd;
	uint8_t cmplt_code;
	uint8_t data[SSIF_MAX_IPMI_DATA_SIZE - 3]; // -netfn -cmd -cc
} __attribute__((packed));

struct ssif_rd_start {
	uint8_t len;
	uint16_t start_key; // should equal 0x0001
	uint8_t netfn; // netfn(6bit) + lun(2bit)
	uint8_t cmd;
	uint8_t cmplt_code;
	uint8_t data[SSIF_MAX_IPMI_DATA_SIZE - 5]; // -netfn -cmd -cc -key(2bytes)
} __attribute__((packed));

struct ssif_rd_middle {
	uint8_t len;
	uint8_t block;
	uint8_t data[0];
} __attribute__((packed));

void ssif_device_init(struct ssif_init_cfg *config, uint8_t size);
ssif_err_status_t ssif_get_error_status();
bool ssif_set_data(uint8_t channel, ipmi_msg_cfg *msg_cfg);
void ssif_error_record(uint8_t channel, ssif_err_status_t errcode);
ssif_dev *ssif_inst_get_by_bus(uint8_t bus);
void pal_ssif_alert_trigger(uint8_t status);
void pal_bios_post_complete();
bool get_ssif_ok();
void reset_ssif_ok();

#endif /* ENABLE_SSIF */

#endif /* SSIF_H */
