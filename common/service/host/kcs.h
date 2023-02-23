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

#ifndef KCS_H
#define KCS_H

#ifdef CONFIG_IPMI_KCS_ASPEED

#include <stdbool.h>
#include <stdint.h>
#include <zephyr.h>

#define KCS_POLL_STACK_SIZE 4096
#define KCS_POLLING_INTERVAL 100
#define KCS_BUFF_SIZE 256
#define KCS_MAX_CHANNEL_NUM 0x0F

#define CMD_SYS_INFO_FW_VERSION 0x01
#define KCS_TASK_NAME_LEN 32

typedef struct _kcs_dev {
	const struct device *dev;
	uint8_t index;
	k_tid_t task_tid;
	K_KERNEL_STACK_MEMBER(task_stack, KCS_POLL_STACK_SIZE);
	uint8_t task_name[KCS_TASK_NAME_LEN];
	struct k_thread task_thread;
} kcs_dev;

struct kcs_request {
	uint8_t netfn;
	uint8_t cmd;
	uint8_t data[0];
};

struct kcs_response {
	uint8_t netfn;
	uint8_t cmd;
	uint8_t cmplt_code;
	uint8_t data[0];
};

void kcs_device_init(char **config, uint8_t size);
void kcs_write(uint8_t index, uint8_t *buf, uint32_t buf_sz);
bool get_kcs_ok();
void reset_kcs_ok();

#endif

#endif
