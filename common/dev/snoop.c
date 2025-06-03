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

#ifdef CONFIG_SNOOP_ASPEED

#include <zephyr.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <device.h>
#include <drivers/misc/aspeed/snoop_aspeed.h>
#include "snoop.h"
#include "libutil.h"
#include "ipmi.h"
#include "pldm.h"
#include "power_status.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(dev_snoop);

const struct device *snoop_dev;
struct snoop_context {
	intptr_t reserved;
	uint32_t byte;
};
struct k_fifo snoop_fifo[SNOOP_CHANNEL_NUM];
static uint8_t *snoop_read_buffer;
int snoop_read_num = 0;
int send_postcode_start_position = 0;
static bool proc_postcode_ok = false;
static bool snoop_rx_registered = false;

K_THREAD_STACK_DEFINE(snoop_thread, SNOOP_STACK_SIZE);
struct k_thread snoop_thread_handler;
k_tid_t snoop_tid;

K_THREAD_STACK_DEFINE(send_postcode_thread, SENDPOSTCODE_STACK_SIZE);
struct k_thread send_postcode_thread_handler;
k_tid_t send_postcode_tid;

struct k_mutex snoop_mutex;

int snoop_fifo_read(uint32_t ch, uint8_t *out, bool blocking)
{
	struct snoop_context *node;

	if (ch >= SNOOP_CHANNEL_NUM)
		return -EINVAL;

	node = k_fifo_get(&snoop_fifo[ch], (blocking) ? K_FOREVER : K_NO_WAIT);
	if (!node)
		return -ENODATA;

	*out = (uint8_t)node->byte;

	k_free(node);

	return 0;
}

void snoop_rx_callback(const uint8_t *snoop0, const uint8_t *snoop1)
{
	if (snoop0) {
		struct snoop_context *node;
		node = k_malloc(sizeof(struct snoop_context));
		if (node) {
			node->byte = *snoop0;
			k_fifo_put(&snoop_fifo[0], node);
		} else {
			LOG_ERR("failed to allocate node0, drop data\n");
		}
	}
}

void snoop_init()
{
	snoop_dev = device_get_binding(DT_LABEL(DT_NODELABEL(snoop)));
	if (!snoop_dev) {
		LOG_ERR("No snoop device found");
		return;
	}

	for (int i = 0; i < SNOOP_CHANNEL_NUM; ++i) {
		k_fifo_init(&snoop_fifo[i]);
	}

	if (!snoop_rx_registered) {
		int rc;
		rc = snoop_aspeed_register_rx_callback(snoop_dev, snoop_rx_callback);
		if (rc) {
			LOG_ERR("snoop register rx callback fail");
			return;
		}
		snoop_rx_registered = true;
	}

	return;
}

void copy_snoop_read_buffer(uint8_t offset, int size_num, uint8_t *buffer, uint8_t copy_mode)
{
	if (buffer == NULL) {
		return;
	}

	if (size_num > SNOOP_MAX_LEN) {
		LOG_ERR("copy snoop buffer size exceeded");
		return;
	}
	if (!k_mutex_lock(&snoop_mutex, K_MSEC(1000))) {
		if (copy_mode == COPY_ALL_POSTCODE) {
			memcpy(&buffer[0], &snoop_read_buffer[offset], size_num - offset);
			memcpy(&buffer[size_num - offset], &snoop_read_buffer[0], offset);
		} else {
			if (offset + size_num > SNOOP_MAX_LEN) {
				memcpy(&buffer[0], &snoop_read_buffer[offset],
				       SNOOP_MAX_LEN - offset);
				memcpy(&buffer[SNOOP_MAX_LEN - offset], &snoop_read_buffer[0],
				       size_num - (SNOOP_MAX_LEN - offset));
			} else {
				memcpy(&buffer[0], &snoop_read_buffer[offset], size_num);
			}
		}
	} else {
		LOG_ERR("copy snoop buffer lock fail");
	}
	if (k_mutex_unlock(&snoop_mutex)) {
		LOG_ERR("copy snoop buffer unlock fail");
	}
}

bool get_postcode_ok()
{
	return proc_postcode_ok;
}

void reset_postcode_ok()
{
	proc_postcode_ok = false;
}

void snoop_read()
{
	int rc;
	if (snoop_read_buffer == NULL) {
		snoop_read_buffer = malloc(sizeof(uint8_t) * SNOOP_MAX_LEN);
	}
	if (snoop_read_buffer == NULL) {
		LOG_ERR("snoop read buffer alloc fail");
		return;
	}

	uint8_t snoop_data;
	while (1) {
		rc = snoop_fifo_read(0, &snoop_data, true);
		if (rc == 0) {
			proc_postcode_ok = true;
			if (!k_mutex_lock(&snoop_mutex, K_MSEC(1000))) {
				snoop_read_buffer[snoop_read_num % SNOOP_MAX_LEN] = snoop_data;
				snoop_read_num++;
				if (k_mutex_unlock(&snoop_mutex)) {
					LOG_ERR("snoop read unlock fail");
				}
			} else {
				LOG_ERR("snoop read lock fail");
			}
		}
	}
}

void init_snoop_thread()
{
	snoop_init();
	snoop_read_num = 0;
	if (snoop_tid != NULL && strcmp(k_thread_state_str(snoop_tid), "dead") != 0) {
		return;
	}
	snoop_tid = k_thread_create(&snoop_thread_handler, snoop_thread,
				    K_THREAD_STACK_SIZEOF(snoop_thread), snoop_read, NULL, NULL,
				    NULL, CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&snoop_thread_handler, "snoop_thread");
}

void abort_snoop_thread()
{
	if (snoop_tid != NULL && strcmp(k_thread_state_str(snoop_tid), "dead") != 0) {
		k_thread_abort(snoop_tid);
	}
}

void send_post_code_to_BMC()
{
	int send_postcode_end_position;
	ipmi_msg *send_postcode_msg;
	ipmb_error status;

	while (1) {
		k_msleep(100); // send post code to BMC once 100 ms
		send_postcode_end_position = snoop_read_num;
		if (get_DC_status() == 0) {
			return;
		}
		if (send_postcode_start_position != send_postcode_end_position) {
			send_postcode_msg = (ipmi_msg *)malloc(sizeof(ipmi_msg));
			static uint8_t alloc_sendmsg_retry = 0;
			if (send_postcode_msg == NULL) {
				if (get_post_status()) {
					alloc_sendmsg_retry += 1;
					if (alloc_sendmsg_retry > 3) {
						LOG_ERR("post complete and send post code thread alloc fail three times continuously");
						return;
					} else {
						continue;
					}
				} else {
					LOG_ERR("send post code thread alloc fail");
					continue;
				}
			}

			alloc_sendmsg_retry = 0;

			if (send_postcode_end_position - send_postcode_start_position >
			    SNOOP_MAX_LEN) {
				send_postcode_end_position =
					send_postcode_start_position + SNOOP_MAX_LEN;
			}
			memset(send_postcode_msg, 0, sizeof(ipmi_msg));
			send_postcode_msg->InF_source = SELF;
			send_postcode_msg->InF_target = BMC_IPMB;
			send_postcode_msg->netfn = NETFN_OEM_1S_REQ;
			send_postcode_msg->cmd = CMD_OEM_1S_SEND_POST_CODE_TO_BMC;
			send_postcode_msg->data_len =
				send_postcode_end_position - send_postcode_start_position + 4;
			send_postcode_msg->data[0] = IANA_ID & 0xFF;
			send_postcode_msg->data[1] = (IANA_ID >> 8) & 0xFF;
			send_postcode_msg->data[2] = (IANA_ID >> 16) & 0xFF;
			send_postcode_msg->data[3] =
				send_postcode_end_position - send_postcode_start_position;
			copy_snoop_read_buffer(send_postcode_start_position % SNOOP_MAX_LEN,
					       send_postcode_msg->data[3],
					       &send_postcode_msg->data[4], COPY_SPECIFIC_POSTCODE);

			// Check BMC communication interface if use IPMB or not
			if (pal_is_interface_use_ipmb(IPMB_inf_index_map[BMC_IPMB])) {
				status = ipmb_read(
					send_postcode_msg,
					IPMB_inf_index_map[send_postcode_msg->InF_target]);
				SAFE_FREE(send_postcode_msg);
				if (status == IPMB_ERROR_FAILURE) {
					LOG_ERR("Fail to post msg to txqueue for send post code from %d to %d",
						send_postcode_start_position,
						send_postcode_end_position);
					continue;
				} else if (status == IPMB_ERROR_GET_MESSAGE_QUEUE) {
					LOG_ERR("No response from bmc for send post code");
					continue;
				}
			} else {
				send_postcode_msg->InF_target = PLDM;
				status = pldm_send_ipmi_request(send_postcode_msg);
				SAFE_FREE(send_postcode_msg);
			}
			send_postcode_start_position = send_postcode_end_position;
		} else {
			if (CPU_power_good() == false) {
				return;
			}
		}
	}
}

void init_send_postcode_thread()
{
	send_postcode_start_position = 0;
	if (send_postcode_tid != NULL &&
	    strcmp(k_thread_state_str(send_postcode_tid), "dead") != 0) {
		return;
	}
	send_postcode_tid =
		k_thread_create(&send_postcode_thread_handler, send_postcode_thread,
				K_THREAD_STACK_SIZEOF(send_postcode_thread), send_post_code_to_BMC,
				NULL, NULL, NULL, CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&send_postcode_thread_handler, "send_postcode_thread");
}

#endif
