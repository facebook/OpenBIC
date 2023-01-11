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

#ifdef CONFIG_IPMI_KCS_ASPEED

#include <zephyr.h>
#include <string.h>
#include <stdio.h>
#include <device.h>
#include <drivers/ipmi/kcs_aspeed.h>
#include <stdlib.h>
#include <logging/log.h>
#include "ipmi.h"
#include "kcs.h"
#include "plat_def.h"
#include "libutil.h"

LOG_MODULE_REGISTER(kcs);

kcs_dev *kcs;
static bool proc_kcs_ok = false;

void kcs_write(uint8_t index, uint8_t *buf, uint32_t buf_sz)
{
	int rc;

	rc = kcs_aspeed_write(kcs[index].dev, buf, buf_sz);
	if (rc < 0) {
		LOG_ERR("Failed to write KCS data, rc = %d", rc);
	}
}

bool get_kcs_ok()
{
	return proc_kcs_ok;
}

void reset_kcs_ok()
{
	proc_kcs_ok = false;
}

static void kcs_read_task(void *arvg0, void *arvg1, void *arvg2)
{
	int rc = 0;
	uint8_t ibuf[KCS_BUFF_SIZE];
	ipmi_msg bridge_msg;
	ipmi_msg_cfg current_msg;
	ipmb_error status;

	struct kcs_request *req;

	ARG_UNUSED(arvg1);
	ARG_UNUSED(arvg2);
	kcs_dev *kcs_inst = (kcs_dev *)arvg0;
	if (!kcs_inst) {
		LOG_ERR("failed to get the kcs instance\n");
		return;
	}

	while (1) {
		k_msleep(KCS_POLLING_INTERVAL);

		rc = kcs_aspeed_read(kcs_inst->dev, ibuf, sizeof(ibuf));
		if (rc < 0) {
			if (rc != -ENODATA)
				LOG_ERR("Failed to read KCS data, rc = %d", rc);
			continue;
		}

		LOG_HEXDUMP_DBG(&ibuf[0], rc, "host KCS read dump data:");

		proc_kcs_ok = true;
		req = (struct kcs_request *)ibuf;
		req->netfn = req->netfn >> 2;

		if (pal_request_msg_to_BIC_from_KCS(
			    req->netfn, req->cmd)) { // In-band update command, not bridging to bmc
			current_msg.buffer.InF_source = HOST_KCS_1 + kcs_inst->index;
			current_msg.buffer.netfn = req->netfn;
			current_msg.buffer.cmd = req->cmd;
			current_msg.buffer.data_len = rc - 2; // exclude netfn, cmd
			if (current_msg.buffer.data_len != 0) {
				memcpy(current_msg.buffer.data, req->data,
				       current_msg.buffer.data_len);
			}

			LOG_DBG("KCS to ipmi netfn 0x%x, cmd 0x%x, length %d",
				current_msg.buffer.netfn, current_msg.buffer.cmd,
				current_msg.buffer.data_len);

			while (k_msgq_put(&ipmi_msgq, &current_msg, K_NO_WAIT) != 0) {
				k_msgq_purge(&ipmi_msgq);
				LOG_WRN("KCS retrying put ipmi msgq");
			}
		} else { // default command for BMC, should add BIC firmware update, BMC reset, real time sensor read in future
			if (pal_immediate_respond_from_KCS(req->netfn, req->cmd)) {
				do { // break if malloc fail.
					uint8_t *kcs_buff;
					kcs_buff = malloc(KCS_BUFF_SIZE * sizeof(uint8_t));
					if (kcs_buff == NULL) {
						LOG_ERR("Failed to malloc for kcs_buff");
						break;
					}
					kcs_buff[0] = (req->netfn | BIT(0)) << 2;
					kcs_buff[1] = req->cmd;
					kcs_buff[2] = CC_SUCCESS;

					if (((req->netfn == NETFN_STORAGE_REQ) &&
					     (req->cmd == CMD_STORAGE_ADD_SEL))) {
						kcs_buff[3] = 0x00;
						kcs_buff[4] = 0x00;
						kcs_write(kcs_inst->index, kcs_buff, 5);
					} else {
						kcs_write(kcs_inst->index, kcs_buff, 3);
					}
					SAFE_FREE(kcs_buff);
				} while (0);
			}

			if ((req->netfn == NETFN_APP_REQ) &&
			    (req->cmd == CMD_APP_SET_SYS_INFO_PARAMS) &&
			    (req->data[0] == CMD_SYS_INFO_FW_VERSION)) {
				int ret = pal_record_bios_fw_version(ibuf, rc);
				if (ret == -1) {
					LOG_ERR("Record bios fw version fail");
				}
			}
			bridge_msg.data_len = rc - 2; // exclude netfn, cmd
			bridge_msg.seq_source = 0xff; // No seq for KCS
			bridge_msg.InF_source = HOST_KCS_1 + kcs_inst->index;
			bridge_msg.InF_target =
				BMC_IPMB; // default bypassing IPMI standard command to BMC
			bridge_msg.netfn = req->netfn;
			bridge_msg.cmd = req->cmd;
			if (bridge_msg.data_len != 0) {
				memcpy(&bridge_msg.data[0], &ibuf[2], bridge_msg.data_len);
			}

			status = ipmb_send_request(&bridge_msg, IPMB_inf_index_map[BMC_IPMB]);
			if (status != IPMB_ERROR_SUCCESS) {
				LOG_ERR("kcs_read_task send to BMC fail status: 0x%x", status);
			}
		}
	}
}

void kcs_device_init(char **config, uint8_t size)
{
	uint8_t i;
	kcs = (kcs_dev *)malloc(size * sizeof(*kcs));
	if (!kcs)
		return;
	memset(kcs, 0, size * sizeof(*kcs));

	/* IPMI channel target [50h-5Fh] are reserved for KCS channel.
	 * The max channel number KCS_MAX_CHANNEL_NUM is 0xF.
	 */
	if (size > KCS_MAX_CHANNEL_NUM) {
		LOG_ERR("the kcs config size is too large.\n");
		SAFE_FREE(kcs);
		return;
	}

	for (i = 0; i < size; i++) {
		kcs[i].dev = device_get_binding(config[i]);
		if (!kcs[i].dev) {
			LOG_ERR("failed to find kcs device\n");
			continue;
		}
		snprintf(kcs[i].task_name, sizeof(kcs[i].task_name), "%s_polling", config[i]);
		kcs[i].index = i;
		kcs[i].task_tid = k_thread_create(&kcs[i].task_thread, kcs[i].task_stack,
						  K_THREAD_STACK_SIZEOF(kcs[i].task_stack),
						  kcs_read_task, (void *)&kcs[i], NULL, NULL,
						  CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
		k_thread_name_set(kcs[i].task_tid, kcs[i].task_name);
	}
	return;
}

#endif
