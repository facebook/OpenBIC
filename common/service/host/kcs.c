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
#include "pldm.h"
#include "plat_def.h"
#include "libutil.h"
#include "ipmb.h"

#ifdef ENABLE_PLDM
#include "plat_mctp.h"
#include "pldm_smbios.h"
#include "pldm_monitor.h"
#endif

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

#ifdef ENABLE_PLDM

enum cmd_app_get_sys_info_params {
	LENGTH_INDEX = 0x05, // skip netfun, cmd code, paramter selctor, set selctor, encoding
	VERIONS_START_INDEX = 0x06,
};

static uint8_t init_text_string_value(const char *new_text_string, char **text_strings)
{
	const uint8_t NULL_BYTE = 1, EMPTY_TEXT_STRINGS_SIZE = 2;
	const uint8_t original_size = pldm_smbios_get_text_strings_size(*text_strings);
	const uint8_t new_text_string_len = strlen(new_text_string);
	uint8_t new_text_string_start_idx = original_size - 1;

	if (original_size == EMPTY_TEXT_STRINGS_SIZE) {
		new_text_string_start_idx = 0;
	}

	*text_strings = realloc(*text_strings, original_size + new_text_string_len + NULL_BYTE);
	if (!(*text_strings)) {
		LOG_ERR("%s:%s:%d: Failed to allocate memory.", __FILE__, __func__, __LINE__);
		return 0;
	}
	memcpy(((*text_strings) + new_text_string_start_idx), new_text_string, new_text_string_len);

	*((*text_strings) + new_text_string_start_idx + new_text_string_len) = '\0';
	*((*text_strings) + new_text_string_start_idx + new_text_string_len + 1) = '\0';
	return pldm_smbios_get_text_strings_count(*text_strings);
}

static int init_bios_information(smbios_bios_information *bios_info, char *bios_version)
{
	const uint8_t NOT_IMPLEMENTED = 0;

	bios_info->header.type = SMBIOS_BIOS_INFORMATION;
	bios_info->text_strings = malloc(sizeof(char) * 2);
	if (!bios_info->text_strings) {
		LOG_ERR("%s:%s:%d: Failed to allocate memory.", __FILE__, __func__, __LINE__);
		return -1;
	}
	bios_info->text_strings[0] = bios_info->text_strings[1] = '\0';
	/* DSP0134: 12h + number of BIOS Characteristics Extension Bytes.*/
	bios_info->header.length = 0x12;
	bios_info->header.handle = 0x0000;
	bios_info->vendor = init_text_string_value("N/A", &bios_info->text_strings);
	bios_info->bios_version = init_text_string_value(bios_version, &bios_info->text_strings);
	bios_info->bios_starting_address_segment = NOT_IMPLEMENTED;
	bios_info->bios_release_date = init_text_string_value("N/A", &bios_info->text_strings);
	bios_info->bios_rom_size = NOT_IMPLEMENTED;
	bios_info->bios_characteristics = NOT_IMPLEMENTED;
	bios_info->bios_characteristics_extension_bytes = NOT_IMPLEMENTED;
	bios_info->system_bios_major_release = NOT_IMPLEMENTED;
	bios_info->system_bios_minor_release = NOT_IMPLEMENTED;
	bios_info->embedded_controller_firmware_major_release = NOT_IMPLEMENTED;
	bios_info->embedded_controller_firmware_minor_release = NOT_IMPLEMENTED;
	bios_info->extended_bios_rom_size = NOT_IMPLEMENTED;

	return 0;
}

static int update_bios_information(char *bios_version_start_ptr, uint8_t bios_version_len)
{
	int rc = -1;
	smbios_bios_information *bios_info = malloc(sizeof(smbios_bios_information));

	char *bios_version_str = malloc(sizeof(char) * bios_version_len + 1);
	if (!bios_version_str || !bios_info) {
		LOG_ERR("%s:%s:%d: Failed to allocate memory.", __FILE__, __func__, __LINE__);
		rc = -1;
		goto exit;
	}
	memcpy(bios_version_str, bios_version_start_ptr, bios_version_len);
	*(bios_version_str + bios_version_len) = '\0';

	rc = init_bios_information(bios_info, bios_version_str);
	if (rc < 0) {
		LOG_ERR("Failed to initialize bios information, error code=%d", rc);
		goto exit;
	}

	rc = pldm_smbios_set_bios_information(bios_info);
	if (rc < 0) {
		LOG_ERR("Failed to set smbios information, error code=%d", rc);
		goto exit;
	}

exit:
	SAFE_FREE(bios_info->text_strings);
	SAFE_FREE(bios_info);
	SAFE_FREE(bios_version_str);

	return rc;
}

#endif

static void kcs_read_task(void *arvg0, void *arvg1, void *arvg2)
{
	int rc = 0;
	uint8_t ibuf[KCS_BUFF_SIZE];
#ifndef ENABLE_PLDM
	ipmi_msg bridge_msg;
	ipmb_error status;
#endif
	ipmi_msg_cfg current_msg;
	struct kcs_request *req;

	ARG_UNUSED(arvg1);
	ARG_UNUSED(arvg2);
	kcs_dev *kcs_inst = (kcs_dev *)arvg0;
	if (!kcs_inst) {
		LOG_ERR("Failed to get the kcs instance");
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

		if (pal_request_msg_to_BIC_from_HOST(
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
			notify_ipmi_client(&current_msg);
		} else { // default command for BMC, should add BIC firmware update, BMC reset, real time sensor read in future
			if (pal_immediate_respond_from_HOST(req->netfn, req->cmd)) {
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
#ifdef ENABLE_PLDM
#ifndef ENABLE_OEM_PLDM
			//Send SEL entry to BMC via PLDM command with OEM event class 0xFB
			if ((req->netfn == NETFN_STORAGE_REQ) &&
			    (req->cmd == CMD_STORAGE_ADD_SEL)) {
				// Send SEL to BMC via PLDM over MCTP
				mctp_ext_params ext_params = {0};
				ext_params.type = MCTP_MEDIUM_TYPE_SMBUS;
				ext_params.smbus_ext_params.addr = I2C_ADDR_BMC;
				ext_params.ep = MCTP_EID_BMC;
				uint8_t pldm_event_length = rc - 5; // exclude netfn, cmd, record_id, record_type
				pldm_platform_event_message_req(
					find_mctp_by_bus(I2C_BUS_BMC), ext_params, 0xFB, &ibuf[5], pldm_event_length);
			}
#endif
#endif
			if ((req->netfn == NETFN_APP_REQ) &&
			    (req->cmd == CMD_APP_SET_SYS_INFO_PARAMS) &&
			    (req->data[0] == CMD_SYS_INFO_FW_VERSION)) {
				int ret = pal_record_bios_fw_version(ibuf, rc);
				if (ret == -1) {
					LOG_ERR("Record bios fw version fail");
				}
#ifdef ENABLE_PLDM
				char *bios_version = ibuf + VERIONS_START_INDEX;
				uint8_t length = ibuf[LENGTH_INDEX];
				ret = update_bios_information(bios_version, length);
				if (ret < 0) {
					LOG_ERR("Failed to update bios information, rc = %d", ret);
				}
#endif
			}
			if ((req->netfn == NETFN_OEM_Q_REQ) &&
			    (req->cmd == CMD_OEM_Q_SET_DIMM_INFO) &&
			    (req->data[4] == CMD_DIMM_LOCATION)) {
				int ret = pal_set_dimm_presence_status(ibuf);
				if (!ret) {
					LOG_ERR("Set dimm presence status fail");
				}
			}
#ifndef ENABLE_PLDM
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

			// Check BMC communication interface if use IPMB or not
			if (!pal_is_interface_use_ipmb(IPMB_inf_index_map[BMC_IPMB])) {
				int ret = 0;
				// Send request to MCTP/PLDM thread to ask BMC
				bridge_msg.InF_target = PLDM;
				ret = pldm_send_ipmi_request(&bridge_msg);
				if (ret < 0) {
					LOG_ERR("kcs_read_task send to BMC fail");
				}

				uint8_t *kcs_buff;
				kcs_buff = malloc(3 + bridge_msg.data_len);
				if (kcs_buff == NULL) {
					LOG_ERR("Memory allocation failed");
					continue;
				}

				// Write MCTP/PLDM response to KCS
				kcs_buff[0] = (bridge_msg.netfn | BIT(0)) << 2;
				kcs_buff[1] = bridge_msg.cmd;
				kcs_buff[2] = bridge_msg.completion_code;
				memcpy(&kcs_buff[3], &bridge_msg.data, bridge_msg.data_len);

				if (!pal_immediate_respond_from_HOST(req->netfn, req->cmd)) {
					kcs_write(kcs_inst->index, kcs_buff,
						  3 + bridge_msg.data_len);
				}

				SAFE_FREE(kcs_buff);
			} else {
				status = ipmb_send_request(&bridge_msg,
							   IPMB_inf_index_map[BMC_IPMB]);
				if (status != IPMB_ERROR_SUCCESS) {
					LOG_ERR("kcs_read_task send to BMC fail status: 0x%x",
						status);
				}
			}
#endif
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
		LOG_ERR("KCS config size is too large.");
		SAFE_FREE(kcs);
		return;
	}

	for (i = 0; i < size; i++) {
		kcs[i].dev = device_get_binding(config[i]);
		if (!kcs[i].dev) {
			LOG_ERR("Failed to find kcs device");
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
