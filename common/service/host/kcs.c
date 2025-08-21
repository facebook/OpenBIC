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
#include "util_sys.h"

#ifdef ENABLE_PLDM
#include "plat_mctp.h"
#include "pldm_smbios.h"
#include "pldm_monitor.h"
#endif

LOG_MODULE_REGISTER(kcs);

kcs_dev *kcs;
static bool proc_kcs_ok = false;

int kcs_write(uint8_t index, uint8_t *buf, uint32_t buf_sz)
{
	int rc;

	rc = kcs_aspeed_write(kcs[index].dev, buf, buf_sz);
	if (rc < 0) {
		LOG_ERR("Failed to write KCS data, rc = %d", rc);
		return rc;
	}

	return 0;
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
#ifndef ENABLE_OEM_PLDM

K_THREAD_STACK_DEFINE(kcs_stack_area, KCS_WORKER_STACK_SIZE);
static struct k_work_q kcs_work_q;
static struct k_mutex mutex_use_count;
static uint8_t work_count = 0;

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

	char *temp_ptr = realloc(*text_strings, original_size + new_text_string_len + NULL_BYTE);
	if (!(temp_ptr)) {
		LOG_ERR("%s:%s:%d: Failed to allocate memory.", __FILE__, __func__, __LINE__);
		return 0;
	}

	*text_strings = temp_ptr;

	memcpy(((*text_strings) + new_text_string_start_idx), new_text_string, new_text_string_len);

	*((*text_strings) + new_text_string_start_idx + new_text_string_len) = '\0';
	*((*text_strings) + new_text_string_start_idx + new_text_string_len + 1) = '\0';
	return pldm_smbios_get_text_strings_count(*text_strings);
}

static int init_bios_information(smbios_bios_information *bios_info, const char *bios_version)
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

static int update_bios_information(const char *bios_version_start_ptr, uint8_t bios_version_len)
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

static int send_bios_version_to_bmc(const char *bios_version, uint8_t bios_version_len)
{
	pldm_msg msg = { 0 };
	uint8_t bmc_bus = I2C_BUS_BMC, bmc_interface = BMC_INTERFACE_I2C;

	bmc_interface = pal_get_bmc_interface();
	if (bmc_interface == BMC_INTERFACE_I3C) {
		bmc_bus = I3C_BUS_BMC;
		msg.ext_params.type = MCTP_MEDIUM_TYPE_TARGET_I3C;
		msg.ext_params.i3c_ext_params.addr = I3C_STATIC_ADDR_BMC;
		msg.ext_params.ep = MCTP_EID_BMC;
	} else {
		bmc_bus = I2C_BUS_BMC;
		msg.ext_params.type = MCTP_MEDIUM_TYPE_SMBUS;
		msg.ext_params.smbus_ext_params.addr = I2C_ADDR_BMC;
		msg.ext_params.ep = MCTP_EID_BMC;
	}

	msg.hdr.pldm_type = PLDM_TYPE_OEM;
	msg.hdr.cmd = PLDM_OEM_WRITE_FILE_IO;
	msg.hdr.rq = 1;

	struct pldm_oem_write_file_io_req *ptr = (struct pldm_oem_write_file_io_req *)malloc(
		sizeof(struct pldm_oem_write_file_io_req) + (sizeof(uint8_t) * bios_version_len));

	if (!ptr) {
		LOG_ERR("Failed to allocate memory.");
		return 1;
	}

	ptr->cmd_code = BIOS_VERSION;
	ptr->data_length = bios_version_len;
	memcpy(ptr->messages, bios_version, bios_version_len);

	msg.buf = (uint8_t *)ptr;
	msg.len = sizeof(struct pldm_oem_write_file_io_req) + bios_version_len;

	uint8_t resp_len = sizeof(struct pldm_oem_write_file_io_resp);
	uint8_t rbuf[resp_len];

	if (!mctp_pldm_read(find_mctp_by_bus(bmc_bus), &msg, rbuf, resp_len)) {
		SAFE_FREE(ptr);
		LOG_ERR("mctp_pldm_read fail");
		return 2;
	}

	struct pldm_oem_write_file_io_resp *resp = (struct pldm_oem_write_file_io_resp *)rbuf;
	if (resp->completion_code != PLDM_SUCCESS) {
		SAFE_FREE(ptr);
		LOG_ERR("Check reponse completion code fail %x", resp->completion_code);
		return resp->completion_code;
	}

	SAFE_FREE(ptr);
	return 0;
}

static int send_crashdump_to_bmc(const uint8_t *data, uint8_t data_length)
{
	pldm_msg msg = { 0 };
	uint8_t bmc_bus = I2C_BUS_BMC, bmc_interface = BMC_INTERFACE_I2C;

	bmc_interface = pal_get_bmc_interface();
	if (bmc_interface == BMC_INTERFACE_I3C) {
		bmc_bus = I3C_BUS_BMC;
		msg.ext_params.type = MCTP_MEDIUM_TYPE_TARGET_I3C;
		msg.ext_params.i3c_ext_params.addr = I3C_STATIC_ADDR_BMC;
		msg.ext_params.ep = MCTP_EID_BMC;
	} else {
		bmc_bus = I2C_BUS_BMC;
		msg.ext_params.type = MCTP_MEDIUM_TYPE_SMBUS;
		msg.ext_params.smbus_ext_params.addr = I2C_ADDR_BMC;
		msg.ext_params.ep = MCTP_EID_BMC;
	}

	msg.hdr.pldm_type = PLDM_TYPE_OEM;
	msg.hdr.cmd = PLDM_OEM_WRITE_FILE_IO;
	msg.hdr.rq = 1;

	struct pldm_oem_write_file_io_req *ptr = (struct pldm_oem_write_file_io_req *)malloc(
		sizeof(struct pldm_oem_write_file_io_req) + (sizeof(uint8_t) * data_length));

	if (!ptr) {
		LOG_ERR("Failed to allocate memory.");
		return 1;
	}

	ptr->cmd_code = CRASH_DUMP;
	ptr->data_length = data_length;
	memcpy(ptr->messages, data, data_length);

	msg.buf = (uint8_t *)ptr;
	msg.len = sizeof(struct pldm_oem_write_file_io_req) + data_length;

	uint8_t resp_len = sizeof(struct pldm_oem_write_file_io_resp);
	uint8_t rbuf[resp_len];

	if (!mctp_pldm_read(find_mctp_by_bus(bmc_bus), &msg, rbuf, resp_len)) {
		SAFE_FREE(ptr);
		LOG_ERR("mctp_pldm_read fail");
		return 2;
	}

	struct pldm_oem_write_file_io_resp *resp = (struct pldm_oem_write_file_io_resp *)rbuf;
	if (resp->completion_code != PLDM_SUCCESS) {
		SAFE_FREE(ptr);
		LOG_ERR("Check reponse completion code fail %x", resp->completion_code);
		return resp->completion_code;
	}

	SAFE_FREE(ptr);
	return 0;
}

static void kcs_add_sel_handler(struct k_work *kcs_pldm_work)
{
	struct kcs_work_info *kcs_work = CONTAINER_OF(kcs_pldm_work, struct kcs_work_info, work);
	// Send SEL to BMC via PLDM over MCTP
	mctp_ext_params ext_params = { 0 };
	uint8_t pldm_event_length = kcs_work->data_length - 4; // exclude netfn, cmd, record_id
	uint8_t bmc_bus = I2C_BUS_BMC, bmc_interface = BMC_INTERFACE_I2C;

	bmc_interface = pal_get_bmc_interface();
	if (bmc_interface == BMC_INTERFACE_I3C) {
		bmc_bus = I3C_BUS_BMC;
		ext_params.type = MCTP_MEDIUM_TYPE_TARGET_I3C;
		ext_params.i3c_ext_params.addr = I3C_STATIC_ADDR_BMC;
		ext_params.ep = MCTP_EID_BMC;
	} else {
		bmc_bus = I2C_BUS_BMC;
		ext_params.type = MCTP_MEDIUM_TYPE_SMBUS;
		ext_params.smbus_ext_params.addr = I2C_ADDR_BMC;
		ext_params.ep = MCTP_EID_BMC;
	}
	pldm_platform_event_message_req(find_mctp_by_bus(bmc_bus),
					ext_params, 0xFB, &kcs_work->ibuf[4],
					pldm_event_length);

	k_mutex_lock(&mutex_use_count, K_MSEC(1000));
	work_count--;
	k_mutex_unlock(&mutex_use_count);
	SAFE_FREE(kcs_work);
}

static void kcs_crash_dump_handler(struct k_work *kcs_pldm_work)
{
	struct kcs_work_info *kcs_work = CONTAINER_OF(kcs_pldm_work, struct kcs_work_info, work);
	LOG_HEXDUMP_DBG(&kcs_work->ibuf[0], kcs_work->data_length, "host KCS read dump data:");
	int ret = send_crashdump_to_bmc(&kcs_work->ibuf[2],
		 (kcs_work->data_length - 2)); // exclude netfn, cmd
	if (ret) {
		LOG_ERR("Failed to send crashdump data to BMC, rc = %d", ret);
	}

	k_mutex_lock(&mutex_use_count, K_MSEC(1000));
	work_count--;
	k_mutex_unlock(&mutex_use_count);
	SAFE_FREE(kcs_work);
}

static void kcs_oem_post_start_end_handler(struct k_work *kcs_pldm_work)
{
	struct kcs_work_info *kcs_work = CONTAINER_OF(kcs_pldm_work, struct kcs_work_info, work);

	struct pldm_addsel_data msg = { 0 };
	// ibuf[0] is netfn, ibuf[1] is cmd
	if (kcs_work->ibuf[1] == CMD_OEM_POST_START) {
		msg.event_type = POST_STARTED;
	} else {
		msg.event_type = POST_ENDED;
	}
	msg.assert_type = EVENT_ASSERTED;
	if (send_event_log_to_bmc(msg) != PLDM_SUCCESS) {
		LOG_ERR("Failed to assert POST log");
	};

	k_mutex_lock(&mutex_use_count, K_MSEC(1000));
	work_count--;
	k_mutex_unlock(&mutex_use_count);
	SAFE_FREE(kcs_work);
}

static void kcs_bios_fw_version_handler(struct k_work *kcs_pldm_work)
{
	struct kcs_work_info *kcs_work = CONTAINER_OF(kcs_pldm_work, struct kcs_work_info, work);

	const char *bios_version = kcs_work->ibuf + VERIONS_START_INDEX;
	uint8_t length = kcs_work->ibuf[LENGTH_INDEX];
	int ret = update_bios_information(bios_version, length);
	if (ret < 0) {
		LOG_ERR("Failed to update bios information, rc = %d", ret);
	}

	ret = send_bios_version_to_bmc(bios_version, length);
	if (ret < 0) {
		LOG_ERR("Failed to send bios version to bmc, rc = %d", ret);
	}

	k_mutex_lock(&mutex_use_count, K_MSEC(1000));
	work_count--;
	k_mutex_unlock(&mutex_use_count);
	SAFE_FREE(kcs_work);
}

static int schedule_kcs_work_handle(k_work_handler_t handler,
									uint8_t *kcs_buff,
									kcs_dev *kcs_inst,
									uint8_t *buf,
									int data_length)
{
	// BIC return C3 to BIOS when kcs work queue is full or schedule work fail
	// do next loop and waiting for BIOS retry
	struct kcs_work_info *kcs_work = malloc(sizeof(struct kcs_work_info));
	if (kcs_work == NULL) {
		LOG_ERR("Failed to malloc for kcs_work");
		SAFE_FREE(kcs_buff);
		return -1;
	}
	kcs_work->data_length = data_length;
	memcpy(kcs_work->ibuf, &buf[0], kcs_work->data_length); // include netfn, cmd, record_id

	if (work_count >= MAX_KCS_WORK_COUNT) {
		LOG_ERR("kcs_work queue is full, work count = %d", work_count);
		kcs_buff[2] = CC_TIMEOUT;
		kcs_write(kcs_inst->index, kcs_buff, 3);
		SAFE_FREE(kcs_work);
		SAFE_FREE(kcs_buff);
		return -1;
	}
	// POST Start/End send event log to BMC only if kcs_write was successful
	if (((kcs_work->ibuf[0] == NETFN_OEM_REQ) && (kcs_work->ibuf[1] == CMD_OEM_POST_START)) ||
		((kcs_work->ibuf[0] == NETFN_OEM_REQ) && (kcs_work->ibuf[1] == CMD_OEM_POST_END))){
		if (kcs_write(kcs_inst->index, kcs_buff, 3) != 0) {
			SAFE_FREE(kcs_work);
			SAFE_FREE(kcs_buff);
			return -1;
		}
	}
	// put work in kcs work queue
	k_work_init_delayable(&kcs_work->work.delay_work, handler);
	if (k_work_schedule_for_queue(&kcs_work_q, &kcs_work->work.delay_work, K_NO_WAIT) < 1) {
		LOG_ERR("Failed to schedule work, netfn = %x, cmd = %x", kcs_buff[0], kcs_buff[1]);
		kcs_buff[2] = CC_TIMEOUT;
		kcs_write(kcs_inst->index, kcs_buff, 3);
		SAFE_FREE(kcs_work);
		SAFE_FREE(kcs_buff);
		return -1;
	}
	k_mutex_lock(&mutex_use_count, K_MSEC(1000));
	work_count++;
	k_mutex_unlock(&mutex_use_count);
	return 0;
}

#endif
#endif

static void kcs_read_task(void *arvg0, void *arvg1, void *arvg2)
{
	int rc = 0;
	uint8_t ibuf[KCS_BUFF_SIZE];
#if (!defined(ENABLE_PLDM) || defined(ENABLE_OEM_PLDM))
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
			uint8_t *kcs_buff;
			kcs_buff = malloc(KCS_BUFF_SIZE * sizeof(uint8_t));
			if (kcs_buff == NULL) {
				LOG_ERR("Failed to malloc for kcs_buff");
				continue;
			}
			kcs_buff[0] = (req->netfn | BIT(0)) << 2;
			kcs_buff[1] = req->cmd;
			kcs_buff[2] = CC_SUCCESS;
			if (pal_immediate_respond_from_HOST(req->netfn, req->cmd)) {
				if (((req->netfn == NETFN_STORAGE_REQ) &&
					 (req->cmd == CMD_STORAGE_ADD_SEL))) {
					if (rc != ADD_SEL_EVENT_DATA_MAX_LEN) { // do not write kcs when data length error
						LOG_ERR("ADD SEL event data length error, rc = %d", rc);
						SAFE_FREE(kcs_buff);
						continue;
					}
					kcs_buff[3] = 0x00;
					kcs_buff[4] = 0x00;
#ifdef ENABLE_PLDM
#ifndef ENABLE_OEM_PLDM
					if (schedule_kcs_work_handle(kcs_add_sel_handler, kcs_buff, 
						kcs_inst, ibuf, rc) != 0) {
						continue;
					}
#endif
#endif
					kcs_write(kcs_inst->index, kcs_buff, 5);
				} else {
					kcs_write(kcs_inst->index, kcs_buff, 3);
				}
			}
#ifdef ENABLE_PLDM
#ifndef ENABLE_OEM_PLDM
			// IPMI OEM command for crashdump, send crashdump data to BMC
			if ((req->netfn == NETFN_OEM_REQ) && (req->cmd == CMD_OEM_CRASH_DUMP)) {
				if (schedule_kcs_work_handle(kcs_crash_dump_handler, kcs_buff, 
					kcs_inst, ibuf, rc) != 0) {
					continue;
				}
				if (ibuf[6] == 0x80 && ibuf[10] == 0x01) {
					kcs_buff[3] = 0x02; // state wait_data(0x02)
					kcs_write(kcs_inst->index, kcs_buff, 4);
				} else {
					kcs_write(kcs_inst->index, kcs_buff, 3);
				}
			}
			// IPMI OEM command for POST Start/End. Send SEL to BMC
			if (((req->netfn == NETFN_OEM_REQ) && (req->cmd == CMD_OEM_POST_START)) ||
			    ((req->netfn == NETFN_OEM_REQ) && (req->cmd == CMD_OEM_POST_END))) {
				if (schedule_kcs_work_handle(kcs_oem_post_start_end_handler, kcs_buff, 
					kcs_inst, ibuf, rc) != 0) {
					continue;
				}
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
#ifndef ENABLE_OEM_PLDM
				if (schedule_kcs_work_handle(kcs_bios_fw_version_handler, kcs_buff, 
					kcs_inst, ibuf, rc) != 0) {
					continue;
				} else {
					kcs_write(kcs_inst->index, kcs_buff, 3);
				}
#endif
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
			SAFE_FREE(kcs_buff); // SAFE FREE kcs_buff
#if (!defined(ENABLE_PLDM) || defined(ENABLE_OEM_PLDM))
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
#ifdef ENABLE_PLDM
#ifndef ENABLE_OEM_PLDM
		k_work_queue_start(&kcs_work_q, kcs_stack_area,
			   K_THREAD_STACK_SIZEOF(kcs_stack_area), CONFIG_MAIN_THREAD_PRIORITY, NULL);
		k_thread_name_set(&kcs_work_q.thread, "kcs_worker");
		k_mutex_init(&mutex_use_count);
#endif
#endif

	return;
}

#endif
