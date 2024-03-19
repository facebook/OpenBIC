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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <logging/log.h>
#include "libutil.h"
#include "plat_pldm_fw_update.h"
#include "plat_i2c.h"
#include "pldm.h"
#include "pldm_firmware_update.h"
#include "plat_pldm_sensor.h"
#include "power_status.h"
#include "mctp_ctrl.h"
#include "xdpe12284c.h"

LOG_MODULE_REGISTER(plat_fwupdate);

static bool plat_pldm_vr_i2c_info_get(int comp_identifier, uint8_t *bus, uint8_t *addr);
static uint8_t plat_pldm_pre_vr_update(void *fw_update_param);
static uint8_t plat_pldm_post_vr_update(void *fw_update_param);
static bool plat_get_vr_fw_version(void *info_p, uint8_t *buf, uint8_t *len);

enum FIRMWARE_COMPONENT {
	FF_COMPNT_BIC,
	FF_COMPNT_VR_PVDDQ_AB_ASIC,
	FF_COMPNT_VR_PVDDQ_CD_ASIC,
};

/* PLDM FW update table */
pldm_fw_update_info_t PLDMUPDATE_FW_CONFIG_TABLE[] = {
	{
		.enable = true,
		.comp_classification = COMP_CLASS_TYPE_DOWNSTREAM,
		.comp_identifier = FF_COMPNT_BIC,
		.comp_classification_index = 0x00,
		.pre_update_func = NULL,
		.update_func = pldm_bic_update,
		.pos_update_func = NULL,
		.inf = COMP_UPDATE_VIA_SPI,
		.activate_method = COMP_ACT_SELF,
		.self_act_func = pldm_bic_activate,
		.get_fw_version_fn = NULL,
		.self_apply_work_func = NULL,
		.comp_version_str = NULL,
	},
	{
		.enable = true,
		.comp_classification = COMP_CLASS_TYPE_DOWNSTREAM,
		.comp_identifier = FF_COMPNT_VR_PVDDQ_AB_ASIC,
		.comp_classification_index = 0x00,
		.pre_update_func = plat_pldm_pre_vr_update,
		.update_func = pldm_vr_update,
		.pos_update_func = plat_pldm_post_vr_update,
		.inf = COMP_UPDATE_VIA_I2C,
		.activate_method = COMP_ACT_AC_PWR_CYCLE,
		.self_act_func = NULL,
		.get_fw_version_fn = plat_get_vr_fw_version,
		.self_apply_work_func = NULL,
		.comp_version_str = NULL,
	},
	{
		.enable = true,
		.comp_classification = COMP_CLASS_TYPE_DOWNSTREAM,
		.comp_identifier = FF_COMPNT_VR_PVDDQ_CD_ASIC,
		.comp_classification_index = 0x00,
		.pre_update_func = plat_pldm_pre_vr_update,
		.update_func = pldm_vr_update,
		.pos_update_func = plat_pldm_post_vr_update,
		.inf = COMP_UPDATE_VIA_I2C,
		.activate_method = COMP_ACT_AC_PWR_CYCLE,
		.self_act_func = NULL,
		.get_fw_version_fn = plat_get_vr_fw_version,
		.self_apply_work_func = NULL,
		.comp_version_str = NULL,
	},
};

uint8_t plat_pldm_query_device_identifiers(const uint8_t *buf, uint16_t len, uint8_t *resp,
					   uint16_t *resp_len)
{
	CHECK_NULL_ARG_WITH_RETURN(buf, false);
	CHECK_NULL_ARG_WITH_RETURN(resp, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp_len, PLDM_ERROR);

	struct pldm_query_device_identifiers_resp *resp_p =
		(struct pldm_query_device_identifiers_resp *)resp;

	resp_p->completion_code = PLDM_SUCCESS;
	resp_p->descriptor_count = 0x03;

	uint8_t iana[PLDM_FWUP_IANA_ENTERPRISE_ID_LENGTH] = { 0x00, 0x00, 0xA0, 0x15 };

	// Set the device id for ff bic
	uint8_t deviceId[PLDM_PCI_DEVICE_ID_LENGTH] = { 0x00, 0x01 };

	uint8_t slotNumber = plat_get_eid() / 10;
	uint8_t slot[PLDM_ASCII_MODEL_NUMBER_SHORT_STRING_LENGTH] = { (char)(slotNumber + '0') };

	uint8_t total_size_of_iana_descriptor =
		sizeof(struct pldm_descriptor_tlv) + sizeof(iana) - 1;

	uint8_t total_size_of_device_id_descriptor =
		sizeof(struct pldm_descriptor_tlv) + sizeof(deviceId) - 1;

	uint8_t total_size_of_slot_descriptor =
		sizeof(struct pldm_descriptor_tlv) + sizeof(slot) - 1;

	if (sizeof(struct pldm_query_device_identifiers_resp) + total_size_of_iana_descriptor +
		    total_size_of_device_id_descriptor + total_size_of_slot_descriptor >
	    PLDM_MAX_DATA_SIZE) {
		LOG_ERR("QueryDeviceIdentifiers data length is over PLDM_MAX_DATA_SIZE define size %d",
			PLDM_MAX_DATA_SIZE);
		resp_p->completion_code = PLDM_ERROR;
		return PLDM_ERROR;
	}

	// Allocate data for tlv which including descriptors data
	struct pldm_descriptor_tlv *tlv_ptr = malloc(total_size_of_iana_descriptor);
	if (tlv_ptr == NULL) {
		LOG_ERR("Memory allocation failed!");
		return PLDM_ERROR;
	}

	tlv_ptr->descriptor_type = PLDM_FWUP_IANA_ENTERPRISE_ID;
	tlv_ptr->descriptor_length = PLDM_FWUP_IANA_ENTERPRISE_ID_LENGTH;
	memcpy(tlv_ptr->descriptor_data, iana, sizeof(iana));

	uint8_t *end_of_id_ptr =
		(uint8_t *)resp + sizeof(struct pldm_query_device_identifiers_resp);

	memcpy(end_of_id_ptr, tlv_ptr, total_size_of_iana_descriptor);
	free(tlv_ptr);

	tlv_ptr = malloc(total_size_of_device_id_descriptor);
	if (tlv_ptr == NULL) {
		LOG_ERR("Memory allocation failed!");
		return PLDM_ERROR;
	}

	tlv_ptr->descriptor_type = PLDM_PCI_DEVICE_ID;
	tlv_ptr->descriptor_length = PLDM_PCI_DEVICE_ID_LENGTH;
	memcpy(tlv_ptr->descriptor_data, deviceId, sizeof(deviceId));

	end_of_id_ptr += total_size_of_iana_descriptor;
	memcpy(end_of_id_ptr, tlv_ptr, total_size_of_device_id_descriptor);
	free(tlv_ptr);

	tlv_ptr = malloc(total_size_of_slot_descriptor);
	if (tlv_ptr == NULL) {
		LOG_ERR("Memory allocation failed!");
		return PLDM_ERROR;
	}

	tlv_ptr->descriptor_type = PLDM_ASCII_MODEL_NUMBER_SHORT_STRING;
	tlv_ptr->descriptor_length = PLDM_ASCII_MODEL_NUMBER_SHORT_STRING_LENGTH;
	memcpy(tlv_ptr->descriptor_data, slot, sizeof(slot));

	end_of_id_ptr += total_size_of_device_id_descriptor;
	memcpy(end_of_id_ptr, tlv_ptr, total_size_of_slot_descriptor);
	free(tlv_ptr);

	resp_p->device_identifiers_len = total_size_of_iana_descriptor +
					 total_size_of_device_id_descriptor +
					 total_size_of_slot_descriptor;

	*resp_len = sizeof(struct pldm_query_device_identifiers_resp) +
		    total_size_of_iana_descriptor + total_size_of_device_id_descriptor +
		    total_size_of_slot_descriptor;

	return PLDM_SUCCESS;
}

void load_pldmupdate_comp_config(void)
{
	if (comp_config) {
		LOG_WRN("PLDM update component table has already been load");
		return;
	}

	comp_config_count = ARRAY_SIZE(PLDMUPDATE_FW_CONFIG_TABLE);
	comp_config = malloc(sizeof(pldm_fw_update_info_t) * comp_config_count);
	if (!comp_config) {
		LOG_ERR("comp_config malloc failed");
		return;
	}

	memcpy(comp_config, PLDMUPDATE_FW_CONFIG_TABLE, sizeof(PLDMUPDATE_FW_CONFIG_TABLE));
}

static bool plat_pldm_vr_i2c_info_get(int comp_identifier, uint8_t *bus, uint8_t *addr)
{
	switch (comp_identifier) {
	case FF_COMPNT_VR_PVDDQ_AB_ASIC:
		*bus = (uint8_t)I2C_BUS9;
		*addr = (uint8_t)ADDR_VR_PVDDQ_AB;
		break;
	case FF_COMPNT_VR_PVDDQ_CD_ASIC:
		*bus = (uint8_t)I2C_BUS9;
		*addr = (uint8_t)ADDR_VR_PVDDQ_CD;
		break;
	default:
		LOG_ERR("Unknown component identifier for VR");
		return false;
	}
	return true;
}

static uint8_t plat_pldm_pre_vr_update(void *fw_update_param)
{
	CHECK_NULL_ARG_WITH_RETURN(fw_update_param, 1);

	pldm_fw_update_param_t *p = (pldm_fw_update_param_t *)fw_update_param;

	/* Stop sensor polling */
	set_vr_monitor_status(false);

	plat_pldm_vr_i2c_info_get(p->comp_id, &p->bus, &p->addr);

	return 0;
}

static uint8_t plat_pldm_post_vr_update(void *fw_update_param)
{
	ARG_UNUSED(fw_update_param);

	set_vr_monitor_status(true);

	return 0;
}

static bool plat_get_vr_fw_version(void *info_p, uint8_t *buf, uint8_t *len)
{
	CHECK_NULL_ARG_WITH_RETURN(info_p, false);
	CHECK_NULL_ARG_WITH_RETURN(buf, false);
	CHECK_NULL_ARG_WITH_RETURN(len, false);

	pldm_fw_update_info_t *p = (pldm_fw_update_info_t *)info_p;

	bool ret = false;
	uint8_t bus = 0;
	uint8_t addr = 0;
	uint32_t version;
	uint16_t remain = 0xFFFF;

	plat_pldm_vr_i2c_info_get(p->comp_identifier, &bus, &addr);

	set_vr_monitor_status(false);
	if (!xdpe12284c_get_checksum(bus, addr, (uint8_t *)&version)) {
		LOG_ERR("The VR XDPE12284 version reading failed");
		return ret;
	}

	if (!xdpe12284c_get_remaining_write(bus, addr, &remain)) {
		LOG_ERR("The VR XDPE12284 remaining reading failed");
		return ret;
	}

	set_vr_monitor_status(true);

	const char *remain_str_p = ", Remaining Write: ";
	uint8_t *buf_p = buf;
	const uint8_t *vr_name_p = INF_CRC_PREFIX;
	if (!vr_name_p) {
		LOG_ERR("The pointer of VR string name is NULL");
		return ret;
	}
	*len = 0;

	if (PLDM_MAX_DATA_SIZE < (strlen(vr_name_p) + strlen(remain_str_p) + 10)) {
		LOG_ERR("vr version string wiil be too long to operate, failed");
		return ret;
	}

	memcpy(buf_p, vr_name_p, strlen(vr_name_p));
	buf_p += strlen(vr_name_p);
	*len += bin2hex((uint8_t *)&version, 4, buf_p, 8) + strlen(vr_name_p);
	buf_p += 8;

	if (remain != 0xFFFF) {
		memcpy(buf_p, remain_str_p, strlen(remain_str_p));
		buf_p += strlen(remain_str_p);
		remain = (uint8_t)((remain % 10) | (remain / 10 << 4));
		*len += bin2hex((uint8_t *)&remain, 1, buf_p, 2) + strlen(remain_str_p);
		buf_p += 2;
	}

	ret = true;

	return ret;
}
