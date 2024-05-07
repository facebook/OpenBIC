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
#include "pldm.h"
#include "pldm_firmware_update.h"
#include "mctp_ctrl.h"
#include "power_status.h"
#include "util_spi.h"
#include "plat_pldm_fw_update.h"
#include "plat_i2c.h"
#include "plat_gpio.h"
#include "plat_pldm_sensor.h"

#include "mp2971.h"
#include "pt5161l.h"
#include "raa229621.h"
#include "plat_class.h"
#include "plat_pldm_device_identifier.h"
#include "sensor.h"
#include "tps53689.h"

LOG_MODULE_REGISTER(plat_fwupdate);

static uint8_t plat_pldm_pre_vr_update(void *fw_update_param);
static uint8_t plat_pldm_post_vr_update(void *fw_update_param);
static bool plat_get_vr_fw_version(void *info_p, uint8_t *buf, uint8_t *len);
static uint8_t plat_pldm_pre_retimer_update(void *fw_update_param);
static bool plat_get_retimer_fw_version(void *info_p, uint8_t *buf, uint8_t *len);
static bool plat_get_bios_fw_version(void *info_p, uint8_t *buf, uint8_t *len);

enum FIRMWARE_COMPONENT {
	SD_COMPNT_BIC,
	SD_COMPNT_VR_PVDDCR_CPU1,
	SD_COMPNT_VR_PVDD11_S3,
	SD_COMPNT_VR_PVDDCR_CPU0,
	SD_COMPNT_X16_RETIMER,
	SD_COMPNT_X8_RETIMER,
	SD_COMPNT_BIOS,
};

uint8_t MCTP_SUPPORTED_MESSAGES_TYPES[] = {
	TYPE_MCTP_CONTROL,
	TYPE_PLDM,
};

enum RETIMER_ADDR {
	X16_RETIMER_ADDR = 0x20,
	X8_RETIMER_ADDR = 0x23,
};

enum VR_TYPE {
	VR_TYPE_MPS,
	VR_TYPE_RNS,
	VR_TYPE_IFX,
	VR_TYPE_TI,
	VR_TYPE_UNKNOWN = 0xFF,
};

/* PLDM FW update table */
pldm_fw_update_info_t PLDMUPDATE_FW_CONFIG_TABLE[] = {
	{
		.enable = true,
		.comp_classification = COMP_CLASS_TYPE_DOWNSTREAM,
		.comp_identifier = SD_COMPNT_BIC,
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
		.comp_identifier = SD_COMPNT_VR_PVDDCR_CPU1,
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
		.comp_identifier = SD_COMPNT_VR_PVDD11_S3,
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
		.comp_identifier = SD_COMPNT_VR_PVDDCR_CPU0,
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
		.comp_identifier = SD_COMPNT_X16_RETIMER,
		.comp_classification_index = 0x00,
		.pre_update_func = plat_pldm_pre_retimer_update,
		.update_func = pldm_retimer_update,
		.pos_update_func = NULL,
		.inf = COMP_UPDATE_VIA_I2C,
		.activate_method = COMP_ACT_SELF,
		.self_act_func = NULL,
		.get_fw_version_fn = plat_get_retimer_fw_version,
		.self_apply_work_func = NULL,
		.comp_version_str = NULL,
	},
	{
		.enable = true,
		.comp_classification = COMP_CLASS_TYPE_DOWNSTREAM,
		.comp_identifier = SD_COMPNT_X8_RETIMER,
		.comp_classification_index = 0x00,
		.pre_update_func = plat_pldm_pre_retimer_update,
		.update_func = pldm_retimer_update,
		.pos_update_func = NULL,
		.inf = COMP_UPDATE_VIA_I2C,
		.activate_method = COMP_ACT_SELF,
		.self_act_func = NULL,
		.get_fw_version_fn = plat_get_retimer_fw_version,
		.self_apply_work_func = NULL,
		.comp_version_str = NULL,
	},
	{
		.enable = true,
		.comp_classification = COMP_CLASS_TYPE_DOWNSTREAM,
		.comp_identifier = SD_COMPNT_BIOS,
		.comp_classification_index = 0x00,
		.pre_update_func = NULL,
		.update_func = NULL,
		.pos_update_func = NULL,
		.inf = COMP_UPDATE_VIA_SPI,
		.activate_method = COMP_ACT_SYS_REBOOT,
		.self_act_func = NULL,
		.get_fw_version_fn = plat_get_bios_fw_version,
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

	// Set the device id for sd bic
	uint8_t deviceId[PLDM_PCI_DEVICE_ID_LENGTH] = { 0x00, 0x00 };

	uint8_t slotNumber = get_slot_eid() / 10;
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

uint8_t plat_pldm_query_downstream_devices(const uint8_t *buf, uint16_t len, uint8_t *resp,
					   uint16_t *resp_len)
{
	CHECK_NULL_ARG_WITH_RETURN(buf, false);
	CHECK_NULL_ARG_WITH_RETURN(resp, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp_len, PLDM_ERROR);

	struct pldm_query_downstream_devices_resp *resp_p =
		(struct pldm_query_downstream_devices_resp *)resp;

	resp_p->completion_code = PLDM_SUCCESS;
	resp_p->downstream_device_update_supported = PLDM_FW_UPDATE_SUPPORT_DOWNSTREAM_DEVICES;
	resp_p->number_of_downstream_devices = 5;
	resp_p->max_number_of_downstream_devices = 5;

	resp_p->capabilities.dynamically_attached = 0;
	resp_p->capabilities.dynamically_removed = 0;
	resp_p->capabilities.support_update_simultaneously = 0;

	*resp_len = sizeof(struct pldm_query_downstream_devices_resp);

	return PLDM_SUCCESS;
}

static uint8_t get_returned_devices_start_index(struct pldm_query_downstream_identifier_req *req,
						uint32_t *index)
{
	switch (req->transferoperationflag) {
	case PLDM_FW_UPDATE_GET_FIRST_PART:
		*(index) = 0;
		break;
	case PLDM_FW_UPDATE_GET_NEXT_PART:
		if (req->datatransferhandle >= downstream_devices_count) {
			LOG_ERR("%s:%s:%d: Invalid data transfer handle: 0x%x", __FILE__, __func__,
				__LINE__, req->datatransferhandle);
			return PLDM_FW_UPDATE_CC_INVALID_TRANSFER_HANDLE;
		}
		*(index) = req->datatransferhandle;
		break;
	default:
		LOG_ERR("%s:%s:%d: Invalid transfer operation flag: 0x%x", __FILE__, __func__,
			__LINE__, req->transferoperationflag);
		return PLDM_FW_UPDATE_CC_INVALID_TRANSFER_OPERATION_FLAG;
	}

	return PLDM_SUCCESS;
}

static size_t calculate_descriptors_size(struct pldm_descriptor_string *descriptors, uint8_t count)
{
	size_t size = 0;
	for (size_t i = 0; i < count; i++) {
		size += strlen(descriptors[i].descriptor_data);
		switch (descriptors[i].descriptor_type) {
		case PLDM_FWUP_VENDOR_DEFINED:
			size += sizeof(struct pldm_vendor_defined_descriptor_tlv) +
						descriptors[i].title_string ?
					strlen(descriptors[i].title_string) :
					0;
			break;
		default:
			size += sizeof(struct pldm_descriptor_tlv);
			break;
		}
	}

	return size;
}

static bool remaining_data_can_be_returned_in_one_transaction(uint32_t start_index,
							      uint32_t *next_transaction_index)
{
	size_t total_size = sizeof(pldm_hdr) + sizeof(struct pldm_query_downstream_identifier_resp);
	uint32_t i = start_index;
	while (i < downstream_devices_count) {
		total_size += sizeof(struct pldm_downstream_device) +
			      calculate_descriptors_size(downstream_table[i].descriptor,
							 downstream_table[i].descriptor_count);
		if (total_size >= PLDM_MAX_DATA_SIZE) {
			*next_transaction_index = i;
			return false;
		}

		i++;
	}
	*next_transaction_index = 0;

	return true;
}

uint8_t plat_pldm_query_downstream_identifiers(const uint8_t *buf, uint16_t len, uint8_t *resp,
					       uint16_t *resp_len)
{
	CHECK_NULL_ARG_WITH_RETURN(buf, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp_len, PLDM_ERROR);

	uint8_t rc = PLDM_ERROR;

	struct pldm_query_downstream_identifier_req *req_p =
		(struct pldm_query_downstream_identifier_req *)buf;
	struct pldm_query_downstream_identifier_resp *resp_p =
		(struct pldm_query_downstream_identifier_resp *)resp;

	resp_p->completion_code = PLDM_ERROR;

	uint32_t start_index = 0;
	rc = get_returned_devices_start_index(req_p, &start_index);
	if (rc) {
		LOG_ERR("%s:%s:%d: Failed to get returning devices start index.", __FILE__,
			__func__, __LINE__);
		return rc;
	}

	uint32_t next_transaction_index = 0;
	if (remaining_data_can_be_returned_in_one_transaction(start_index,
							      &next_transaction_index)) {
		resp_p->nextdatatransferhandle = 0;
		resp_p->numbwerofdownstreamdevice = downstream_devices_count - start_index;
		switch (req_p->transferoperationflag) {
		case PLDM_FW_UPDATE_GET_FIRST_PART:
			resp_p->transferflag = PLDM_FW_UPDATE_TRANSFER_START_AND_END;
			break;
		case PLDM_FW_UPDATE_GET_NEXT_PART:
			resp_p->transferflag = PLDM_FW_UPDATE_TRANSFER_END;
			break;
		}
	} else {
		resp_p->nextdatatransferhandle = next_transaction_index;
		resp_p->numbwerofdownstreamdevice = resp_p->nextdatatransferhandle - start_index;
		switch (req_p->transferoperationflag) {
		case PLDM_FW_UPDATE_GET_FIRST_PART:
			resp_p->transferflag = PLDM_FW_UPDATE_TRANSFER_START;
			break;
		case PLDM_FW_UPDATE_GET_NEXT_PART:
			resp_p->transferflag = PLDM_FW_UPDATE_TRANSFER_MIDDLE;
			break;
		}
	}

	uint32_t downstream_devices_length = 0;
	uint8_t curr_descriptor_length = 0;
	struct pldm_downstream_device *curr_device =
		(struct pldm_downstream_device
			 *)(resp + sizeof(struct pldm_query_downstream_identifier_resp));
	struct pldm_descriptor_string *curr_descriptors_tbl;

	for (uint32_t i = start_index; i < start_index + resp_p->numbwerofdownstreamdevice; i++) {
		curr_descriptors_tbl = downstream_table[i].descriptor;
		curr_device->downstreamdeviceindex = i + 1;
		curr_device->downstreamdescriptorcount = downstream_table[i].descriptor_count;
		downstream_devices_length += sizeof(struct pldm_downstream_device);
		uint8_t *descriptor_ptr = curr_device->downstreamdescriptors;
		for (uint32_t j = 0; j < downstream_table[i].descriptor_count; j++) {
			rc = fill_descriptor_into_buf(&curr_descriptors_tbl[j], descriptor_ptr,
						      &curr_descriptor_length,
						      downstream_devices_length);
			if (rc) {
				LOG_ERR("%s:%s:%d: Fill device descriptor into buffer fail.",
					__FILE__, __func__, __LINE__);
				return PLDM_ERROR;
			}
			downstream_devices_length += curr_descriptor_length;
			descriptor_ptr += curr_descriptor_length;
		}

		curr_device = (struct pldm_downstream_device *)descriptor_ptr;
	}

	resp_p->downstreamdevicelength = downstream_devices_length;
	*resp_len =
		sizeof(struct pldm_query_downstream_identifier_resp) + downstream_devices_length;
	resp_p->completion_code = PLDM_SUCCESS;

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

int load_mctp_support_types(uint8_t *type_len, uint8_t *types)
{
	*type_len = sizeof(MCTP_SUPPORTED_MESSAGES_TYPES);
	memcpy(types, MCTP_SUPPORTED_MESSAGES_TYPES, sizeof(MCTP_SUPPORTED_MESSAGES_TYPES));
	return MCTP_SUCCESS;
}

static uint8_t plat_pldm_pre_vr_update(void *fw_update_param)
{
	CHECK_NULL_ARG_WITH_RETURN(fw_update_param, 1);

	pldm_fw_update_param_t *p = (pldm_fw_update_param_t *)fw_update_param;

	/* Stop sensor polling */
	set_vr_monitor_status(false);
	p->bus = I2C_BUS4;

	if (p->comp_id == SD_COMPNT_VR_PVDDCR_CPU1) {
		p->addr = 0x63;
	} else if (p->comp_id == SD_COMPNT_VR_PVDD11_S3) {
		p->addr = 0x72;
	} else if (p->comp_id == SD_COMPNT_VR_PVDDCR_CPU0) {
		p->addr = 0x76;
	} else {
		LOG_ERR("Unsupported VR image");
	}

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
	uint32_t version;
	uint16_t remain = 0xFFFF;
	uint8_t bus = I2C_BUS4;
	uint8_t addr = 0;
	uint8_t vr_type = VR_TYPE_UNKNOWN, vr_dev = VR_DEVICE_UNKNOWN;

	if (p->comp_identifier == SD_COMPNT_VR_PVDDCR_CPU1) {
		addr = 0x63;
	} else if (p->comp_identifier == SD_COMPNT_VR_PVDD11_S3) {
		addr = 0x72;
	} else if (p->comp_identifier == SD_COMPNT_VR_PVDDCR_CPU0) {
		addr = 0x76;
	} else {
		LOG_ERR("Unknown component identifier for VR");
	}

	if (plat_pldm_sensor_get_vr_dev(&vr_dev) != 0) {
		LOG_ERR("Failed to get VR device");
		return ret;
	}

	switch (vr_dev) {
	case sensor_dev_mp2856gut:
		vr_type = VR_TYPE_MPS;
		break;
	case sensor_dev_raa229621:
		vr_type = VR_TYPE_RNS;
		break;
	case sensor_dev_xdpe19283b:
		vr_type = VR_TYPE_IFX;
		break;
	case sensor_dev_tps53689:
		vr_type = VR_TYPE_TI;
		break;
	default:
		LOG_ERR("Unknown VR type");
		return ret;
	}

	const char *vr_name[] = {
		[VR_TYPE_MPS] = "MPS ",	     [VR_TYPE_RNS] = "Renesas ",
		[VR_TYPE_IFX] = "Infineon ", [VR_TYPE_TI] = "Texas Instruments ",
		[VR_TYPE_UNKNOWN] = NULL,
	};

	const uint8_t *vr_name_p = vr_name[vr_type];
	set_vr_monitor_status(false);
	switch (vr_type) {
	case VR_TYPE_MPS:
		if (!mp2971_get_checksum(bus, addr, &version)) {
			LOG_ERR("Read VR checksum failed");
			return ret;
		}
		break;
	case VR_TYPE_RNS:
		if (!raa229621_get_crc(bus, addr, &version)) {
			LOG_ERR("Read VR checksum failed");
			return ret;
		}

		if (raa229621_get_remaining_wr(bus, addr, (uint8_t *)&remain) < 0) {
			LOG_ERR("Read VR remaining write failed");
			return ret;
		}
		break;
	case VR_TYPE_IFX:
		LOG_ERR("Infineon VR does not support FW version read");
		break;
	case VR_TYPE_TI:
		if (!tps536xx_get_crc(bus, addr, &version)) {
			LOG_ERR("Read TI VR checksum failed");
			return ret;
		}
		break;
	default:
		LOG_ERR("Unknown VR device");
		return ret;
	}
	set_vr_monitor_status(true);

	version = sys_cpu_to_be32(version);
	const char *remain_str_p = ", Remaining Write: ";
	uint8_t *buf_p = buf;
	*len = 0;

	if (!vr_name_p) {
		LOG_ERR("The pointer of VR string name is NULL");
		return ret;
	}

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

static uint8_t plat_pldm_pre_retimer_update(void *fw_update_param)
{
	CHECK_NULL_ARG_WITH_RETURN(fw_update_param, 1);

	pldm_fw_update_param_t *p = (pldm_fw_update_param_t *)fw_update_param;

	p->bus = I2C_BUS6;

	if (p->comp_id == SD_COMPNT_X16_RETIMER) {
		p->addr = X16_RETIMER_ADDR;
	} else {
		p->addr = X8_RETIMER_ADDR;
	}

	return 0;
}

static bool plat_get_retimer_fw_version(void *info_p, uint8_t *buf, uint8_t *len)
{
	CHECK_NULL_ARG_WITH_RETURN(info_p, false);
	CHECK_NULL_ARG_WITH_RETURN(buf, false);
	CHECK_NULL_ARG_WITH_RETURN(len, false);

	pldm_fw_update_info_t *p = (pldm_fw_update_info_t *)info_p;

	bool ret = false;
	uint8_t version[RETIMER_PT5161L_FW_VER_LEN];
	I2C_MSG i2c_msg;

	i2c_msg.bus = I2C_BUS6;

	if (p->comp_identifier == SD_COMPNT_X16_RETIMER) {
		i2c_msg.target_addr = X16_RETIMER_ADDR;
	} else if (p->comp_identifier == SD_COMPNT_X8_RETIMER) {
		i2c_msg.target_addr = X8_RETIMER_ADDR;
	} else {
		LOG_ERR("Unknown component identifier for retimer");
		return ret;
	}

	uint8_t *buf_p = buf;
	ret = get_retimer_fw_version(&i2c_msg, version);
	memcpy(buf_p, version, RETIMER_PT5161L_FW_VER_LEN);
	*len += bin2hex(version, 4, buf_p, 8);
	buf_p += 8;

	return ret;
}

static bool plat_get_bios_fw_version(void *info_p, uint8_t *buf, uint8_t *len)
{
	CHECK_NULL_ARG_WITH_RETURN(info_p, false);
	CHECK_NULL_ARG_WITH_RETURN(buf, false);
	CHECK_NULL_ARG_WITH_RETURN(len, false);

	bool ret = false;
	const char *bios_version = pldm_smbios_get_bios_version();
	if (!bios_version) {
		memcpy(buf, "N/A", 3);
		*len = 3;
	} else {
		memcpy(buf, bios_version, strlen(bios_version));
	}

	ret = true;
	*len = strlen(bios_version);

	return ret;
}
