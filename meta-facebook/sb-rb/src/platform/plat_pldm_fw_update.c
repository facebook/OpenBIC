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

#include "pldm_firmware_update.h"
#include "plat_pldm_fw_update.h"
#include "plat_pldm_sensor.h"
#include "plat_class.h"
#include "plat_hook.h"
#include "mp2971.h"
#include "mp29816a.h"
#include "raa228249.h"

LOG_MODULE_REGISTER(plat_fwupdate);

static uint8_t pldm_pre_vr_update(void *fw_update_param);
static uint8_t pldm_post_vr_update(void *fw_update_param);
static bool get_vr_fw_version(void *info_p, uint8_t *buf, uint8_t *len);

typedef struct {
	uint8_t firmware_comp_id;
	uint8_t plat_pldm_sensor_id;
	char sensor_name[MAX_AUX_SENSOR_NAME_LEN];
} compnt_mapping_sensor;

compnt_mapping_sensor vr_compnt_mapping_sensor_table[] = {
	{ COMPNT_VR_1, SENSOR_NUM_ASIC_P0V85_MEDHA0_VDD_TEMP_C, "ASIC_P0V85_MEDHA0_VDD" },
	{ COMPNT_VR_2, SENSOR_NUM_ASIC_P0V85_MEDHA1_VDD_TEMP_C, "ASIC_P0V85_MEDHA1_VDD" },
	{ COMPNT_VR_3, SENSOR_NUM_ASIC_P0V9_OWL_E_TRVDD_TEMP_C, "ASIC_P0V9_OWL_E_TRVDD" },
	{ COMPNT_VR_4, SENSOR_NUM_ASIC_P0V75_MAX_M_VDD_TEMP_C, "ASIC_P0V75_MAX_M_VDD" },
	{ COMPNT_VR_5, SENSOR_NUM_ASIC_P0V75_OWL_E_VDD_TEMP_C, "ASIC_P0V75_OWL_E_VDD" },
	{ COMPNT_VR_6, SENSOR_NUM_ASIC_P1V1_VDDQC_HBM1357_TEMP_C, "ASIC_P1V1_VDDQC_HBM1357" },
	{ COMPNT_VR_7, SENSOR_NUM_ASIC_P0V75_MAX_N_VDD_TEMP_C, "ASIC_P0V75_MAX_N_VDD" },
	{ COMPNT_VR_8, SENSOR_NUM_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_TEMP_C,
	  "ASIC_P1V2_HAMSA_VDDHRXTX_PCIE" },
	{ COMPNT_VR_9, SENSOR_NUM_ASIC_P1V1_VDDQC_HBM0246_TEMP_C, "ASIC_P1V1_VDDQC_HBM0246" },
	{ COMPNT_VR_10, SENSOR_NUM_ASIC_P0V4_VDDQL_HBM0246_TEMP_C, "ASIC_P0V4_VDDQL_HBM0246" },
	{ COMPNT_VR_11, SENSOR_NUM_ASIC_P0V75_OWL_W_VDD_TEMP_C, "ASIC_P0V75_OWL_W_VDD" },
	{ COMPNT_VR_12, SENSOR_NUM_ASIC_P0V9_OWL_W_TRVDD_TEMP_C, "ASIC_P0V9_OWL_W_TRVDD" },
};

// clang-format off
#define VR_COMPONENT_DEF(comp_id)                                                                  \
	{                                                                                          \
		.enable = true,                                                                    \
		.comp_classification = COMP_CLASS_TYPE_DOWNSTREAM,                                 \
		.comp_identifier = comp_id,                                                        \
		.comp_classification_index = 0x00,                                                 \
		.pre_update_func = pldm_pre_vr_update,                                             \
		.update_func = pldm_vr_update,                                                     \
		.pos_update_func = pldm_post_vr_update,                                            \
		.inf = COMP_UPDATE_VIA_I2C,                                                        \
		.activate_method = COMP_ACT_AC_PWR_CYCLE,                                          \
		.self_act_func = NULL,                                                             \
		.get_fw_version_fn = get_vr_fw_version,                                            \
		.self_apply_work_func = NULL,                                                      \
		.comp_version_str = NULL,                                                          \
	}
// clang-format on

/* PLDM FW update table */
pldm_fw_update_info_t PLDMUPDATE_FW_CONFIG_TABLE[] = {
	{
		.enable = true,
		.comp_classification = COMP_CLASS_TYPE_DOWNSTREAM,
		.comp_identifier = COMPNT_BIC,
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
	VR_COMPONENT_DEF(COMPNT_VR_1),
	VR_COMPONENT_DEF(COMPNT_VR_2),
	VR_COMPONENT_DEF(COMPNT_VR_3),
	VR_COMPONENT_DEF(COMPNT_VR_4),
	VR_COMPONENT_DEF(COMPNT_VR_5),
	VR_COMPONENT_DEF(COMPNT_VR_6),
	VR_COMPONENT_DEF(COMPNT_VR_7),
	VR_COMPONENT_DEF(COMPNT_VR_8),
	VR_COMPONENT_DEF(COMPNT_VR_9),
	VR_COMPONENT_DEF(COMPNT_VR_10),
	VR_COMPONENT_DEF(COMPNT_VR_11),
	VR_COMPONENT_DEF(COMPNT_VR_12),
};

uint8_t plat_pldm_query_device_identifiers(const uint8_t *buf, uint16_t len, uint8_t *resp,
					   uint16_t *resp_len)
{
	CHECK_NULL_ARG_WITH_RETURN(buf, false);
	CHECK_NULL_ARG_WITH_RETURN(resp, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp_len, PLDM_ERROR);

	LOG_INF("pldm_query_device_identifiers");

	struct pldm_query_device_identifiers_resp *resp_p =
		(struct pldm_query_device_identifiers_resp *)resp;

	resp_p->completion_code = PLDM_SUCCESS;
	resp_p->descriptor_count = 0x03;

	uint8_t iana[PLDM_FWUP_IANA_ENTERPRISE_ID_LENGTH] = { 0x00, 0x00, 0xA0, 0x15 };

	// Set the device id for sd bic
	uint8_t deviceId[PLDM_PCI_DEVICE_ID_LENGTH] = { 0x00, 0x00 };

	uint8_t total_size_of_iana_descriptor =
		sizeof(struct pldm_descriptor_tlv) + sizeof(iana) - 1;

	uint8_t total_size_of_device_id_descriptor =
		sizeof(struct pldm_descriptor_tlv) + sizeof(deviceId) - 1;

	if (sizeof(struct pldm_query_device_identifiers_resp) + total_size_of_iana_descriptor +
		    total_size_of_device_id_descriptor >
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

	resp_p->device_identifiers_len =
		total_size_of_iana_descriptor + total_size_of_device_id_descriptor;

	*resp_len = sizeof(struct pldm_query_device_identifiers_resp) +
		    total_size_of_iana_descriptor + total_size_of_device_id_descriptor;

	LOG_INF("pldm_query_device_identifiers done");
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

// vr update
static uint8_t pldm_pre_vr_update(void *fw_update_param)
{
	CHECK_NULL_ARG_WITH_RETURN(fw_update_param, 1);

	pldm_fw_update_param_t *p = (pldm_fw_update_param_t *)fw_update_param;

	/* Stop sensor polling */
	set_plat_sensor_polling_enable_flag(false);
	k_msleep(100);

	uint8_t sensor_id = 0;
	char sensor_name[MAX_AUX_SENSOR_NAME_LEN] = { 0 };

	if (!find_sensor_id_and_name_by_firmware_comp_id(p->comp_id, &sensor_id, sensor_name)) {
		LOG_ERR("Can't find sensor id and name by comp id: 0x%x", p->comp_id);
		return 1;
	}

	sensor_cfg *cfg = get_sensor_cfg_by_sensor_id(sensor_id);
	CHECK_NULL_ARG_WITH_RETURN(cfg, 1);

	/* Get bus and target address by sensor number in sensor configuration */
	p->bus = cfg->port;
	p->addr = cfg->target_addr;

	return 0;
}
static uint8_t pldm_post_vr_update(void *fw_update_param)
{
	ARG_UNUSED(fw_update_param);

	/* Start sensor polling */
	set_plat_sensor_polling_enable_flag(true);

	return 0;
}
static bool get_vr_fw_version(void *info_p, uint8_t *buf, uint8_t *len)
{
	CHECK_NULL_ARG_WITH_RETURN(info_p, false);
	CHECK_NULL_ARG_WITH_RETURN(buf, false);
	CHECK_NULL_ARG_WITH_RETURN(len, false);

	pldm_fw_update_info_t *p = (pldm_fw_update_info_t *)info_p;

	bool ret = false;
	uint8_t sensor_id = 0;
	char sensor_name[MAX_AUX_SENSOR_NAME_LEN] = { 0 };

	if (is_mb_dc_on() == false)
		return ret;

	if (!find_sensor_id_and_name_by_firmware_comp_id(p->comp_identifier, &sensor_id,
							 sensor_name)) {
		LOG_ERR("Can't find sensor id and name by comp id: 0x%x", p->comp_identifier);
		return ret;
	}

	sensor_cfg *cfg = get_sensor_cfg_by_sensor_id(sensor_id);
	CHECK_NULL_ARG_WITH_RETURN(cfg, ret);

	if ((cfg->pre_sensor_read_hook)) {
		if ((cfg->pre_sensor_read_hook)(cfg, cfg->pre_sensor_read_args) == false) {
			LOG_DBG("%d read vr fw pre hook fail!", sensor_id);
			return false;
		}
	};

	uint8_t vr_module = get_vr_module();
	uint32_t version = 0;
	uint16_t remain = 0xFFFF;
	switch (cfg->type) {
	case sensor_dev_mp2971:
		if (!mp2971_get_checksum(cfg->port, cfg->target_addr, &version)) {
			LOG_ERR("The VR MPS2971 version reading failed");
			goto err;
		}
		break;
	case sensor_dev_mp29816a:
		if (!mp29816a_get_fw_version(cfg->port, cfg->target_addr, &version)) {
			LOG_ERR("The VR MPS29816a version reading failed");
			goto err;
		}
		break;
	case sensor_dev_raa228249:
		if (!raa228249_get_crc(cfg->port, cfg->target_addr, &version)) {
			LOG_ERR("The VR RAA228249 version reading failed");
			goto err;
		}
		if (raa228249_get_remaining_wr(cfg->port, cfg->target_addr, (uint8_t *)&remain) <
		    0) {
			LOG_ERR("The VR RAA228249 remaining reading failed");
			goto err;
		}
		break;
	default:
		LOG_ERR("Unsupport VR type(%d)", cfg->type);
		goto err;
	}

	if (cfg->type == sensor_dev_mp2891 || cfg->type == sensor_dev_mp29816a)
		version = sys_cpu_to_be16(version);
	else if (cfg->type == sensor_dev_raa228249 || cfg->type == sensor_dev_mp2971)
		version = sys_cpu_to_be32(version);
	else
		LOG_ERR("Unsupport VR type(%d)", cfg->type);

	const char *vr_name[] = {
		[VR_MODULE_MPS] = "MPS ",
		[VR_MODULE_RNS] = "RNS ",
		[VR_MODULE_UNKNOWN] = NULL,
	};

	const char *remain_str_p = ", Remaining Write: ";
	uint8_t *buf_p = buf;
	const uint8_t *vr_name_p = vr_name[vr_module];
	*len = 0;

	if (!vr_name_p) {
		LOG_ERR("The pointer of VR string name is NULL");
		goto err;
	}

	memcpy(buf_p, vr_name_p, strlen(vr_name_p));
	buf_p += strlen(vr_name_p);
	*len += strlen(vr_name_p);

	if (cfg->type == sensor_dev_mp2891 || cfg->type == sensor_dev_mp29816a) {
		*len += bin2hex((uint8_t *)&version, 2, buf_p, 4);
		buf_p += 4;
	} else {
		LOG_ERR("Unsupport VR type(%d)", cfg->type);
	}

	memcpy(buf_p, remain_str_p, strlen(remain_str_p));
	buf_p += strlen(remain_str_p);
	*len += strlen(remain_str_p);

	if (remain != 0xFFFF) {
		uint8_t packed_remain = (uint8_t)((remain % 10) | (remain / 10 << 4));
		*len += bin2hex(&packed_remain, 1, buf_p, 2);
		buf_p += 2;
	} else {
		*len += bin2hex((uint8_t *)&remain, 2, buf_p, 4);
		buf_p += 4;
	}

	ret = true;

err:
	if ((cfg->post_sensor_read_hook)) {
		if ((cfg->post_sensor_read_hook)(cfg, cfg->post_sensor_read_args, 0) == false) {
			LOG_DBG("%d read vr fw post hook fail!", sensor_id);
			ret = false;
		}
	}
	return ret;
}

bool find_sensor_id_and_name_by_firmware_comp_id(uint8_t comp_identifier, uint8_t *sensor_id,
						 char *sensor_name)
{
	CHECK_NULL_ARG_WITH_RETURN(sensor_id, false);
	CHECK_NULL_ARG_WITH_RETURN(sensor_name, false);

	for (uint8_t i = 0; i < ARRAY_SIZE(vr_compnt_mapping_sensor_table); i++) {
		if (vr_compnt_mapping_sensor_table[i].firmware_comp_id == comp_identifier) {
			*sensor_id = vr_compnt_mapping_sensor_table[i].plat_pldm_sensor_id;
			strncpy(sensor_name, vr_compnt_mapping_sensor_table[i].sensor_name,
				MAX_AUX_SENSOR_NAME_LEN);
			return true;
		}
	}

	return false;
}