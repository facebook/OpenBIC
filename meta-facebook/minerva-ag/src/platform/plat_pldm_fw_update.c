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
#include "sensor.h"
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
#include "mp2891.h"
#include "raa229621.h"
#include "raa228249.h"
#include "plat_class.h"
#include "pldm_sensor.h"
#include "mp29816a.h"
#include "plat_hook.h"
#include "plat_event.h"

LOG_MODULE_REGISTER(plat_fwupdate);

static uint8_t pldm_pre_vr_update(void *fw_update_param);
static uint8_t pldm_post_vr_update(void *fw_update_param);
static uint8_t pldm_pre_bic_update(void *fw_update_param);
static bool get_vr_fw_version(void *info_p, uint8_t *buf, uint8_t *len);

typedef struct aegis_compnt_mapping_sensor {
	uint8_t firmware_comp_id;
	uint8_t plat_pldm_sensor_id;
	char sensor_name[MAX_AUX_SENSOR_NAME_LEN];
} aegis_compnt_mapping_sensor;

aegis_compnt_mapping_sensor aegis_vr_compnt_mapping_sensor_table[] = {
	{ AG_COMPNT_P3V3, VR_P3V3_TEMP_C, "CB_VR_P3V3" },
	{ AG_COMPNT_P0V85_PVDD, VR_ASIC_P0V85_PVDD_TEMP_C, "CB_VR_ASIC_P0V85_PVDD" },
	{ AG_COMPNT_P0V75_PVDD_CH_N, VR_ASIC_P0V75_PVDD_CH_N_TEMP_C, "CB_VR_ASIC_P0V75_PVDD_CH_N" },
	{ AG_COMPNT_P0V75_PVDD_CH_S, VR_ASIC_P0V75_PVDD_CH_S_TEMP_C, "CB_VR_ASIC_P0V75_PVDD_CH_S" },
	{ AG_COMPNT_P0V75_TRVDD_ZONEA, VR_ASIC_P0V75_TRVDD_ZONEA_TEMP_C,
	  "CB_VR_ASIC_P0V75_TRVDD_ZONEA" },
	{ AG_COMPNT_P0V75_TRVDD_ZONEB, VR_ASIC_P0V75_TRVDD_ZONEB_TEMP_C,
	  "CB_VR_ASIC_P0V75_TRVDD_ZONEB" },
	{ AG_COMPNT_P1V1_VDDC_HBM0_HBM2_HBM4, VR_ASIC_P1V1_VDDC_HBM0_HBM2_HBM4_TEMP_C,
	  "CB_VR_ASIC_P1V1_VDDC_HBM0_HBM2_HBM4" },
	{ AG_COMPNT_P0V9_TRVDD_ZONEA, VR_ASIC_P0V9_TRVDD_ZONEA_TEMP_C,
	  "CB_VR_ASIC_P0V9_TRVDD_ZONEA" },
	{ AG_COMPNT_P0V9_TRVDD_ZONEB, VR_ASIC_P0V9_TRVDD_ZONEB_TEMP_C,
	  "CB_VR_ASIC_P0V9_TRVDD_ZONEB" },
	{ AG_COMPNT_P1V1_VDDC_HBM1_HBM3_HBM5, VR_ASIC_P1V1_VDDC_HBM1_HBM3_HBM5_TEMP_C,
	  "CB_VR_ASIC_P1V1_VDDC_HBM1_HBM3_HBM5" },
	{ AG_COMPNT_P0V8_VDDA_PCIE, VR_ASIC_P0V8_VDDA_PCIE_TEMP_C, "CB_VR_ASIC_P0V8_VDDA_PCIE" },
};

/* PLDM FW update table */
pldm_fw_update_info_t PLDMUPDATE_FW_CONFIG_TABLE[] = {
	{
		.enable = true,
		.comp_classification = COMP_CLASS_TYPE_DOWNSTREAM,
		.comp_identifier = AG_COMPNT_BIC,
		.comp_classification_index = 0x00,
		.pre_update_func = pldm_pre_bic_update,
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
		.comp_identifier = AG_COMPNT_P3V3,
		.comp_classification_index = 0x00,
		.pre_update_func = pldm_pre_vr_update,
		.update_func = pldm_vr_update,
		.pos_update_func = pldm_post_vr_update,
		.inf = COMP_UPDATE_VIA_I2C,
		.activate_method = COMP_ACT_AC_PWR_CYCLE,
		.self_act_func = NULL,
		.get_fw_version_fn = get_vr_fw_version,
		.self_apply_work_func = NULL,
		.comp_version_str = NULL,
	},
	{
		.enable = true,
		.comp_classification = COMP_CLASS_TYPE_DOWNSTREAM,
		.comp_identifier = AG_COMPNT_P0V85_PVDD,
		.comp_classification_index = 0x00,
		.pre_update_func = pldm_pre_vr_update,
		.update_func = pldm_vr_update,
		.pos_update_func = pldm_post_vr_update,
		.inf = COMP_UPDATE_VIA_I2C,
		.activate_method = COMP_ACT_AC_PWR_CYCLE,
		.self_act_func = NULL,
		.get_fw_version_fn = get_vr_fw_version,
		.self_apply_work_func = NULL,
		.comp_version_str = NULL,
	},
	{
		.enable = true,
		.comp_classification = COMP_CLASS_TYPE_DOWNSTREAM,
		.comp_identifier = AG_COMPNT_P0V75_PVDD_CH_N,
		.comp_classification_index = 0x00,
		.pre_update_func = pldm_pre_vr_update,
		.update_func = pldm_vr_update,
		.pos_update_func = pldm_post_vr_update,
		.inf = COMP_UPDATE_VIA_I2C,
		.activate_method = COMP_ACT_AC_PWR_CYCLE,
		.self_act_func = NULL,
		.get_fw_version_fn = get_vr_fw_version,
		.self_apply_work_func = NULL,
		.comp_version_str = NULL,
	},
	{
		.enable = true,
		.comp_classification = COMP_CLASS_TYPE_DOWNSTREAM,
		.comp_identifier = AG_COMPNT_P0V75_PVDD_CH_S,
		.comp_classification_index = 0x00,
		.pre_update_func = pldm_pre_vr_update,
		.update_func = pldm_vr_update,
		.pos_update_func = pldm_post_vr_update,
		.inf = COMP_UPDATE_VIA_I2C,
		.activate_method = COMP_ACT_AC_PWR_CYCLE,
		.self_act_func = NULL,
		.get_fw_version_fn = get_vr_fw_version,
		.self_apply_work_func = NULL,
		.comp_version_str = NULL,
	},
	{
		.enable = true,
		.comp_classification = COMP_CLASS_TYPE_DOWNSTREAM,
		.comp_identifier = AG_COMPNT_P0V75_TRVDD_ZONEA,
		.comp_classification_index = 0x00,
		.pre_update_func = pldm_pre_vr_update,
		.update_func = pldm_vr_update,
		.pos_update_func = pldm_post_vr_update,
		.inf = COMP_UPDATE_VIA_I2C,
		.activate_method = COMP_ACT_AC_PWR_CYCLE,
		.self_act_func = NULL,
		.get_fw_version_fn = get_vr_fw_version,
		.self_apply_work_func = NULL,
		.comp_version_str = NULL,
	},
	{
		.enable = true,
		.comp_classification = COMP_CLASS_TYPE_DOWNSTREAM,
		.comp_identifier = AG_COMPNT_P0V75_TRVDD_ZONEB,
		.comp_classification_index = 0x00,
		.pre_update_func = pldm_pre_vr_update,
		.update_func = pldm_vr_update,
		.pos_update_func = pldm_post_vr_update,
		.inf = COMP_UPDATE_VIA_I2C,
		.activate_method = COMP_ACT_AC_PWR_CYCLE,
		.self_act_func = NULL,
		.get_fw_version_fn = get_vr_fw_version,
		.self_apply_work_func = NULL,
		.comp_version_str = NULL,
	},
	{
		.enable = true,
		.comp_classification = COMP_CLASS_TYPE_DOWNSTREAM,
		.comp_identifier = AG_COMPNT_P1V1_VDDC_HBM0_HBM2_HBM4,
		.comp_classification_index = 0x00,
		.pre_update_func = pldm_pre_vr_update,
		.update_func = pldm_vr_update,
		.pos_update_func = pldm_post_vr_update,
		.inf = COMP_UPDATE_VIA_I2C,
		.activate_method = COMP_ACT_AC_PWR_CYCLE,
		.self_act_func = NULL,
		.get_fw_version_fn = get_vr_fw_version,
		.self_apply_work_func = NULL,
		.comp_version_str = NULL,
	},
	{
		.enable = true,
		.comp_classification = COMP_CLASS_TYPE_DOWNSTREAM,
		.comp_identifier = AG_COMPNT_P0V9_TRVDD_ZONEA,
		.comp_classification_index = 0x00,
		.pre_update_func = pldm_pre_vr_update,
		.update_func = pldm_vr_update,
		.pos_update_func = pldm_post_vr_update,
		.inf = COMP_UPDATE_VIA_I2C,
		.activate_method = COMP_ACT_AC_PWR_CYCLE,
		.self_act_func = NULL,
		.get_fw_version_fn = get_vr_fw_version,
		.self_apply_work_func = NULL,
		.comp_version_str = NULL,
	},
	{
		.enable = true,
		.comp_classification = COMP_CLASS_TYPE_DOWNSTREAM,
		.comp_identifier = AG_COMPNT_P0V9_TRVDD_ZONEB,
		.comp_classification_index = 0x00,
		.pre_update_func = pldm_pre_vr_update,
		.update_func = pldm_vr_update,
		.pos_update_func = pldm_post_vr_update,
		.inf = COMP_UPDATE_VIA_I2C,
		.activate_method = COMP_ACT_AC_PWR_CYCLE,
		.self_act_func = NULL,
		.get_fw_version_fn = get_vr_fw_version,
		.self_apply_work_func = NULL,
		.comp_version_str = NULL,
	},
	{
		.enable = true,
		.comp_classification = COMP_CLASS_TYPE_DOWNSTREAM,
		.comp_identifier = AG_COMPNT_P1V1_VDDC_HBM1_HBM3_HBM5,
		.comp_classification_index = 0x00,
		.pre_update_func = pldm_pre_vr_update,
		.update_func = pldm_vr_update,
		.pos_update_func = pldm_post_vr_update,
		.inf = COMP_UPDATE_VIA_I2C,
		.activate_method = COMP_ACT_AC_PWR_CYCLE,
		.self_act_func = NULL,
		.get_fw_version_fn = get_vr_fw_version,
		.self_apply_work_func = NULL,
		.comp_version_str = NULL,
	},
	{
		.enable = true,
		.comp_classification = COMP_CLASS_TYPE_DOWNSTREAM,
		.comp_identifier = AG_COMPNT_P0V8_VDDA_PCIE,
		.comp_classification_index = 0x00,
		.pre_update_func = pldm_pre_vr_update,
		.update_func = pldm_vr_update,
		.pos_update_func = pldm_post_vr_update,
		.inf = COMP_UPDATE_VIA_I2C,
		.activate_method = COMP_ACT_AC_PWR_CYCLE,
		.self_act_func = NULL,
		.get_fw_version_fn = get_vr_fw_version,
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

	size_t filtered_count = 0;
	for (size_t i = 0; i < comp_config_count; i++) {
		// Skip the AG_COMPNT_P3V3 for MINERVA_AEGIS_BD
		if ((get_board_type() == MINERVA_AEGIS_BD) &&
		    PLDMUPDATE_FW_CONFIG_TABLE[i].comp_identifier == AG_COMPNT_P3V3)
			continue;

		comp_config[filtered_count++] = PLDMUPDATE_FW_CONFIG_TABLE[i];
	}

	comp_config_count = filtered_count;
}

/* pldm pre-update func */
static uint8_t pldm_pre_vr_update(void *fw_update_param)
{
	CHECK_NULL_ARG_WITH_RETURN(fw_update_param, 1);

	pldm_fw_update_param_t *p = (pldm_fw_update_param_t *)fw_update_param;

	/* Stop sensor polling */
	set_plat_sensor_polling_enable_flag(false);

	uint8_t bus = 0;
	uint8_t addr = 0;
	uint8_t sensor_id = 0;
	uint8_t sensor_dev = 0;
	char sensor_name[MAX_AUX_SENSOR_NAME_LEN] = { 0 };

	if (!find_sensor_id_and_name_by_firmware_comp_id(p->comp_id, &sensor_id, sensor_name)) {
		LOG_ERR("Can't find sensor id and name by comp id: 0x%x", p->comp_id);
		return 1;
	}

	if (!get_sensor_info_by_sensor_id(sensor_id, &bus, &addr, &sensor_dev)) {
		LOG_ERR("Can't find vr addr and bus by sensor id: 0x%x", sensor_id);
		return 1;
	}

	/* Get bus and target address by sensor number in sensor configuration */
	p->bus = bus;
	p->addr = addr;

	return 0;
}

static uint8_t pldm_pre_bic_update(void *fw_update_param)
{
	ARG_UNUSED(fw_update_param);

	/* Stop sensor polling */
	set_plat_sensor_polling_enable_flag(false);

	return 0;
}

/* pldm post-update func */
static uint8_t pldm_post_vr_update(void *fw_update_param)
{
	ARG_UNUSED(fw_update_param);

	/* Start sensor polling */
	set_plat_sensor_polling_enable_flag(true);

	return 0;
}
static struct k_mutex *get_vr_mutex_by_comp_id(uint8_t comp_id)
{
	uint8_t vr_mutex_id = VR_INDEX_MAX;

	switch (comp_id) {
	case AG_COMPNT_P3V3:
		vr_mutex_id = VR_INDEX_E_P3V3;
		break;
	case AG_COMPNT_P0V85_PVDD:
		vr_mutex_id = VR_INDEX_E_P0V85;
		break;
	case AG_COMPNT_P0V75_PVDD_CH_N:
		vr_mutex_id = VR_INDEX_E_P0V75_CH_N;
		break;
	case AG_COMPNT_P0V75_PVDD_CH_S:
		vr_mutex_id = VR_INDEX_E_P0V75_CH_S;
		break;
	case AG_COMPNT_P0V75_TRVDD_ZONEA:
		vr_mutex_id = VR_INDEX_E_P0V75_TRVDD_ZONEA;
		break;
	case AG_COMPNT_P0V75_TRVDD_ZONEB:
		vr_mutex_id = VR_INDEX_E_P0V75_TRVDD_ZONEB;
		break;
	case AG_COMPNT_P1V1_VDDC_HBM0_HBM2_HBM4:
		vr_mutex_id = VR_INDEX_E_P1V1_VDDC_HBM0_HBM2_HBM4;
		break;
	case AG_COMPNT_P0V9_TRVDD_ZONEA:
		vr_mutex_id = VR_INDEX_E_P0V9_TRVDD_ZONEA;
		break;
	case AG_COMPNT_P0V9_TRVDD_ZONEB:
		vr_mutex_id = VR_INDEX_E_P0V9_TRVDD_ZONEB;
		break;
	case AG_COMPNT_P1V1_VDDC_HBM1_HBM3_HBM5:
		vr_mutex_id = VR_INDEX_E_P1V1_VDDC_HBM1_HBM3_HBM5;
		break;
	case AG_COMPNT_P0V8_VDDA_PCIE:
		vr_mutex_id = VR_INDEX_E_P0V8_VDDA_PCIE;
		break;
	default:
		LOG_ERR("Invalid component id(%d)", comp_id);
		break;
	}

	return vr_mutex_get(vr_mutex_id);
}

static bool get_vr_fw_version(void *info_p, uint8_t *buf, uint8_t *len)
{
	CHECK_NULL_ARG_WITH_RETURN(info_p, false);
	CHECK_NULL_ARG_WITH_RETURN(buf, false);
	CHECK_NULL_ARG_WITH_RETURN(len, false);

	pldm_fw_update_info_t *p = (pldm_fw_update_info_t *)info_p;

	bool ret = false;
	uint8_t bus = 0;
	uint8_t addr = 0;
	uint8_t sensor_id = 0;
	uint8_t sensor_dev = 0;
	char sensor_name[MAX_AUX_SENSOR_NAME_LEN] = { 0 };

	/* is_ubc_enabled_delayed_enabled() is to wait for all VR to be enabled when UBC is enabled  */
	/* (gpio_get(FM_PLD_UBC_EN_R) == GPIO_LOW) is to stop VR FW accessing immediately when UBC is disabled */
	/* is_mb_dc_on() is to start VR FW accessing when all VRs are enabled PW GD */
	if (is_mb_dc_on() == false) {
		if ((gpio_get(FM_PLD_UBC_EN_R) == GPIO_LOW) || !is_ubc_enabled_delayed_enabled()) {
			LOG_INF("Comp id %d FW version request failed due to UBC is not enabled",
				p->comp_identifier);
			return ret;
		}
	}

	if (!find_sensor_id_and_name_by_firmware_comp_id(p->comp_identifier, &sensor_id,
							 sensor_name)) {
		LOG_ERR("Can't find sensor id and name by comp id: 0x%x", p->comp_identifier);
		return ret;
	}

	if (!get_sensor_info_by_sensor_id(sensor_id, &bus, &addr, &sensor_dev)) {
		LOG_ERR("Can't find vr addr and bus by sensor id: 0x%x", sensor_id);
		return ret;
	}

	struct k_mutex *p_mutex = get_vr_mutex_by_comp_id(p->comp_identifier);

	if (!p_mutex) {
		LOG_ERR("vr comp id %d, mutex is NULL", p->comp_identifier);
		return ret;
	}

	if (k_mutex_lock(p_mutex, K_MSEC(VR_MUTEX_LOCK_TIMEOUT_MS))) {
		LOG_ERR("vr comp id %d, mutex %p lock fail", p->comp_identifier, p_mutex);
		return ret;
	}
	LOG_DBG("vr comp id %d, mutex %p lock", p->comp_identifier, p_mutex);

	uint8_t type = get_vr_type();
	uint32_t version = 0;
	uint16_t remain = 0xFFFF;
	switch (sensor_dev) {
	case sensor_dev_isl69259:
		if (!raa229621_get_crc(bus, addr, &version)) {
			LOG_ERR("The VR ISL69260 version reading failed");
			goto err;
		}
		if (raa229621_get_remaining_wr(bus, addr, (uint8_t *)&remain) < 0) {
			LOG_ERR("The VR ISL69260 remaining reading failed");
			goto err;
		}
		break;
	case sensor_dev_raa228238:
		if (!raa229621_get_crc(bus, addr, &version)) {
			LOG_ERR("The VR RAA228238 version reading failed");
			goto err;
		}
		if (raa229621_get_remaining_wr(bus, addr, (uint8_t *)&remain) < 0) {
			LOG_ERR("The VR RAA228238 remaining reading failed");
			goto err;
		}
		break;
	case sensor_dev_mp2971:
		if (!mp2971_get_checksum(bus, addr, &version)) {
			LOG_ERR("The VR MPS2971 version reading failed");
			goto err;
		}
		break;
	case sensor_dev_mp2891:
		if (!mp2891_get_fw_version(bus, addr, &version)) {
			LOG_ERR("The VR MPS2891 version reading failed");
			goto err;
		}
		break;
	case sensor_dev_mp29816a:
		if (!mp29816a_get_fw_version(bus, addr, &version)) {
			LOG_ERR("The VR MPS29816a version reading failed");
			goto err;
		}
		break;
	case sensor_dev_raa228249:
		if (!raa228249_get_crc(bus, addr, &version)) {
			LOG_ERR("The VR RAA228249 version reading failed");
			goto err;
		}
		if (raa228249_get_remaining_wr(bus, addr, (uint8_t *)&remain) < 0) {
			LOG_ERR("The VR RAA228249 remaining reading failed");
			goto err;
		}
		break;
	default:
		LOG_ERR("Unsupport VR type(%d)", type);
		goto err;
	}

	if (sensor_dev == sensor_dev_mp2891 || sensor_dev == sensor_dev_mp29816a)
		version = sys_cpu_to_be16(version);
	else if (sensor_dev == sensor_dev_isl69259 || sensor_dev == sensor_dev_raa228238 ||
		 sensor_dev == sensor_dev_raa228249 || sensor_dev == sensor_dev_mp2971)
		version = sys_cpu_to_be32(version);
	else
		LOG_ERR("Unsupport VR type(%d)", type);

	const char *vr_name[] = {
		[VR_RNS_ISL69260_RAA228238] = "RNS ",
		[VR_MPS_MP2971_MP2891] = "MPS ",
		[VR_RNS_ISL69260_RAA228249] = "RNS ",
		[VR_MPS_MP2971_MP29816A] = "MPS ",
		[VR_UNKNOWN] = NULL,
	};

	const char *remain_str_p = ", Remaining Write: ";
	uint8_t *buf_p = buf;
	const uint8_t *vr_name_p = vr_name[type];
	*len = 0;

	if (!vr_name_p) {
		LOG_ERR("The pointer of VR string name is NULL");
		goto err;
	}

	memcpy(buf_p, vr_name_p, strlen(vr_name_p));
	buf_p += strlen(vr_name_p);
	*len += strlen(vr_name_p);

	if (sensor_dev == sensor_dev_mp2891 || sensor_dev == sensor_dev_mp29816a) {
		*len += bin2hex((uint8_t *)&version, 2, buf_p, 4);
		buf_p += 4;
	} else if (sensor_dev == sensor_dev_isl69259 || sensor_dev == sensor_dev_raa228238 ||
		   sensor_dev == sensor_dev_raa228249 || sensor_dev == sensor_dev_mp2971) {
		*len += bin2hex((uint8_t *)&version, 4, buf_p, 8);
		buf_p += 8;
	} else
		LOG_ERR("Unsupport VR type(%d)", type);

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
	LOG_DBG("vr comp id %d, mutex %p unlock", p->comp_identifier, p_mutex);
	if (k_mutex_unlock(p_mutex))
		LOG_ERR("vr comp id %d, mutex %p unlock fail", p->comp_identifier, p_mutex);

	return ret;
}

void clear_pending_version(uint8_t activate_method)
{
	if (!comp_config || !comp_config_count) {
		LOG_ERR("Component configuration is empty");
		return;
	}

	for (uint8_t i = 0; i < comp_config_count; i++) {
		if (comp_config[i].activate_method == activate_method)
			SAFE_FREE(comp_config[i].pending_version_p);
	}
}

bool find_sensor_id_and_name_by_firmware_comp_id(uint8_t comp_identifier, uint8_t *sensor_id,
						 char *sensor_name)
{
	CHECK_NULL_ARG_WITH_RETURN(sensor_id, false);
	CHECK_NULL_ARG_WITH_RETURN(sensor_name, false);

	for (uint8_t i = 0; i < ARRAY_SIZE(aegis_vr_compnt_mapping_sensor_table); i++) {
		if (aegis_vr_compnt_mapping_sensor_table[i].firmware_comp_id == comp_identifier) {
			*sensor_id = aegis_vr_compnt_mapping_sensor_table[i].plat_pldm_sensor_id;
			strncpy(sensor_name, aegis_vr_compnt_mapping_sensor_table[i].sensor_name,
				MAX_AUX_SENSOR_NAME_LEN);
			return true;
		}
	}

	return false;
}

int get_aegis_vr_compnt_mapping_sensor_table_count(void)
{
	int count = 0;
	count = ARRAY_SIZE(aegis_vr_compnt_mapping_sensor_table);
	return count;
}
