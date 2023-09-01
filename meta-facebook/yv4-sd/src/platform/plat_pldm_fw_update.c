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
#include "plat_pldm_fw_update.h"
#include "mctp_ctrl.h"

LOG_MODULE_REGISTER(plat_fwupdate);

enum FIRMWARE_COMPONENT {
	SD_COMPNT_BIC,
};

uint8_t MCTP_SUPPORTED_MESSAGES_TYPES[] = {
	TYPE_MCTP_CONTROL,
	TYPE_PLDM,
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
	resp_p->descriptor_count = 0x01;

	uint8_t iana[PLDM_FWUP_IANA_ENTERPRISE_ID_LENGTH] = { 0x00, 0x00, 0xA0, 0x15 };

	// Allocate data for tlv which including descriptors data
	struct pldm_descriptor_tlv *tlv_ptr =
		malloc(sizeof(struct pldm_descriptor_tlv) + sizeof(iana) - 1);

	tlv_ptr->descriptor_type = PLDM_FWUP_IANA_ENTERPRISE_ID;
	tlv_ptr->descriptor_length = PLDM_FWUP_IANA_ENTERPRISE_ID_LENGTH;
	memcpy(tlv_ptr->descriptor_data, iana, sizeof(iana));

	// Set pointer to the end of identifiers
	uint8_t *end_of_id_ptr =
		(uint8_t *)resp + sizeof(struct pldm_query_device_identifiers_resp);

	uint8_t total_len_of_tlv = sizeof(struct pldm_descriptor_tlv) + sizeof(iana) - 1;

	// Copy tlv at end of identifiers
	memcpy(end_of_id_ptr, tlv_ptr, total_len_of_tlv);

	resp_p->device_identifiers_len = total_len_of_tlv;

	*resp_len = sizeof(struct pldm_query_device_identifiers_resp) + total_len_of_tlv;

	free(tlv_ptr);
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
