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
#include "plat_fwupdate.h"
#include "pldm_firmware_update.h"
#include "sensor.h"
#include "plat_sensor_table.h"
#include "plat_hook.h"

LOG_MODULE_REGISTER(plat_fwupdate);

static uint8_t pldm_vr_update(uint16_t comp_id, void *mctp_p, void *ext_params);
static uint8_t pldm_pex_update(uint16_t comp_id, void *mctp_p, void *ext_params);
static uint8_t pldm_cpld_update(uint16_t comp_id, void *mctp_p, void *ext_params);

/* PLDM FW update table */
// clang-format off
pldm_fw_update_info_t PLDMUPDATE_FW_CONFIG_TABLE[] = {
	{ ENABLE, COMP_CLASS_TYPE_DOWNSTREAM, COMP_ID_BIC, 0x00, pldm_bic_update, COMP_ACT_SELF, pldm_bic_activate },
	{ ENABLE, COMP_CLASS_TYPE_DOWNSTREAM, COMP_ID_VR0, 0x00, pldm_vr_update, COMP_ACT_AC_PWR_CYCLE, NULL },
	{ ENABLE, COMP_CLASS_TYPE_DOWNSTREAM, COMP_ID_VR1, 0x00, pldm_vr_update, COMP_ACT_AC_PWR_CYCLE, NULL },
	{ ENABLE, COMP_CLASS_TYPE_DOWNSTREAM, COMP_ID_PEX0, 0x00, pldm_pex_update, COMP_ACT_AC_PWR_CYCLE, NULL },
	{ ENABLE, COMP_CLASS_TYPE_DOWNSTREAM, COMP_ID_PEX1, 0x00, pldm_pex_update, COMP_ACT_AC_PWR_CYCLE, NULL },
	{ ENABLE, COMP_CLASS_TYPE_DOWNSTREAM, COMP_ID_PEX2, 0x00, pldm_pex_update, COMP_ACT_AC_PWR_CYCLE, NULL },
	{ ENABLE, COMP_CLASS_TYPE_DOWNSTREAM, COMP_ID_PEX3, 0x00, pldm_pex_update, COMP_ACT_AC_PWR_CYCLE, NULL },
	{ ENABLE, COMP_CLASS_TYPE_DOWNSTREAM, COMP_ID_CPLD, 0x00, pldm_cpld_update, COMP_ACT_AC_PWR_CYCLE, NULL },
};
// clang-format on

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

static uint8_t pldm_vr_update(uint16_t comp_id, void *mctp_p, void *ext_params)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_p, 1);
	CHECK_NULL_ARG_WITH_RETURN(ext_params, 1);

	LOG_WRN("Not support vr pldm update yet!");

	return 0;
}

static uint8_t pldm_cpld_update(uint16_t comp_id, void *mctp_p, void *ext_params)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_p, 1);
	CHECK_NULL_ARG_WITH_RETURN(ext_params, 1);

	LOG_WRN("Not support pex pldm update yet!");

	return 0;
}

static uint8_t pldm_pex_update(uint16_t comp_id, void *mctp_p, void *ext_params)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_p, 1);
	CHECK_NULL_ARG_WITH_RETURN(ext_params, 1);

	LOG_WRN("Not support pex pldm update yet!");

	return 0;
}
