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
#include "i2c-mux-tca9548.h"

LOG_MODULE_REGISTER(plat_fwupdate);

static uint8_t pldm_pre_vr_update(void *fw_update_param);
static uint8_t pldm_post_vr_update(void *fw_update_param);
static uint8_t pldm_pre_cpld_update(void *fw_update_param);

/* PLDM FW update table */
// clang-format off
pldm_fw_update_info_t PLDMUPDATE_FW_CONFIG_TABLE[] = {
	[COMP_ID_BIC] =  { ENABLE, COMP_CLASS_TYPE_DOWNSTREAM, COMP_ID_BIC, 0x00, NULL, pldm_bic_update, NULL, COMP_UPDATE_VIA_SPI, COMP_ACT_SELF, pldm_bic_activate },
	[COMP_ID_VR0] =  { ENABLE, COMP_CLASS_TYPE_DOWNSTREAM, COMP_ID_VR0, 0x00, pldm_pre_vr_update, pldm_vr_update, pldm_post_vr_update, COMP_UPDATE_VIA_I2C, COMP_ACT_AC_PWR_CYCLE, NULL },
	[COMP_ID_VR1] =  { ENABLE, COMP_CLASS_TYPE_DOWNSTREAM, COMP_ID_VR1, 0x00, pldm_pre_vr_update, pldm_vr_update, pldm_post_vr_update, COMP_UPDATE_VIA_I2C, COMP_ACT_AC_PWR_CYCLE, NULL },
	[COMP_ID_PEX0] = { ENABLE, COMP_CLASS_TYPE_DOWNSTREAM, COMP_ID_PEX0, 0x00, NULL, NULL, NULL, COMP_UPDATE_VIA_SPI, COMP_ACT_AC_PWR_CYCLE, NULL },
	[COMP_ID_PEX1] = { ENABLE, COMP_CLASS_TYPE_DOWNSTREAM, COMP_ID_PEX1, 0x00, NULL, NULL, NULL, COMP_UPDATE_VIA_SPI, COMP_ACT_AC_PWR_CYCLE, NULL },
	[COMP_ID_PEX2] = { ENABLE, COMP_CLASS_TYPE_DOWNSTREAM, COMP_ID_PEX2, 0x00, NULL, NULL, NULL, COMP_UPDATE_VIA_SPI, COMP_ACT_AC_PWR_CYCLE, NULL },
	[COMP_ID_PEX3] = { ENABLE, COMP_CLASS_TYPE_DOWNSTREAM, COMP_ID_PEX3, 0x00, NULL, NULL, NULL, COMP_UPDATE_VIA_SPI, COMP_ACT_AC_PWR_CYCLE, NULL },
	[COMP_ID_CPLD] = { ENABLE, COMP_CLASS_TYPE_DOWNSTREAM, COMP_ID_CPLD, 0x00, pldm_pre_cpld_update, pldm_cpld_update, NULL, COMP_UPDATE_VIA_I2C, COMP_ACT_AC_PWR_CYCLE, NULL },
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

/* pldm pre-update func */
static uint8_t pldm_pre_vr_update(void *fw_update_param)
{
	CHECK_NULL_ARG_WITH_RETURN(fw_update_param, 1);

	pldm_fw_update_param_t *p = (pldm_fw_update_param_t *)fw_update_param;

	/* Stop sensor polling */
	disable_sensor_poll();

	/* Assign VR 0/1 related sensor number to get information for accessing VR */
	uint8_t sensor_num =
		(p->comp_id == COMP_ID_VR0) ? SENSOR_NUM_TEMP_PEX_1 : SENSOR_NUM_TEMP_PEX_3;

	if (!tca9548_select_chan(sensor_num, &mux_conf_addr_0xe0[6])) {
		LOG_ERR("Component %d: mux switched failed!", p->comp_id);
		return 1;
	}

	/* Get bus and target address by sensor number in sensor configuration */
	p->bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	p->addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;

	return 0;
}

static uint8_t pldm_pre_cpld_update(void *fw_update_param)
{
	CHECK_NULL_ARG_WITH_RETURN(fw_update_param, 1);

	pldm_fw_update_param_t *p = (pldm_fw_update_param_t *)fw_update_param;

	if (p->inf == COMP_UPDATE_VIA_I2C) {
		p->bus = 0x07;
		p->addr = 0x40;
	}

	return 0;
}

/* pldm post-update func */
static uint8_t pldm_post_vr_update(void *fw_update_param)
{
	/* Stop sensor polling */
	enable_sensor_poll();

	return 0;
}
