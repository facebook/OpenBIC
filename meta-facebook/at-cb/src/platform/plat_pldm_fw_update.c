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
#include "util_spi.h"
#include "hal_jtag.h"
#include "sensor.h"
#include "pldm_firmware_update.h"
#include "plat_pldm_fw_update.h"
#include "plat_ipmi.h"
#include "plat_gpio.h"
#include "plat_i2c.h"
#include "plat_sensor_table.h"
#include "plat_hook.h"
#include "plat_class.h"
#include "lattice.h"
#include "plat_dev.h"
#include "util_sys.h"

LOG_MODULE_REGISTER(plat_fwupdate);

#define CPLD_BUS_5_ADDR 0x40
#define CPLD_USER_CODE_LENGTH 4

static uint8_t pldm_pre_cpld_update(void *fw_update_param);
static bool get_cpld_user_code(void *info_p, uint8_t *buf, uint8_t *len);
static bool get_vr_fw_version(void *info_p, uint8_t *buf, uint8_t *len);

/* PLDM FW update table */
pldm_fw_update_info_t PLDMUPDATE_FW_CONFIG_TABLE[] = {
	{
		.enable = true,
		.comp_classification = COMP_CLASS_TYPE_DOWNSTREAM,
		.comp_identifier = CB_COMPNT_BIC,
		.comp_classification_index = 0x00,
		.pre_update_func = NULL,
		.update_func = pldm_bic_update,
		.pos_update_func = NULL,
		.inf = COMP_UPDATE_VIA_SPI,
		.activate_method = COMP_ACT_SELF,
		.self_act_func = pldm_bic_activate,
		.get_fw_version_fn = NULL,
	},
	{
		.enable = true,
		.comp_classification = COMP_CLASS_TYPE_DOWNSTREAM,
		.comp_identifier = CB_COMPNT_CPLD,
		.comp_classification_index = 0x00,
		.pre_update_func = pldm_pre_cpld_update,
		.update_func = pldm_cpld_update,
		.pos_update_func = NULL,
		.inf = COMP_UPDATE_VIA_I2C,
		.activate_method = COMP_ACT_AC_PWR_CYCLE,
		.self_act_func = NULL,
		.get_fw_version_fn = get_cpld_user_code,
	},
	{
		.enable = true,
		.comp_classification = COMP_CLASS_TYPE_DOWNSTREAM,
		.comp_identifier = CB_COMPNT_VR_XDPE15284,
		.comp_classification_index = 0x00,
		.pre_update_func = NULL,
		.update_func = NULL,
		.pos_update_func = NULL,
		.inf = COMP_UPDATE_VIA_I2C,
		.activate_method = COMP_ACT_AC_PWR_CYCLE,
		.self_act_func = NULL,
		.get_fw_version_fn = get_vr_fw_version,
	},
};

static uint8_t pldm_pre_cpld_update(void *fw_update_param)
{
	CHECK_NULL_ARG_WITH_RETURN(fw_update_param, 1);

	pldm_fw_update_param_t *p = (pldm_fw_update_param_t *)fw_update_param;

	if (p->inf == COMP_UPDATE_VIA_I2C) {
		p->bus = I2C_BUS5;
		p->addr = CPLD_BUS_5_ADDR;
	}

	return 0;
}

static bool get_cpld_user_code(void *info_p, uint8_t *buf, uint8_t *len)
{
	CHECK_NULL_ARG_WITH_RETURN(buf, false);
	CHECK_NULL_ARG_WITH_RETURN(len, false);
	ARG_UNUSED(info_p);

	bool ret = false;
	uint8_t tmp_buf[4] = { 0 };
	uint32_t read_usrcode = 0;

	ret = cpld_i2c_get_usercode(I2C_BUS5, CPLD_BUS_5_ADDR, &read_usrcode);
	if (ret != true) {
		LOG_ERR("Fail to get CPLD usercode");
		return false;
	}

	memcpy(tmp_buf, &read_usrcode, CPLD_USER_CODE_LENGTH);
	*len = bin2hex(tmp_buf, 4, buf, 8);
	return true;
}

static bool get_vr_fw_version(void *info_p, uint8_t *buf, uint8_t *len)
{
	CHECK_NULL_ARG_WITH_RETURN(buf, false);
	CHECK_NULL_ARG_WITH_RETURN(len, false);
	ARG_UNUSED(info_p);

	if (cb_vr_fw_info.is_init == false) {
		LOG_ERR("VR firmware information not ready");
		return false;
	}

	uint8_t *buf_p = buf;
	uint8_t remain_write_p[3] = { 0 };
	uint8_t remain_write_p_count = 0;
	char *vr_vendor = NULL;
	const char *remain_str = ", Remaining Write: ";

	switch (cb_vr_fw_info.vendor) {
	case VENDOR_INFINEON:
		vr_vendor = "Infineon ";
		break;
	case VENDOR_MPS:
		vr_vendor = "MPS ";
		break;
	default:
		vr_vendor = "Unknown ";
		break;
	}

	*len = 0;
	memcpy(buf_p, vr_vendor, strlen(vr_vendor));
	buf_p += strlen(vr_vendor);

	*len += bin2hex(cb_vr_fw_info.checksum, 4, buf_p, 8) + strlen(vr_vendor);
	buf_p += 8;

	memcpy(buf_p, remain_str, strlen(remain_str));
	buf_p += strlen(remain_str);

	remain_write_p_count =
		uint8_t_to_dec_ascii_pointer(cb_vr_fw_info.remaining_write, remain_write_p, 3);
	memcpy(buf_p, remain_write_p, remain_write_p_count);
	*len += remain_write_p_count + strlen(remain_str);
	return true;
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
