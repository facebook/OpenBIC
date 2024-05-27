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
#include "i2c-mux-tca9548.h"
#include "lattice.h"

#include "pldm_firmware_update.h"
#include "plat_pldm_fw_update.h"
#include "plat_gpio.h"
#include "plat_i2c.h"
#include "plat_sensor_table.h"
#include "plat_class.h"
#include "power_status.h"

#include "mpq8746.h"
#include "mp289x.h"
#include "pt5161l.h"
#include "ds160pt801.h"

LOG_MODULE_REGISTER(plat_fwupdate);

static uint8_t pldm_pre_vr_update(void *fw_update_param);
static uint8_t pldm_post_vr_update(void *fw_update_param);
static bool get_vr_fw_version(void *info_p, uint8_t *buf, uint8_t *len);
static uint8_t pldm_pre_retimer_update(void *fw_update_param);
static uint8_t pldm_post_retimer_update(void *fw_update_param);
static bool get_retimer_fw_version(void *info_p, uint8_t *buf, uint8_t *len);

/* PLDM FW update table */
pldm_fw_update_info_t PLDMUPDATE_FW_CONFIG_TABLE[] = {
	{
		.enable = true,
		.comp_classification = COMP_CLASS_TYPE_DOWNSTREAM,
		.comp_identifier = JI_COMPNT_BIC,
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
		.comp_identifier = JI_COMPNT_CPUDVDD,
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
		.comp_identifier = JI_COMPNT_CPUVDD,
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
		.comp_identifier = JI_COMPNT_SOCVDD,
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
		.comp_identifier = JI_COMPNT_RETIMER,
		.comp_classification_index = 0x00,
		.pre_update_func = pldm_pre_retimer_update,
		.update_func = pldm_retimer_update,
		.pos_update_func = pldm_post_retimer_update,
		.inf = COMP_UPDATE_VIA_I2C,
		.activate_method = COMP_ACT_AC_PWR_CYCLE,
		.self_act_func = NULL,
		.get_fw_version_fn = get_retimer_fw_version,
		.self_apply_work_func = NULL,
		.comp_version_str = NULL,
	},
};

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

	uint8_t oth_module = get_oth_module();
	if (oth_module != OTH_MODULE_PRIMARY) {
		LOG_WRN("Given source %d is not supported yet", oth_module);
		return 1;
	}

	pldm_fw_update_param_t *p = (pldm_fw_update_param_t *)fw_update_param;

	/* Stop sensor polling */
	disable_sensor_poll();

	const uint8_t *device_name_p = NULL;
	const char *device_name[3][2] = {
		[0][OTH_MODULE_PRIMARY] = KEYWORD_VR_MPQ8746,
		[1][OTH_MODULE_PRIMARY] = KEYWORD_VR_MP2898,
		[2][OTH_MODULE_PRIMARY] = KEYWORD_VR_MP2894,
	};

	/* Get bus and target address by sensor number in sensor configuration */
	switch (p->comp_id) {
	case JI_COMPNT_CPUDVDD:
		device_name_p = device_name[0][oth_module];
		p->bus = CPUDVDD_I2C_BUS;
		p->addr = CPUDVDD_I2C_ADDR >> 1;
		break;

	case JI_COMPNT_CPUVDD:
		device_name_p = device_name[1][oth_module];
		p->bus = CPUVDD_I2C_BUS;
		p->addr = CPUVDD_I2C_ADDR >> 1;
		break;

	case JI_COMPNT_SOCVDD:
		device_name_p = device_name[2][oth_module];
		p->bus = SOCVDD_I2C_BUS;
		p->addr = SOCVDD_I2C_ADDR >> 1;
		break;

	default:
		LOG_ERR("Unsupport component ID %d while update VR device", p->comp_id);
		return 1;
	}

	/* Check whether fw image's vendor not mach with on board device */
	if (strncmp(p->comp_version_str, device_name_p, strlen(device_name_p) - 1)) {
		LOG_ERR("Given fw's device %s not mach with on board device %s",
			p->comp_version_str, device_name_p);
		return 1;
	}

	return 0;
}

/* pldm post-update func */
static uint8_t pldm_post_vr_update(void *fw_update_param)
{
	ARG_UNUSED(fw_update_param);

	/* Start sensor polling */
	enable_sensor_poll();

	return 0;
}

static bool get_vr_fw_version(void *info_p, uint8_t *buf, uint8_t *len)
{
	CHECK_NULL_ARG_WITH_RETURN(info_p, false);
	CHECK_NULL_ARG_WITH_RETURN(buf, false);
	CHECK_NULL_ARG_WITH_RETURN(len, false);

	pldm_fw_update_info_t *p = (pldm_fw_update_info_t *)info_p;

	if ((p->comp_identifier < JI_COMPNT_CPUDVDD) || (p->comp_identifier > JI_COMPNT_SOCVDD)) {
		LOG_ERR("Unsupport VR component ID(%d)", p->comp_identifier);
		return false;
	}

	bool ret = false;

	gpio_set(BIC_CPLD_VRD_MUX_SEL, GPIO_LOW);

	uint8_t version[15] = { 0 };
	uint16_t tmp_crc_16 = 0;
	uint16_t ver_len = 0;
	uint16_t remain = 0xFFFF;
	uint8_t bus = 0;
	uint8_t addr = 0;

	uint8_t source = get_oth_module();
	if (source != OTH_MODULE_PRIMARY) {
		LOG_WRN("Given source %d is not supported yet", source);
		goto post_hook_and_ret;
	}

	switch (p->comp_identifier) {
	case JI_COMPNT_CPUDVDD:
		bus = CPUDVDD_I2C_BUS;
		addr = CPUDVDD_I2C_ADDR >> 1;
		if (!mpq8746_get_checksum(bus, addr, &tmp_crc_16)) {
			LOG_ERR("Component %d version reading failed", p->comp_identifier);
			goto post_hook_and_ret;
		}
		tmp_crc_16 = sys_cpu_to_be16(tmp_crc_16);
		memcpy(version, &tmp_crc_16, sizeof(tmp_crc_16));
		ver_len = 2;
		break;
	case JI_COMPNT_CPUVDD:
	case JI_COMPNT_SOCVDD:
		if (p->comp_identifier == JI_COMPNT_CPUVDD) {
			bus = CPUVDD_I2C_BUS;
			addr = CPUVDD_I2C_ADDR >> 1;
		} else {
			bus = SOCVDD_I2C_BUS;
			addr = SOCVDD_I2C_ADDR >> 1;
		}
		if (!mp289x_crc_get(bus, addr, MODE_USR, &tmp_crc_16)) {
			LOG_ERR("Component %d version reading failed", p->comp_identifier);
			goto post_hook_and_ret;
		}
		tmp_crc_16 = sys_cpu_to_be16(tmp_crc_16);
		memcpy(version, &tmp_crc_16, sizeof(tmp_crc_16));
		ver_len += 2;

		if (!mp289x_crc_get(bus, addr, MODE_MULTI_CFG_12, &tmp_crc_16)) {
			LOG_ERR("Component %d version reading failed", p->comp_identifier);
			goto post_hook_and_ret;
		}
		tmp_crc_16 = sys_cpu_to_be16(tmp_crc_16);
		memcpy(&version[ver_len], &tmp_crc_16, sizeof(tmp_crc_16));
		ver_len += 2;

		if (!mp289x_crc_get(bus, addr, MODE_MULTI_CFG_34, &tmp_crc_16)) {
			LOG_ERR("Component %d version reading failed", p->comp_identifier);
			goto post_hook_and_ret;
		}
		tmp_crc_16 = sys_cpu_to_be16(tmp_crc_16);
		memcpy(&version[ver_len], &tmp_crc_16, sizeof(tmp_crc_16));
		ver_len += 2;

		if (!mp289x_crc_get(bus, addr, MODE_MULTI_CFG_56, &tmp_crc_16)) {
			LOG_ERR("Component %d version reading failed", p->comp_identifier);
			goto post_hook_and_ret;
		}
		tmp_crc_16 = sys_cpu_to_be16(tmp_crc_16);
		memcpy(&version[ver_len], &tmp_crc_16, sizeof(tmp_crc_16));
		ver_len += 2;
		break;
	default:
		LOG_ERR("Unsupport Component id(%d)", p->comp_identifier);
		goto post_hook_and_ret;
	}

	const char *remain_str_p = ", Remaining Write: ";
	uint8_t *buf_p = buf;
	const uint8_t *vr_name_p = "MPS ";
	*len = 0;

	if (!vr_name_p) {
		LOG_ERR("The pointer of VR string name is NULL");
		goto post_hook_and_ret;
	}

	memcpy(buf_p, vr_name_p, strlen(vr_name_p));
	buf_p += strlen(vr_name_p);

	*len += bin2hex((uint8_t *)&version, ver_len, buf_p, ver_len * 2) + strlen(vr_name_p);
	buf_p += ver_len * 2;

	if (remain != 0xFFFF) {
		memcpy(buf_p, remain_str_p, strlen(remain_str_p));
		buf_p += strlen(remain_str_p);
		remain = (uint8_t)((remain % 10) | (remain / 10 << 4));
		*len += bin2hex((uint8_t *)&remain, 1, buf_p, 2) + strlen(remain_str_p);
		buf_p += 2;
	}

	LOG_HEXDUMP_INF(buf, *len, "VR version string");

	ret = true;

post_hook_and_ret:
	gpio_set(BIC_CPLD_VRD_MUX_SEL, GPIO_HIGH);

	return ret;
}

static uint8_t pldm_pre_retimer_update(void *fw_update_param)
{
	CHECK_NULL_ARG_WITH_RETURN(fw_update_param, 1);

	if (get_board_revision() < SYS_BOARD_EVT) {
		LOG_WRN("Not support retimer relative function before EVT");
		return 1;
	}

	if (get_post_status() == false) {
		LOG_WRN("Not in POST COMPLETE state, skip retimer update");
		return 1;
	}

	/* Stop sensor polling */
	disable_sensor_poll();

	/* Need to switch channel after EVT2 */
	if (get_board_revision() >= SYS_BOARD_EVT2) {
		I2C_MSG msg = { 0 };
		msg.bus = I2C_BUS2;
		msg.target_addr = (0xE2 >> 1); //switch address
		msg.data[0] = 0x02; //channel2
		msg.tx_len = 1;
		msg.rx_len = 0;

		if (i2c_master_write(&msg, 3)) {
			LOG_ERR("Failed to switch channel for retimer");
			return 1;
		}
	}

	uint8_t retimer_module = get_retimer_module();
	pldm_fw_update_param_t *p = (pldm_fw_update_param_t *)fw_update_param;
	p->bus = I2C_BUS2;
	switch (retimer_module) {
	case RETIMER_MODULE_PT4080L:
		p->addr = AL_RETIMER_ADDR;
		break;
	case RETIMER_MODULE_DS160PT801:
		p->addr = TI_RETIMER_ADDR;
		break;
	default:
		LOG_ERR("Unsupport retimer module %d", retimer_module);
		return 1;
	}

	/* Check whether fw image's vendor not mach with on board device */
	const char *device_name[] = {
		[RETIMER_MODULE_PT4080L] = KEYWORD_RETIMER_PT4080L,
		[RETIMER_MODULE_DS160PT801] = KEYWORD_RETIMER_DS160PT801,
	};
	const uint8_t *device_name_p = device_name[retimer_module];

	if (strncmp(p->comp_version_str, device_name_p, strlen(device_name_p) - 1)) {
		LOG_ERR("Given fw's device %s not mach with on board device %s",
			p->comp_version_str, device_name_p);
		return 1;
	}

	return 0;
}

static uint8_t pldm_post_retimer_update(void *fw_update_param)
{
	ARG_UNUSED(fw_update_param);

	/* Start sensor polling */
	enable_sensor_poll();

	return 0;
}

static bool get_retimer_fw_version(void *info_p, uint8_t *buf, uint8_t *len)
{
	CHECK_NULL_ARG_WITH_RETURN(info_p, false);
	CHECK_NULL_ARG_WITH_RETURN(buf, false);
	CHECK_NULL_ARG_WITH_RETURN(len, false);

	if (get_board_revision() < SYS_BOARD_EVT) {
		LOG_WRN("Not support retimer relative function before EVT");
		return false;
	}
	if (get_post_status() == false) {
		LOG_WRN("Not in POST COMPLETE state, skip retimer fw version get");
		return false;
	}

	bool ret = false;

	/* Stop sensor polling */
	disable_sensor_poll();

	/* Need to switch channel after EVT2 */
	if (get_board_revision() >= SYS_BOARD_EVT2) {
		I2C_MSG msg = { 0 };
		msg.bus = I2C_BUS2;
		msg.target_addr = (0xE2 >> 1); //switch address
		msg.data[0] = 0x02; //channel2
		msg.tx_len = 1;
		msg.rx_len = 0;

		if (i2c_master_write(&msg, 3)) {
			LOG_ERR("Failed to switch channel for retimer");
			goto exit;
		}
	}

	uint8_t retimer_module = get_retimer_module();
	uint8_t version[10];
	uint16_t ver_len = 0;
	I2C_MSG i2c_msg;
	i2c_msg.bus = I2C_BUS2;
	switch (retimer_module) {
	case RETIMER_MODULE_PT4080L:
		i2c_msg.target_addr = AL_RETIMER_ADDR;
		if (pt5161l_get_fw_version(&i2c_msg, version) == false) {
			LOG_ERR("Failed to get PT4080L retimer version");
			goto exit;
		}
		ver_len = 3;
		break;
	case RETIMER_MODULE_DS160PT801:
		i2c_msg.target_addr = TI_RETIMER_ADDR;
		if (ds160pt801_get_fw_version(&i2c_msg, version) == false) {
			LOG_ERR("Failed to get DS160PT801 retimer version");
			goto exit;
		}
		ver_len = 1;
		break;
	default:
		LOG_ERR("Unsupport retimer module %d", retimer_module);
		goto exit;
	}

	const char *vr_name[] = {
		[RETIMER_MODULE_PT4080L] = "AL ",
		[RETIMER_MODULE_DS160PT801] = "TI ",
	};
	const uint8_t *vr_name_p = vr_name[retimer_module];

	if (!vr_name_p) {
		LOG_ERR("The pointer of VR string name is NULL");
		goto exit;
	}

	uint8_t *buf_p = buf;
	memcpy(buf_p, vr_name_p, strlen(vr_name_p));
	buf_p += strlen(vr_name_p);
	*len += strlen(vr_name_p);

	memcpy(buf_p, version, ver_len);
	*len += bin2hex(version, ver_len, buf_p, ver_len * 2);
	buf_p += (ver_len * 2);

	LOG_HEXDUMP_INF(buf, *len, "Retimer version string");

	ret = true;
exit:
	/* Start sensor polling */
	enable_sensor_poll();

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
