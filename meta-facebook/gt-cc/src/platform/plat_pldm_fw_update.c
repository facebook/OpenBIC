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
#include "pex89000.h"
#include "isl69259.h"
#include "xdpe12284c.h"
#include "mp2971.h"

#include "pldm_firmware_update.h"
#include "plat_pldm_fw_update.h"
#include "plat_pldm_monitor.h"
#include "plat_gpio.h"
#include "plat_i2c.h"
#include "plat_sensor_table.h"
#include "plat_hook.h"
#include "plat_class.h"

LOG_MODULE_REGISTER(plat_fwupdate);

static uint8_t pldm_pre_vr_update(void *fw_update_param);
static uint8_t pldm_post_vr_update(void *fw_update_param);
static uint8_t pldm_pre_cpld_update(void *fw_update_param);
static uint8_t pldm_pre_pex_update(void *fw_update_param);
static uint8_t pldm_pex_update(void *fw_update_param);
static uint8_t pldm_post_pex_update(void *fw_update_param);
static bool get_fpga_user_code(void *info_p, uint8_t *buf, uint8_t *len);
static bool get_pex_fw_version(void *info_p, uint8_t *buf, uint8_t *len);
static bool get_vr_fw_version(void *info_p, uint8_t *buf, uint8_t *len);

/* PLDM FW update table */
pldm_fw_update_info_t PLDMUPDATE_FW_CONFIG_TABLE[] = {
	{
		.enable = true,
		.comp_classification = COMP_CLASS_TYPE_DOWNSTREAM,
		.comp_identifier = GT_COMPNT_BIC,
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
		.comp_identifier = GT_COMPNT_VR0,
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
		.comp_identifier = GT_COMPNT_VR1,
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
		.comp_identifier = GT_COMPNT_PEX0,
		.comp_classification_index = 0x00,
		.pre_update_func = pldm_pre_pex_update,
		.update_func = pldm_pex_update,
		.pos_update_func = pldm_post_pex_update,
		.inf = COMP_UPDATE_VIA_SPI,
		.activate_method = COMP_ACT_DC_PWR_CYCLE,
		.self_act_func = NULL,
		.get_fw_version_fn = get_pex_fw_version,
		.self_apply_work_func = NULL,
		.comp_version_str = NULL,
	},
	{
		.enable = true,
		.comp_classification = COMP_CLASS_TYPE_DOWNSTREAM,
		.comp_identifier = GT_COMPNT_PEX1,
		.comp_classification_index = 0x00,
		.pre_update_func = pldm_pre_pex_update,
		.update_func = pldm_pex_update,
		.pos_update_func = pldm_post_pex_update,
		.inf = COMP_UPDATE_VIA_SPI,
		.activate_method = COMP_ACT_DC_PWR_CYCLE,
		.self_act_func = NULL,
		.get_fw_version_fn = get_pex_fw_version,
		.self_apply_work_func = NULL,
		.comp_version_str = NULL,
	},
	{
		.enable = true,
		.comp_classification = COMP_CLASS_TYPE_DOWNSTREAM,
		.comp_identifier = GT_COMPNT_PEX2,
		.comp_classification_index = 0x00,
		.pre_update_func = pldm_pre_pex_update,
		.update_func = pldm_pex_update,
		.pos_update_func = pldm_post_pex_update,
		.inf = COMP_UPDATE_VIA_SPI,
		.activate_method = COMP_ACT_DC_PWR_CYCLE,
		.self_act_func = NULL,
		.get_fw_version_fn = get_pex_fw_version,
		.self_apply_work_func = NULL,
		.comp_version_str = NULL,
	},
	{
		.enable = true,
		.comp_classification = COMP_CLASS_TYPE_DOWNSTREAM,
		.comp_identifier = GT_COMPNT_PEX3,
		.comp_classification_index = 0x00,
		.pre_update_func = pldm_pre_pex_update,
		.update_func = pldm_pex_update,
		.pos_update_func = pldm_post_pex_update,
		.inf = COMP_UPDATE_VIA_SPI,
		.activate_method = COMP_ACT_DC_PWR_CYCLE,
		.self_act_func = NULL,
		.get_fw_version_fn = get_pex_fw_version,
		.self_apply_work_func = NULL,
		.comp_version_str = NULL,
	},
	{
		.enable = true,
		.comp_classification = COMP_CLASS_TYPE_DOWNSTREAM,
		.comp_identifier = GT_COMPNT_CPLD,
		.comp_classification_index = 0x00,
		.pre_update_func = pldm_pre_cpld_update,
		.update_func = pldm_cpld_update,
		.pos_update_func = NULL,
		.inf = COMP_UPDATE_VIA_I2C,
		.activate_method = COMP_ACT_AC_PWR_CYCLE,
		.self_act_func = NULL,
		.get_fw_version_fn = get_fpga_user_code,
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

	pldm_fw_update_param_t *p = (pldm_fw_update_param_t *)fw_update_param;

	/* Stop sensor polling */
	disable_sensor_poll();

	/* Assign VR 0/1 related sensor number to get information for accessing VR */
	uint8_t sensor_num =
		(p->comp_id == GT_COMPNT_VR0) ? SENSOR_NUM_PEX_0_VR_TEMP : SENSOR_NUM_PEX_2_VR_TEMP;
	sensor_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];

	if (!tca9548_select_chan(cfg, &mux_conf_addr_0xe0[6])) {
		LOG_ERR("Component %d: mux switched failed!", p->comp_id);
		return 1;
	}

	/* Get bus and target address by sensor number in sensor configuration */
	p->bus = cfg->port;
	p->addr = cfg->target_addr;

	return 0;
}

static uint8_t pldm_pre_cpld_update(void *fw_update_param)
{
	CHECK_NULL_ARG_WITH_RETURN(fw_update_param, 1);

	pldm_fw_update_param_t *p = (pldm_fw_update_param_t *)fw_update_param;

	if (p->inf == COMP_UPDATE_VIA_I2C) {
		p->bus = I2C_BUS8;
		p->addr = 0x40;
	}

	return 0;
}

uint8_t flash_sel_pin[4] = { BIC_SEL_FLASH_SW0, BIC_SEL_FLASH_SW1, BIC_SEL_FLASH_SW2,
			     BIC_SEL_FLASH_SW3 };
uint8_t current_sel_pin = 0xFF;

static uint8_t pldm_pre_pex_update(void *fw_update_param)
{
	CHECK_NULL_ARG_WITH_RETURN(fw_update_param, 1);

	pldm_fw_update_param_t *p = (pldm_fw_update_param_t *)fw_update_param;

	uint8_t flash_sel_base = flash_sel_pin[0] - GT_COMPNT_PEX0;

	/* change mux to pex flash */
	for (int i = 0; i < ARRAY_SIZE(flash_sel_pin); i++) {
		if (flash_sel_base + p->comp_id == flash_sel_pin[i]) {
			current_sel_pin = flash_sel_pin[i];
			gpio_set(current_sel_pin, GPIO_HIGH);
		} else {
			gpio_set(flash_sel_pin[i], GPIO_LOW);
		}
	}

	return 0;
}

/* pldm fw-update func */
static uint8_t pldm_pex_update(void *fw_update_param)
{
	CHECK_NULL_ARG_WITH_RETURN(fw_update_param, 1);

	pldm_fw_update_param_t *p = (pldm_fw_update_param_t *)fw_update_param;

	CHECK_NULL_ARG_WITH_RETURN(p->data, 1);

	uint8_t update_flag = 0;

	/* prepare next data offset and length */
	p->next_ofs = p->data_ofs + p->data_len;
	p->next_len = fw_update_cfg.max_buff_size;

	if (p->next_ofs < fw_update_cfg.image_size) {
		if (p->next_ofs + p->next_len > fw_update_cfg.image_size)
			p->next_len = fw_update_cfg.image_size - p->next_ofs;

		if (((p->next_ofs % SECTOR_SZ_64K) + p->next_len) > SECTOR_SZ_64K)
			p->next_len = SECTOR_SZ_64K - (p->next_ofs % SECTOR_SZ_64K);
	} else {
		/* current data is the last packet
		 * set the next data length to 0 to inform the update completely
		 */
		p->next_len = 0;
		update_flag = (SECTOR_END_FLAG | NO_RESET_FLAG);
	}

	uint8_t ret = fw_update(p->data_ofs, p->data_len, p->data, update_flag, DEVSPI_SPI1_CS0);

	if (ret) {
		LOG_ERR("Firmware update failed, offset(0x%x), length(0x%x), status(%d)",
			p->data_ofs, p->data_len, ret);
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

static uint8_t pldm_post_pex_update(void *fw_update_param)
{
	ARG_UNUSED(fw_update_param);

	/* change mux to pex flash */
	if (current_sel_pin != 0xFF) {
		/* change mux to default */
		gpio_set(current_sel_pin, GPIO_LOW);
	}

	return 0;
}

static bool get_fpga_user_code(void *info_p, uint8_t *buf, uint8_t *len)
{
	CHECK_NULL_ARG_WITH_RETURN(buf, false);
	CHECK_NULL_ARG_WITH_RETURN(len, false);
	ARG_UNUSED(info_p);

	uint8_t tmp_buf[4] = { 0 };
	uint8_t ir_value = 0xc0;
	uint8_t dr_value = 0x00;
	const struct device *jtag_dev;

	jtag_dev = device_get_binding("JTAG0");

	if (!jtag_dev) {
		LOG_ERR("JTAG device not found");
		return false;
	}

	gpio_set(JTAG_BIC_EN, GPIO_HIGH);

	if (jtag_tap_set(jtag_dev, TAP_RESET))
		return false;

	k_msleep(10);

	if (jtag_ir_scan(jtag_dev, 8, &ir_value, tmp_buf, TAP_IDLE) ||
	    jtag_dr_scan(jtag_dev, 32, &dr_value, tmp_buf, TAP_IDLE))
		return false;

	gpio_set(JTAG_BIC_EN, GPIO_LOW);

	*(int *)tmp_buf = sys_cpu_to_be32(*(int *)&tmp_buf);
	*len = bin2hex(tmp_buf, 4, buf, 8);

	return true;
}

#define PLDM_PLAT_ERR_CODE_NO_POWER_ON 8
static bool get_pex_fw_version(void *info_p, uint8_t *buf, uint8_t *len)
{
	CHECK_NULL_ARG_WITH_RETURN(buf, false);
	CHECK_NULL_ARG_WITH_RETURN(len, false);
	CHECK_NULL_ARG_WITH_RETURN(info_p, false);

	pldm_fw_update_info_t *p = (pldm_fw_update_info_t *)info_p;

	if ((p->comp_identifier < GT_COMPNT_PEX0) || (p->comp_identifier > GT_COMPNT_PEX3)) {
		LOG_ERR("Unsupport PEX component ID(%d)", p->comp_identifier);
		return false;
	}

	/* Only can be read when DC is on */
	if (!is_mb_dc_on()) {
		uint8_t dc_off_error_code[] =
			PLDM_CREATE_ERR_STR_ARRAY(PLDM_PLAT_ERR_CODE_NO_POWER_ON);
		memcpy(buf, dc_off_error_code, sizeof(dc_off_error_code));
		*len = sizeof(dc_off_error_code);
		return true;
	}
	/* Physical Layer User Test Patterns, Byte 0 Register */
	int reading = 0x6080020c;
	uint8_t sensor_idx = p->comp_identifier - GT_COMPNT_PEX0;
	uint8_t pex_sensor_num = pex_sensor_num_table[sensor_idx];
	sensor_cfg *cfg = &sensor_config[sensor_config_index_map[pex_sensor_num]];

	if (!cfg) {
		LOG_ERR("The pointer of the sensor configuration is NULL");
		return false;
	}

	uint8_t unit_idx = ((pex89000_unit *)(cfg->priv_data))->idx;

	if (pex_access_engine(cfg->port, cfg->target_addr, unit_idx, pex_access_register,
			      &reading)) {
		return false;
	}

	/* Change version register to SBR because the old PEX firmware did not fill in version information at register 0x6080020c yet */
	if (((reading & 0xFF) == sensor_idx) && ((reading >> 8) & 0xFF) == 0xCC) {
		if (pex_access_engine(cfg->port, cfg->target_addr, unit_idx, pex_access_sbr_ver,
				      &reading)) {
			return false;
		}
	}

	uint8_t tmp_buf[4] = { 0 };
	uint8_t idx = 0;
	uint8_t i;

	memcpy(tmp_buf, &reading, sizeof(reading));
	*(int *)tmp_buf = sys_cpu_to_be32(*(int *)&tmp_buf);

	for (i = 0; i < ARRAY_SIZE(tmp_buf) - 1; i++) {
		idx += bin2hex(&tmp_buf[i], 1, &buf[idx], 2);
		buf[idx++] = '.';
	}

	idx += bin2hex(&tmp_buf[i], 1, &buf[idx], 2);

	*len = idx;

	return true;
}

static bool get_vr_fw_version(void *info_p, uint8_t *buf, uint8_t *len)
{
	CHECK_NULL_ARG_WITH_RETURN(info_p, false);
	CHECK_NULL_ARG_WITH_RETURN(buf, false);
	CHECK_NULL_ARG_WITH_RETURN(len, false);

	pldm_fw_update_info_t *p = (pldm_fw_update_info_t *)info_p;

	if ((p->comp_identifier != GT_COMPNT_VR0) && (p->comp_identifier != GT_COMPNT_VR1)) {
		LOG_ERR("Unsupport VR component ID(%d)", p->comp_identifier);
		return false;
	}

	bool ret = false;
	uint8_t sensor_num = ((p->comp_identifier == GT_COMPNT_VR0) ? SENSOR_NUM_PEX_0_VR_TEMP :
								      SENSOR_NUM_PEX_2_VR_TEMP);
	sensor_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];

	if (!cfg) {
		LOG_ERR("The pointer of the sensor configuration is NULL");
		return ret;
	}

	if (cfg->pre_sensor_read_hook) {
		if (!cfg->pre_sensor_read_hook(cfg, cfg->pre_sensor_read_args)) {
			LOG_ERR("The VR%d pre-reading hook function failed",
				(sensor_num == SENSOR_NUM_PEX_0_VR_TEMP) ? 0 : 1);
			goto post_hook_and_ret;
		}
	}

	uint8_t type = get_vr_type();
	uint32_t version;
	uint16_t remain = 0xFFFF;
	switch (type) {
	case VR_RNS_ISL69259: {
		uint8_t mode;

		if (!isl69259_get_raa_hex_mode(cfg->port, cfg->target_addr, &mode)) {
			LOG_ERR("Get VR ISL69259 raa hex mode failed");
			goto post_hook_and_ret;
		}

		if (!isl69259_get_raa_crc(cfg->port, cfg->target_addr, mode, &version)) {
			LOG_ERR("The VR ISL69259 version reading failed");
			goto post_hook_and_ret;
		}

		if (!get_raa_remaining_wr(cfg->port, cfg->target_addr, mode, &remain)) {
			LOG_ERR("The VR ISL69259 remaining reading failed");
			goto post_hook_and_ret;
		}
		break;
	}
	case VR_INF_XDPE12284:
		if (!xdpe12284c_get_checksum(cfg->port, cfg->target_addr, (uint8_t *)&version)) {
			LOG_ERR("The VR XDPE12284 version reading failed");
			goto post_hook_and_ret;
		}

		if (!xdpe12284c_get_remaining_write(cfg->port, cfg->target_addr, &remain)) {
			LOG_ERR("The VR XDPE12284 remaining reading failed");
			goto post_hook_and_ret;
		}
		break;
	case VR_MPS_MPS2971:
		if (!mp2971_get_checksum(cfg->port, cfg->target_addr, &version)) {
			LOG_ERR("The VR MPS2971 version reading failed");
			goto post_hook_and_ret;
		}
		break;
	default:
		LOG_ERR("Unsupport VR type(%d)", type);
		goto post_hook_and_ret;
	}

	if (type != VR_INF_XDPE12284)
		version = sys_cpu_to_be32(version);

	const char *vr_name[] = {
		[VR_RNS_ISL69259] = "Renesas ",
		[VR_INF_XDPE12284] = "Infineon ",
		[VR_MPS_MPS2971] = "MPS ",
	};

	const char *remain_str_p = ", Remaining Write: ";
	uint8_t *buf_p = buf;
	const uint8_t *vr_name_p = vr_name[type];
	*len = 0;

	if (!vr_name_p) {
		LOG_ERR("The pointer of VR string name is NULL");
		goto post_hook_and_ret;
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

post_hook_and_ret:
	if (cfg->post_sensor_read_hook) {
		if (!cfg->post_sensor_read_hook(cfg, cfg->post_sensor_read_args, NULL)) {
			LOG_ERR("The VR%d post-reading hook function failed",
				(sensor_num == SENSOR_NUM_PEX_0_VR_TEMP) ? 0 : 1);
		}
	}

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
