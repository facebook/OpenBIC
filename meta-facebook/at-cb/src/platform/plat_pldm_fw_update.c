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
#include "pex89000.h"
#include "plat_fru.h"
#include "mp2985.h"
#include "plat_pldm_device_identifier.h"

LOG_MODULE_REGISTER(plat_fwupdate);

#define CPLD_BUS_5_ADDR 0x40
#define CPLD_USER_CODE_LENGTH 4
#define PLDM_DOWNSTREAM_START_FLAG 1
#define PLDM_PROGESS_PERCENT_DEFAULT 0
#define PLDM_PROGESS_COMPLETE 100
#define DISABLE_SENSOR_POLLING_DELAY_MS 10

static uint8_t pldm_pre_cpld_update(void *fw_update_param);
static bool get_cpld_user_code(void *info_p, uint8_t *buf, uint8_t *len);
static bool get_vr_fw_version(void *info_p, uint8_t *buf, uint8_t *len);
static uint8_t pldm_pre_pex_update(void *fw_update_param);
static uint8_t pldm_pex_update(void *fw_update_param);
static uint8_t pldm_post_pex_update(void *fw_update_param);
static bool get_pex_fw_version(void *info_p, uint8_t *buf, uint8_t *len);
static uint8_t pldm_pre_vr_update(void *fw_update_param);
static uint8_t pldm_post_vr_update(void *fw_update_param);
static uint8_t plat_pldm_vr_update(void *fw_update_param);
static uint8_t pldm_pre_atm_update(void *fw_update_param);
static uint8_t pldm_pre_boot1_update(void *fw_update_param);
static uint8_t pldm_post_atm_update(void *fw_update_param);
static uint8_t pldm_atm_update(void *fw_update_param);
static uint8_t pldm_atm_apply_work();
static bool get_atm_fw_version(void *info_p, uint8_t *buf, uint8_t *len);

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
		.self_apply_work_func = NULL,
		.comp_version_str = NULL,

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
		.self_apply_work_func = NULL,
		.comp_version_str = NULL,
	},
	{
		.enable = true,
		.comp_classification = COMP_CLASS_TYPE_DOWNSTREAM,
		.comp_identifier = CB_COMPNT_VR_XDPE15284,
		.comp_classification_index = 0x00,
		.pre_update_func = pldm_pre_vr_update,
		.update_func = plat_pldm_vr_update,
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
		.comp_identifier = CB_COMPNT_PCIE_SWITCH0,
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
		.comp_identifier = CB_COMPNT_PCIE_SWITCH1,
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
		.comp_identifier = CB_COMPNT_ACCL1_CH1_FREYA,
		.comp_classification_index = 0x00,
		.pre_update_func = pldm_pre_atm_update,
		.update_func = pldm_atm_update,
		.pos_update_func = pldm_post_atm_update,
		.inf = COMP_UPDATE_VIA_I2C,
		.activate_method = COMP_ACT_AC_PWR_CYCLE,
		.self_act_func = NULL,
		.get_fw_version_fn = get_atm_fw_version,
		.self_apply_work_func = pldm_atm_apply_work,
		.comp_version_str = "psoc",
	},
	{
		.enable = true,
		.comp_classification = COMP_CLASS_TYPE_DOWNSTREAM,
		.comp_identifier = CB_COMPNT_ACCL1_CH1_FREYA,
		.comp_classification_index = 0x00,
		.pre_update_func = pldm_pre_atm_update,
		.update_func = pldm_atm_update,
		.pos_update_func = pldm_post_atm_update,
		.inf = COMP_UPDATE_VIA_I2C,
		.activate_method = COMP_ACT_AC_PWR_CYCLE,
		.self_act_func = NULL,
		.get_fw_version_fn = get_atm_fw_version,
		.self_apply_work_func = pldm_atm_apply_work,
		.comp_version_str = "qspi",
	},
	{
		.enable = true,
		.comp_classification = COMP_CLASS_TYPE_DOWNSTREAM,
		.comp_identifier = CB_COMPNT_ACCL1_CH1_FREYA,
		.comp_classification_index = 0x00,
		.pre_update_func = pldm_pre_boot1_update,
		.update_func = pldm_atm_update,
		.pos_update_func = pldm_post_atm_update,
		.inf = COMP_UPDATE_VIA_I2C,
		.activate_method = COMP_ACT_AC_PWR_CYCLE,
		.self_act_func = NULL,
		.get_fw_version_fn = get_atm_fw_version,
		.self_apply_work_func = pldm_atm_apply_work,
		.comp_version_str = "boot1",
	},
};

static uint8_t pldm_pre_cpld_update(void *fw_update_param)
{
	CHECK_NULL_ARG_WITH_RETURN(fw_update_param, PLDM_FW_UPDATE_ERROR);

	pldm_fw_update_param_t *p = (pldm_fw_update_param_t *)fw_update_param;

	if (p->inf == COMP_UPDATE_VIA_I2C) {
		p->bus = I2C_BUS5;
		p->addr = CPLD_BUS_5_ADDR;
	}

	return PLDM_FW_UPDATE_SUCCESS;
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

static uint8_t pldm_pre_pex_update(void *fw_update_param)
{
	CHECK_NULL_ARG_WITH_RETURN(fw_update_param, PLDM_FW_UPDATE_ERROR);

	if (is_acb_power_good() == false) {
		LOG_WRN("Can't update switch firmware because ACB dc off");
		return PLDM_FW_UPDATE_ERROR;
	}

	pldm_fw_update_param_t *p = (pldm_fw_update_param_t *)fw_update_param;
	uint8_t pex_id = p->comp_id - CB_COMPNT_PCIE_SWITCH0;

	gpio_set(pcie_switch_mux_info[pex_id].control_gpio,
		 pcie_switch_mux_info[pex_id].bic_to_flash_value);
	return PLDM_FW_UPDATE_SUCCESS;
}

static uint8_t pldm_pex_update(void *fw_update_param)
{
	CHECK_NULL_ARG_WITH_RETURN(fw_update_param, PLDM_FW_UPDATE_ERROR);

	pldm_fw_update_param_t *p = (pldm_fw_update_param_t *)fw_update_param;

	CHECK_NULL_ARG_WITH_RETURN(p->data, PLDM_FW_UPDATE_ERROR);

	uint8_t update_flag = 0;
	uint8_t pex_id = p->comp_id - CB_COMPNT_PCIE_SWITCH0;

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

	uint8_t ret = fw_update(p->data_ofs, p->data_len, p->data, update_flag,
				pcie_switch_mux_info[pex_id].device);
	CHECK_PLDM_FW_UPDATE_RESULT_WITH_RETURN(p->comp_id, p->data_ofs, p->data_len, ret,
						PLDM_FW_UPDATE_ERROR);

	return PLDM_FW_UPDATE_SUCCESS;
}

static uint8_t pldm_post_pex_update(void *fw_update_param)
{
	CHECK_NULL_ARG_WITH_RETURN(fw_update_param, PLDM_FW_UPDATE_ERROR);

	pldm_fw_update_param_t *p = (pldm_fw_update_param_t *)fw_update_param;
	uint8_t pex_id = p->comp_id - CB_COMPNT_PCIE_SWITCH0;

	gpio_set(pcie_switch_mux_info[pex_id].control_gpio,
		 pcie_switch_mux_info[pex_id].sw_to_flash_value);
	return PLDM_FW_UPDATE_SUCCESS;
}

#define PLDM_PLAT_ERR_CODE_NO_POWER_ON 8
static bool get_pex_fw_version(void *info_p, uint8_t *buf, uint8_t *len)
{
	CHECK_NULL_ARG_WITH_RETURN(buf, false);
	CHECK_NULL_ARG_WITH_RETURN(len, false);
	CHECK_NULL_ARG_WITH_RETURN(info_p, false);

	pldm_fw_update_info_t *p = (pldm_fw_update_info_t *)info_p;

	if ((p->comp_identifier != CB_COMPNT_PCIE_SWITCH0) &&
	    (p->comp_identifier != CB_COMPNT_PCIE_SWITCH1)) {
		LOG_ERR("Unsupport PEX component ID(%d)", p->comp_identifier);
		return false;
	}

	/* Only can be read when DC is on */
	if (is_acb_power_good() == false) {
		uint8_t dc_off_error_code[] =
			PLDM_CREATE_ERR_STR_ARRAY(PLDM_PLAT_ERR_CODE_NO_POWER_ON);
		memcpy(buf, dc_off_error_code, sizeof(dc_off_error_code));
		*len = sizeof(dc_off_error_code);
		return true;
	}
	int reading = 0;
	uint8_t pex_id = p->comp_identifier - CB_COMPNT_PCIE_SWITCH0;
	const uint8_t pex_sensor_num_table[PEX_MAX_NUMBER] = { SENSOR_NUM_TEMP_PEX_0,
							       SENSOR_NUM_TEMP_PEX_1 };
	uint8_t pex_sensor_num = pex_sensor_num_table[pex_id];

	sensor_cfg *cfg = &sensor_config[sensor_config_index_map[pex_sensor_num]];
	if (cfg == NULL) {
		LOG_ERR("Switch temperature sensor config is NULL, sensor num: 0x%x",
			pex_sensor_num);
		return false;
	}

	if (cfg->pre_sensor_read_hook) {
		if (cfg->pre_sensor_read_hook(cfg, cfg->pre_sensor_read_args) == false) {
			LOG_ERR("PEX%d pre-read fail", pex_id);
			return false;
		}
	}
	uint8_t unit_idx = ((pex89000_unit *)(cfg->priv_data))->idx;

	if (pex_access_engine(cfg->port, cfg->target_addr, unit_idx, pex_access_sbr_ver,
			      &reading)) {
		if (cfg->post_sensor_read_hook) {
			if (cfg->post_sensor_read_hook(cfg, cfg->post_sensor_read_args, NULL) ==
			    false) {
				LOG_ERR("PEX%d post-read fail after reading firmware version fail",
					pex_id);
			}
		}
		return false;
	}

	if (cfg->post_sensor_read_hook) {
		if (cfg->post_sensor_read_hook(cfg, cfg->post_sensor_read_args, NULL) == false) {
			LOG_ERR("PEX%d post-read fail", pex_id);
		}
	}

	uint8_t tmp_buf[4] = { 0 };
	uint8_t idx = 0;

	memcpy(tmp_buf, &reading, sizeof(reading));
	reverse_array(tmp_buf, sizeof(reading));

	idx += bin2hex(&tmp_buf[0], 1, &buf[idx], 2);
	buf[idx++] = '.';
	idx += bin2hex(&tmp_buf[1], 1, &buf[idx], 2);
	buf[idx++] = '.';
	idx += bin2hex(&tmp_buf[3], 1, &buf[idx], 2);
	//Append the config id into the version string
	buf[idx++] = '-';
	idx += bin2hex(&tmp_buf[2], 1, &buf[idx], 2);
	*len = idx;

	return true;
}

static uint8_t pldm_pre_vr_update(void *fw_update_param)
{
	CHECK_NULL_ARG_WITH_RETURN(fw_update_param, PLDM_FW_UPDATE_ERROR);

	pldm_fw_update_param_t *p = (pldm_fw_update_param_t *)fw_update_param;

	/* Stop sensor polling */
	disable_sensor_poll();

	p->bus = I2C_BUS1;
	p->addr = XDPE15284D_ADDR;

	return 0;
}

static uint8_t plat_pldm_vr_update(void *fw_update_param)
{
	CHECK_NULL_ARG_WITH_RETURN(fw_update_param, PLDM_FW_UPDATE_ERROR);

	pldm_fw_update_param_t *p = (pldm_fw_update_param_t *)fw_update_param;

	uint8_t count = 0;
	uint8_t update_result = 0;
	uint8_t vr_module = get_vr_module();

	switch (vr_module) {
	case VR_XDPE15284D:
		if (strncmp(p->comp_version_str, KEYWORD_VR_XDPE15284,
			    ARRAY_SIZE(KEYWORD_VR_XDPE15284) - 1) != 0) {
			return PLDM_FW_UPDATE_ERROR;
		}
		break;
	case VR_MP2985H:
		if (strncmp(p->comp_version_str, KEYWORD_VR_MP2985,
			    ARRAY_SIZE(KEYWORD_VR_MP2985) - 1) != 0) {
			return PLDM_FW_UPDATE_ERROR;
		}

		if (cb_vr_fw_info.is_init) {
			count = cb_vr_fw_info.remaining_write;
		} else {
			if (get_mp2985_remaining_write(&count) != true) {
				LOG_ERR("Fail to get mp2985 remaining write");
				return PLDM_FW_UPDATE_ERROR;
			}
		}

		if (count == 0) {
			LOG_ERR("MP2985 have insufficient remaining writes");
			return PLDM_FW_UPDATE_ERROR;
		}

		int ret = -1;
		int retry = 5;
		uint8_t offset = CPLD_NORMAL_ENABLE_OFFSET;

		I2C_MSG msg = construct_i2c_message(I2C_BUS3, CPLD_ADDR, 1, &offset, 1);
		ret = i2c_master_read(&msg, retry);
		if (ret != 0) {
			LOG_ERR("Fail to get cpld normal enable register");
			return PLDM_FW_UPDATE_ERROR;
		}

		if ((msg.data[0] & CPLD_P0V8_1_EN_BIT) == 0) {
			ret = mp2985_set_power_regular_mode(p->bus, p->addr);
			if (ret != 0) {
				LOG_ERR("Fail to set power regular mode");
				return PLDM_FW_UPDATE_ERROR;
			}
		}
		break;
	default:
		LOG_ERR("Unknown VR module: 0x%x", vr_module);
		return PLDM_FW_UPDATE_ERROR;
	}

	update_result = pldm_vr_update(fw_update_param);
	if (update_result != PLDM_FW_UPDATE_ERROR) {
		if (set_mp2985_remaining_write(count - 1) != true) {
			LOG_ERR("Fail to set mp2985 remaining write to 0x%x", count - 1);
		}
	}

	return update_result;
}

static uint8_t pldm_post_vr_update(void *fw_update_param)
{
	ARG_UNUSED(fw_update_param);

	/* Start sensor polling */
	enable_sensor_poll();

	return PLDM_FW_UPDATE_SUCCESS;
}

static bool get_atm_fw_version(void *info_p, uint8_t *buf, uint8_t *len)
{
	CHECK_NULL_ARG_WITH_RETURN(buf, false);
	CHECK_NULL_ARG_WITH_RETURN(len, false);
	CHECK_NULL_ARG_WITH_RETURN(info_p, false);

	pldm_fw_update_info_t *p = (pldm_fw_update_info_t *)info_p;

	if ((p->comp_identifier < CB_COMPNT_ACCL1_CH1_FREYA) ||
	    (p->comp_identifier > CB_COMPNT_ACCL12_CH2_FREYA)) {
		LOG_ERR("Invalid Artemis module id: 0x%x", p->comp_identifier);
		return false;
	}

	/* Only can be read when DC is on */
	if (is_acb_power_good() == false) {
		*len = strlen("NA (NVME NOT READY)");
		memcpy(buf, "NA (NVME NOT READY)", strlen("NA (NVME NOT READY)"));
		return true;
	}

	uint8_t card_id = (p->comp_identifier - CB_COMPNT_ACCL1_CH1_FREYA) / 2;
	uint8_t dev_id = (p->comp_identifier - CB_COMPNT_ACCL1_CH1_FREYA) % 2;

	if (dev_id == FREYA_ID1) {
		if (asic_card_info[card_id].asic_1_status != ASIC_CARD_DEVICE_PRESENT) {
			*len = strlen("NA (NVME NOT PRESENT)");
			memcpy(buf, "NA (NVME NOT PRESENT)", strlen("NA (NVME NOT PRESENT)"));
			return true;
		}

		*len += bin2hex((uint8_t *)&accl_freya_info[card_id].freya1_fw_info,
				sizeof(freya_fw_info), buf, 2 * sizeof(freya_fw_info));
		return true;
	} else {
		if (asic_card_info[card_id].asic_2_status != ASIC_CARD_DEVICE_PRESENT) {
			*len = strlen("NA (NVME NOT PRESENT)");
			memcpy(buf, "NA (NVME NOT PRESENT)", strlen("NA (NVME NOT PRESENT)"));
			return true;
		}

		*len += bin2hex((uint8_t *)&accl_freya_info[card_id].freya2_fw_info,
				sizeof(freya_fw_info), buf, 2 * sizeof(freya_fw_info));
		return true;
	}
}

static uint8_t pldm_pre_boot1_update(void *fw_update_param)
{
	CHECK_NULL_ARG_WITH_RETURN(fw_update_param, PLDM_FW_UPDATE_ERROR);

	pldm_fw_update_param_t *p = (pldm_fw_update_param_t *)fw_update_param;
	uint8_t card_id = (p->comp_id - CB_COMPNT_ACCL1_CH1_FREYA) / 2;
	uint8_t device_id = ((p->comp_id - CB_COMPNT_ACCL1_CH1_FREYA) % 2);

	if (is_time_to_poll_card_sensor(card_id) != true) {
		LOG_ERR("Artemis module power status not ready, card id: 0x%x", card_id);
		return PLDM_FW_UPDATE_ERROR;
	}

	if (device_id == PCIE_DEVICE_ID1) {
		if (accl_freya_info[card_id].freya1_fw_info.is_freya_ready != FREYA_NOT_READY) {
			LOG_ERR("Not support boot1 firmware update when nvme ready, card id: 0x%x, device id: 0x%x",
				card_id, device_id);
			return PLDM_FW_UPDATE_ERROR;
		}
	} else {
		if (accl_freya_info[card_id].freya2_fw_info.is_freya_ready != FREYA_NOT_READY) {
			LOG_ERR("Not support boot1 firmware update when nvme ready, card id: 0x%x, device id: 0x%x",
				card_id, device_id);
			return PLDM_FW_UPDATE_ERROR;
		}
	}

	/* Stop sensor polling at first package */
	if (p->data_ofs == 0) {
		disable_sensor_poll();
		k_msleep(DISABLE_SENSOR_POLLING_DELAY_MS);
	}

	mux_config accl_mux = { 0 };
	if (get_accl_mux_config(card_id, &accl_mux) != true) {
		LOG_ERR("Fail to get ACCL card mux config, card id: 0x%x", card_id);
		return PLDM_FW_UPDATE_ERROR;
	}

	if (set_mux_channel(accl_mux, MUTEX_LOCK_ENABLE) == false) {
		LOG_ERR("ACCL switch card mux fail, card id: 0x%x", card_id);
		return PLDM_FW_UPDATE_ERROR;
	}

	p->bus = ((card_id < (ASIC_CARD_COUNT / 2)) ? I2C_BUS8 : I2C_BUS7);
	p->addr = (p->comp_id % 2 ? ACCL_ARTEMIS_MODULE_2_ADDR : ACCL_ARTEMIS_MODULE_1_ADDR);

	return PLDM_FW_UPDATE_SUCCESS;
}

static uint8_t pldm_pre_atm_update(void *fw_update_param)
{
	CHECK_NULL_ARG_WITH_RETURN(fw_update_param, PLDM_FW_UPDATE_ERROR);

	pldm_fw_update_param_t *p = (pldm_fw_update_param_t *)fw_update_param;
	uint8_t card_id = (p->comp_id - CB_COMPNT_ACCL1_CH1_FREYA) / 2;
	uint8_t device_id = ((p->comp_id - CB_COMPNT_ACCL1_CH1_FREYA) % 2);

	if (is_time_to_poll_card_sensor(card_id) != true) {
		LOG_ERR("Artemis module power status not ready, card id: 0x%x", card_id);
		return PLDM_FW_UPDATE_ERROR;
	}

	if (device_id == PCIE_DEVICE_ID1) {
		if (accl_freya_info[card_id].freya1_fw_info.is_freya_ready != FREYA_READY) {
			LOG_ERR("ACCL card: 0x%x, device id: 0x%x nvme not ready", card_id,
				device_id);
			return PLDM_FW_UPDATE_ERROR;
		}
	} else {
		if (accl_freya_info[card_id].freya2_fw_info.is_freya_ready != FREYA_READY) {
			LOG_ERR("ACCL card: 0x%x, device id: 0x%x nvme not ready", card_id,
				device_id);
			return PLDM_FW_UPDATE_ERROR;
		}
	}

	mux_config accl_mux = { 0 };
	if (get_accl_mux_config(card_id, &accl_mux) != true) {
		LOG_ERR("Fail to get ACCL card mux config, card id: 0x%x", card_id);
		return PLDM_FW_UPDATE_ERROR;
	}

	/* Stop sensor polling at first package */
	if (p->data_ofs == 0) {
		disable_sensor_poll();
		k_msleep(DISABLE_SENSOR_POLLING_DELAY_MS);
	}

	if (set_mux_channel(accl_mux, MUTEX_LOCK_ENABLE) == false) {
		LOG_ERR("ACCL switch card mux fail, card id: 0x%x", card_id);
		enable_sensor_poll();
		return PLDM_FW_UPDATE_ERROR;
	}

	p->bus = ((card_id < (ASIC_CARD_COUNT / 2)) ? I2C_BUS8 : I2C_BUS7);
	p->addr = (p->comp_id % 2 ? ACCL_ARTEMIS_MODULE_2_ADDR : ACCL_ARTEMIS_MODULE_1_ADDR);

	return PLDM_FW_UPDATE_SUCCESS;
}

static uint8_t pldm_post_atm_update(void *fw_update_param)
{
	ARG_UNUSED(fw_update_param);
	enable_sensor_poll();

	return PLDM_FW_UPDATE_SUCCESS;
}

static uint8_t pldm_atm_update(void *fw_update_param)
{
	CHECK_NULL_ARG_WITH_RETURN(fw_update_param, PLDM_FW_UPDATE_ERROR);

	pldm_fw_update_param_t *p = (pldm_fw_update_param_t *)fw_update_param;

	CHECK_NULL_ARG_WITH_RETURN(p->data, PLDM_FW_UPDATE_ERROR);

	int ret = -1;
	bool is_end_package = false;

	/* prepare next data offset and length */
	p->next_ofs = p->data_ofs + p->data_len;
	p->next_len = fw_update_cfg.max_buff_size;

	if (p->next_ofs < fw_update_cfg.image_size) {
		if (p->next_ofs + p->next_len > fw_update_cfg.image_size)
			p->next_len = fw_update_cfg.image_size - p->next_ofs;
	} else {
		/* current data is the last packet
                 * set the next data length to 0 to inform the update completely
                 */
		p->next_len = 0;
		is_end_package = true;
	}

	ret = atm_fw_update(p->bus, p->addr, p->data_ofs, p->data, p->data_len,
			    fw_update_cfg.image_size, is_end_package);
	if (ret != 0) {
		LOG_ERR("Artemis module firmware update fail, offset: 0x%x, length: 0x%x, status: %d",
			p->data_ofs, p->data_len, ret);
		return PLDM_FW_UPDATE_ERROR;
	}
	return PLDM_FW_UPDATE_SUCCESS;
}

uint8_t pldm_atm_apply_work(void *arg)
{
	ARG_UNUSED(arg);

	uint8_t ret = 0;
	uint8_t index = 0;

	for (index = 0; index < atm_wait_fw_info.timeout_s; ++index) {
		k_sleep(K_SECONDS(WAIT_FIRMWARE_READY_DELAY_S));

		if (atm_wait_fw_info.is_work_done == true) {
			switch (atm_wait_fw_info.status) {
			case EXEC_STATUS_COMPLETE:
				if (atm_wait_fw_info.result != EXEC_RESULT_PASS) {
					ret = PLDM_FW_UPDATE_APPLY_FAIL_WITH_MEMORY_WRITE_ISSUE;
					goto exit;
				}
				ret = PLDM_FW_UPDATE_APPLY_SUCCESS;
				goto exit;

			case EXEC_STATUS_TIMEOUT:
				ret = PLDM_FW_UPDATE_APPLY_TIMEOUT_OCCURRED;
				goto exit;

			default:
				LOG_ERR("Invalid status: 0x%x", atm_wait_fw_info.status);
				ret = PLDM_FW_UPDATE_APPLY_TIMEOUT_OCCURRED;
				goto exit;
			}
		}

		index += (WAIT_FIRMWARE_READY_DELAY_S - 1);
	}

	ret = PLDM_FW_UPDATE_APPLY_TIMEOUT_OCCURRED;

exit:
	atm_wait_fw_info.status = EXEC_STATUS_DEFAULT;
	atm_wait_fw_info.result = EXEC_RESULT_DEFAULT;
	atm_wait_fw_info.is_work_done = false;
	return ret;
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

uint8_t plat_pldm_query_device_identifiers(const uint8_t *buf, uint16_t len, uint8_t *resp,
					   uint16_t *resp_len)
{
	CHECK_NULL_ARG_WITH_RETURN(buf, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp_len, PLDM_ERROR);

	uint8_t index = 0;
	uint8_t ret = PLDM_ERROR;
	uint8_t length = 0;
	uint8_t *end_of_id_ptr =
		(uint8_t *)resp + sizeof(struct pldm_query_device_identifiers_resp);
	struct pldm_query_device_identifiers_resp *resp_p =
		(struct pldm_query_device_identifiers_resp *)resp;

	resp_p->completion_code = PLDM_ERROR;
	resp_p->descriptor_count = bic_descriptors_count;

	uint16_t total_size_of_descriptor = 0;

	for (index = 0; index < bic_descriptors_count; ++index) {
		ret = fill_descriptor_into_buf(&PLDM_DEVICE_DESCRIPTOR_TABLE[index], end_of_id_ptr,
					       &length, total_size_of_descriptor);
		if (ret != PLDM_SUCCESS) {
			LOG_ERR("Fill device descriptor into buffer fail");
			continue;
		}

		total_size_of_descriptor += length;
		end_of_id_ptr += length;
	}

	resp_p->device_identifiers_len = total_size_of_descriptor;
	*resp_len = sizeof(struct pldm_query_device_identifiers_resp) + total_size_of_descriptor;
	resp_p->completion_code = PLDM_SUCCESS;
	return PLDM_SUCCESS;
}

uint8_t plat_pldm_query_downstream_identifiers(const uint8_t *buf, uint16_t len, uint8_t *resp,
					       uint16_t *resp_len)
{
	CHECK_NULL_ARG_WITH_RETURN(buf, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp_len, PLDM_ERROR);

	uint8_t ret = PLDM_ERROR;
	uint8_t index = 0;
	uint8_t length = 0;
	uint8_t *end_of_id_ptr =
		(uint8_t *)resp + sizeof(struct pldm_query_downstream_identifier_resp);
	struct pldm_query_downstream_identifier_req *req_p =
		(struct pldm_query_downstream_identifier_req *)buf;
	struct pldm_query_downstream_identifier_resp *resp_p =
		(struct pldm_query_downstream_identifier_resp *)resp;

	resp_p->completion_code = PLDM_ERROR;
	switch (req_p->transferoperationflag) {
	case PLDM_FW_UPDATE_GET_FIRST_PART:
		req_p->datatransferhandle = 0;
		resp_p->transferflag = PLDM_FW_UPDATE_TRANSFER_START;
		resp_p->nextdatatransferhandle = PLDM_DOWNSTREAM_START_FLAG;
		break;
	case PLDM_FW_UPDATE_GET_NEXT_PART:
		if (req_p->datatransferhandle >= downstream_table_count) {
			return ret;
		}
		if (req_p->datatransferhandle == (downstream_table_count - 1)) {
			resp_p->transferflag = PLDM_FW_UPDATE_TRANSFER_END;
		} else {
			resp_p->transferflag = PLDM_FW_UPDATE_TRANSFER_MIDDLE;
			resp_p->nextdatatransferhandle = (req_p->datatransferhandle + 1);
		}
		break;
	default:
		LOG_ERR("Invalid transfer operation flag: 0x%x", req_p->transferoperationflag);
		return ret;
	}

	resp_p->numbwerofdownstreamdevice =
		downstream_table[req_p->datatransferhandle].descriptor_count;
	resp_p->downstreamdeviceindex = req_p->datatransferhandle;

	uint16_t total_size_of_descriptor = 0;
	struct pldm_descriptor_string *descriptor_table =
		downstream_table[req_p->datatransferhandle].descriptor;
	for (index = 0; index < downstream_table[req_p->datatransferhandle].descriptor_count;
	     ++index) {
		ret = fill_descriptor_into_buf(&descriptor_table[index], end_of_id_ptr, &length,
					       total_size_of_descriptor);
		if (ret != PLDM_SUCCESS) {
			LOG_ERR("Fill device descriptor into buffer fail");
			continue;
		}

		total_size_of_descriptor += length;
		end_of_id_ptr += length;
	}

	resp_p->downstreamdevicelength = total_size_of_descriptor + sizeof(uint16_t) +
					 sizeof(resp_p->downstreamdeviceindex) +
					 sizeof(resp_p->downstreamdescriptorcount);
	resp_p->downstreamdescriptorcount = total_size_of_descriptor;
	*resp_len = sizeof(struct pldm_query_downstream_identifier_resp) + total_size_of_descriptor;
	resp_p->completion_code = PLDM_SUCCESS;
	return PLDM_SUCCESS;
}

uint16_t plat_find_update_info_work(uint16_t comp_id)
{
	if (comp_id >= CB_COMPNT_ACCL1_CH1_FREYA && comp_id <= CB_COMPNT_ACCL12_CH2_FREYA) {
		return CB_COMPNT_ACCL1_CH1_FREYA;
	}

	return comp_id;
}
