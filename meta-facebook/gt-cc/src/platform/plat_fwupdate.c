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
#include "plat_gpio.h"
#include "plat_i2c.h"
#include "pldm_firmware_update.h"
#include "sensor.h"
#include "plat_sensor_table.h"
#include "plat_hook.h"
#include "i2c-mux-tca9548.h"
#include "util_spi.h"

LOG_MODULE_REGISTER(plat_fwupdate);

static uint8_t pldm_pre_vr_update(void *fw_update_param);
static uint8_t pldm_post_vr_update(void *fw_update_param);
static uint8_t pldm_pre_cpld_update(void *fw_update_param);
static uint8_t pldm_pre_pex_update(void *fw_update_param);
static uint8_t pldm_pex_update(void *fw_update_param);
static uint8_t pldm_post_pex_update(void *fw_update_param);

/* PLDM FW update table */
// clang-format off
pldm_fw_update_info_t PLDMUPDATE_FW_CONFIG_TABLE[] = {
	[COMP_ID_BIC] =  { ENABLE, COMP_CLASS_TYPE_DOWNSTREAM, COMP_ID_BIC, 0x00, NULL, pldm_bic_update, NULL, COMP_UPDATE_VIA_SPI, COMP_ACT_SELF, pldm_bic_activate },
	[COMP_ID_VR0] =  { ENABLE, COMP_CLASS_TYPE_DOWNSTREAM, COMP_ID_VR0, 0x00, pldm_pre_vr_update, pldm_vr_update, pldm_post_vr_update, COMP_UPDATE_VIA_I2C, COMP_ACT_AC_PWR_CYCLE, NULL },
	[COMP_ID_VR1] =  { ENABLE, COMP_CLASS_TYPE_DOWNSTREAM, COMP_ID_VR1, 0x00, pldm_pre_vr_update, pldm_vr_update, pldm_post_vr_update, COMP_UPDATE_VIA_I2C, COMP_ACT_AC_PWR_CYCLE, NULL },
	[COMP_ID_PEX0] = { ENABLE, COMP_CLASS_TYPE_DOWNSTREAM, COMP_ID_PEX0, 0x00, pldm_pre_pex_update, pldm_pex_update, pldm_post_pex_update, COMP_UPDATE_VIA_SPI, COMP_ACT_AC_PWR_CYCLE, NULL },
	[COMP_ID_PEX1] = { ENABLE, COMP_CLASS_TYPE_DOWNSTREAM, COMP_ID_PEX1, 0x00, pldm_pre_pex_update, pldm_pex_update, pldm_post_pex_update, COMP_UPDATE_VIA_SPI, COMP_ACT_AC_PWR_CYCLE, NULL },
	[COMP_ID_PEX2] = { ENABLE, COMP_CLASS_TYPE_DOWNSTREAM, COMP_ID_PEX2, 0x00, pldm_pre_pex_update, pldm_pex_update, pldm_post_pex_update, COMP_UPDATE_VIA_SPI, COMP_ACT_AC_PWR_CYCLE, NULL },
	[COMP_ID_PEX3] = { ENABLE, COMP_CLASS_TYPE_DOWNSTREAM, COMP_ID_PEX3, 0x00, pldm_pre_pex_update, pldm_pex_update, pldm_post_pex_update, COMP_UPDATE_VIA_SPI, COMP_ACT_AC_PWR_CYCLE, NULL },
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
		(p->comp_id == COMP_ID_VR0) ? SENSOR_NUM_PEX_0_VR_TEMP : SENSOR_NUM_PEX_2_VR_TEMP;

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

	uint8_t flash_sel_base = flash_sel_pin[0] - COMP_ID_PEX0;

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
