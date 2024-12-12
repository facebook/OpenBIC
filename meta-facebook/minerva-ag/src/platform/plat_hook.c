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
#include <string.h>
#include "libutil.h"
#include "ast_adc.h"
#include "sensor.h"
#include "hal_i2c.h"
#include "plat_i2c.h"
#include "plat_gpio.h"
#include "plat_sensor_table.h"
#include "plat_hook.h"
#include <logging/log.h>
#include "mp2971.h"
#include "isl69259.h"
#include "raa228249.h"
#include "mp29816a.h"
#include "plat_pldm_sensor.h"
#include "plat_class.h"

LOG_MODULE_REGISTER(plat_hook);

static struct k_mutex vr_mutex[VR_MAX_NUM];

#define VR_PRE_READ_ARG(idx)                                                                       \
	{ .mutex = vr_mutex + idx, .vr_page = 0x0 },                                               \
	{                                                                                          \
		.mutex = vr_mutex + idx, .vr_page = 0x1                                            \
	}

vr_pre_proc_arg vr_pre_read_args[] = {
	// mutex, vr_page

	FOR_EACH (VR_PRE_READ_ARG, (, ), 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10)
};

mp2971_init_arg mp2971_init_args[] = {
	[0] = { .vout_scale_enable = true },
};

isl69259_init_arg isl69259_init_args[] = {
	[0] = { .vout_scale_enable = true, .vout_scale = (499 / 798.8) },
};

void *vr_mutex_get(enum VR_INDEX_E vr_index)
{
	if (vr_index >= VR_INDEX_MAX) {
		LOG_ERR("vr_mutex_get, invalid vr_index %d", vr_index);
		return NULL;
	}

	return vr_mutex + vr_index;
}

bool pre_vr_read(sensor_cfg *cfg, void *args)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(args, false);

	vr_pre_proc_arg *pre_proc_args = (vr_pre_proc_arg *)args;
	uint8_t retry = 5;
	I2C_MSG msg;

	/* mutex lock */
	if (pre_proc_args->mutex) {
		LOG_DBG("%x l %p", cfg->num, pre_proc_args->mutex);
		if (k_mutex_lock(pre_proc_args->mutex, K_MSEC(VR_MUTEX_LOCK_TIMEOUT_MS))) {
			LOG_ERR("pre_vr_read, mutex lock fail");
			return false;
		}
	}

	/* set page */
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 2;
	msg.data[0] = 0x00;
	msg.data[1] = pre_proc_args->vr_page;
	if (i2c_master_write(&msg, retry)) {
		k_mutex_unlock(pre_proc_args->mutex);
		LOG_ERR("pre_vr_read, set page fail");
		return false;
	}
	return true;
}

bool post_vr_read(sensor_cfg *cfg, void *args, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(args, false);
	ARG_UNUSED(reading);

	vr_pre_proc_arg *pre_proc_args = (vr_pre_proc_arg *)args;

	/* mutex unlock */
	if (pre_proc_args->mutex) {
		LOG_DBG("%x u %p", cfg->num, pre_proc_args->mutex);
		if (k_mutex_unlock(pre_proc_args->mutex)) {
			LOG_ERR("post_vr_read, mutex unlock fail");
			return false;
		}
	}

	return true;
}

bool is_mb_dc_on()
{
	/* RST_ATH_PWR_ON_PLD_R1_N is low active,
   * 1 -> power on
   * 0 -> power off
   */
	return gpio_get(RST_ATH_PWR_ON_PLD_R1_N);
}

void vr_mutex_init(void)
{
	for (uint8_t i = 0; i < ARRAY_SIZE(vr_mutex); i++) {
		k_mutex_init(vr_mutex + i);
		LOG_DBG("vr_mutex[%d] %p init", i, vr_mutex + i);
	}

	for (uint8_t i = 0; i < ARRAY_SIZE(vr_pre_read_args); i++) {
		vr_pre_proc_arg *pre_proc_args = vr_pre_read_args + i;
		LOG_DBG("vr_pre_read_args[%d] mutex %p, page %d", i, pre_proc_args->mutex,
			pre_proc_args->vr_page);
	}
}

/* the order is following enum VR_RAIL_E */
vr_mapping_sensor vr_rail_table[] = {
	{ VR_RAIL_E_OSFP_P3V3, SENSOR_NUM_OSFP_P3V3_VOLT_V, "AEGIS_OSFP_P3V3" },
	{ VR_RAIL_E_P0V85, SENSOR_NUM_CPU_P0V85_PVDD_VOLT_V, "AEGIS_CPU_P0V85_PVDD" },
	{ VR_RAIL_E_P0V75_PVDD_CH_N, SENSOR_NUM_CPU_P0V75_PVDD_CH_N_VOLT_V,
	  "AEGIS_CPU_P0V75_PVDD_CH_N" },
	{ VR_RAIL_E_P0V75_MAX_PHY_N, SENSOR_NUM_CPU_P0V75_MAX_PHY_N_VOLT_V,
	  "AEGIS_CPU_P0V75_MAX_PHY_N" },
	{ VR_RAIL_E_P0V75_PVDD_CH_S, SENSOR_NUM_CPU_P0V75_PVDD_CH_S_VOLT_V,
	  "AEGIS_CPU_P0V75_PVDD_CH_S" },
	{ VR_RAIL_E_P0V75_MAX_PHY_S, SENSOR_NUM_CPU_P0V75_MAX_PHY_S_VOLT_V,
	  "AEGIS_CPU_P0V75_MAX_PHY_S" },
	{ VR_RAIL_E_P0V75_TRVDD_ZONEA, SENSOR_NUM_CPU_P0V75_TRVDD_ZONEA_VOLT_V,
	  "AEGIS_CPU_P0V75_TRVDD_ZONEA" },
	{ VR_RAIL_E_P1V8_VPP_HBM0_2_4, SENSOR_NUM_CPU_P1V8_VPP_HBM0_2_4_VOLT_V,
	  "AEGIS_CPU_P1V8_VPP_HBM0_2_4" },
	{ VR_RAIL_E_P0V75_TRVDD_ZONEB, SENSOR_NUM_CPU_P0V75_TRVDD_ZONEB_VOLT_V,
	  "AEGIS_CPU_P0V75_TRVDD_ZONEB" },
	{ VR_RAIL_E_P0V4_VDDQL_HBM0_2_4, SENSOR_NUM_CPU_P0V4_VDDQL_HBM0_2_4_VOLT_V,
	  "AEGIS_CPU_P0V4_VDDQL_HBM0_2_4" },
	{ VR_RAIL_E_P1V1_VDDC_HBM0_2_4, SENSOR_NUM_CPU_P1V1_VDDC_HBM0_2_4_VOLT_V,
	  "AEGIS_CPU_P1V1_VDDC_HBM0_2_4" },
	{ VR_RAIL_E_P0V75_VDDPHY_HBM0_2_4, SENSOR_NUM_CPU_P0V75_VDDPHY_HBM0_2_4_VOLT_V,
	  "AEGIS_CPU_P0V75_VDDPHY_HBM0_2_4" },
	{ VR_RAIL_E_P0V9_TRVDD_ZONEA, SENSOR_NUM_CPU_P0V9_TRVDD_ZONEA_VOLT_V,
	  "AEGIS_CPU_P0V9_TRVDD_ZONEA" },
	{ VR_RAIL_E_P1V8_VPP_HBM1_3_5, SENSOR_NUM_CPU_P1V8_VPP_HBM1_3_5_VOLT_V,
	  "AEGIS_CPU_P1V8_VPP_HBM1_3_5" },
	{ VR_RAIL_E_P0V9_TRVDD_ZONEB, SENSOR_NUM_CPU_P0V9_TRVDD_ZONEB_VOLT_V,
	  "AEGIS_CPU_P0V9_TRVDD_ZONEB" },
	{ VR_RAIL_E_P0V4_VDDQL_HBM1_3_5, SENSOR_NUM_CPU_P0V4_VDDQL_HBM1_3_5_VOLT_V,
	  "AEGIS_CPU_P0V4_VDDQL_HBM1_3_5" },
	{ VR_RAIL_E_P1V1_VDDC_HBM1_3_5, SENSOR_NUM_CPU_P1V1_VDDC_HBM1_3_5_VOLT_V,
	  "AEGIS_CPU_P1V1_VDDC_HBM1_3_5" },
	{ VR_RAIL_E_P0V75_VDDPHY_HBM1_3_5, SENSOR_NUM_CPU_P0V75_VDDPHY_HBM1_3_5_VOLT_V,
	  "AEGIS_CPU_P0V75_VDDPHY_HBM1_3_5" },
	{ VR_RAIL_E_P0V8_VDDA_PCIE, SENSOR_NUM_CPU_P0V8_VDDA_PCIE_VOLT_V,
	  "AEGIS_CPU_P0V8_VDDA_PCIE" },
	{ VR_RAIL_E_P1V2_VDDHTX_PCIE, SENSOR_NUM_CPU_P1V2_VDDHTX_PCIE_VOLT_V,
	  "AEGIS_CPU_P1V2_VDDHTX_PCIE" },
};

bool vr_rail_name_get(uint8_t rail, uint8_t **name)
{
	CHECK_NULL_ARG_WITH_RETURN(name, false);

	if (rail >= VR_RAIL_E_MAX) {
		*name = NULL;
		return false;
	}

	*name = (uint8_t *)vr_rail_table[rail].sensor_name;
	return true;
}

#define VR_VOUT_USER_SETTINGS_OFFSET 0x8000
struct vr_vout_user_settings {
	uint16_t vout[VR_RAIL_E_MAX];
} __attribute__((packed));

struct vr_vout_user_settings user_settings = { 0 };
struct vr_vout_user_settings default_settings = { 0 };

bool vr_rail_enum_get(uint8_t *name, uint8_t *num)
{
	CHECK_NULL_ARG_WITH_RETURN(name, false);
	CHECK_NULL_ARG_WITH_RETURN(num, false);

	for (int i = 0; i < VR_RAIL_E_MAX; i++) {
		if (strcmp(name, vr_rail_table[i].sensor_name) == 0) {
			*num = i;
			return true;
		}
	}

	LOG_ERR("invalid rail name %s", name);
	return false;
}

bool vr_vout_user_settings_get(void *user_settings)
{
	CHECK_NULL_ARG_WITH_RETURN(user_settings, false);

	/* read the user_settings from eeprom */
	I2C_MSG msg = { 0 };
	uint8_t retry = 5;
	msg.bus = I2C_BUS12;
	msg.target_addr = 0xA0 >> 1;
	msg.tx_len = 2;
	msg.data[0] = VR_VOUT_USER_SETTINGS_OFFSET >> 8;
	msg.data[1] = VR_VOUT_USER_SETTINGS_OFFSET & 0xff;
	msg.rx_len = sizeof(struct vr_vout_user_settings);

	if (i2c_master_read(&msg, retry)) {
		LOG_ERR("Failed to read eeprom, bus: %d, addr: 0x%x, reg: 0x%x", msg.bus,
			msg.target_addr, msg.data[0]);
		return false;
	}
	memcpy(user_settings, msg.data, sizeof(struct vr_vout_user_settings));

	return true;
}

bool vr_vout_user_settings_set(void *user_settings)
{
	CHECK_NULL_ARG_WITH_RETURN(user_settings, false);

	/* write the user_settings to eeprom */
	I2C_MSG msg = { 0 };
	uint8_t retry = 5;
	msg.bus = I2C_BUS12;
	msg.target_addr = 0xA0 >> 1;
	msg.tx_len = sizeof(struct vr_vout_user_settings) + 2;
	msg.data[0] = VR_VOUT_USER_SETTINGS_OFFSET >> 8;
	msg.data[1] = VR_VOUT_USER_SETTINGS_OFFSET & 0xff;

	memcpy(&msg.data[2], user_settings, sizeof(struct vr_vout_user_settings));
	if (i2c_master_write(&msg, retry)) {
		LOG_ERR("Failed to write eeprom, bus: %d, addr: 0x%x, reg: 0x%x", msg.bus,
			msg.target_addr, msg.data[0]);
		return false;
	}

	return true;
}

static bool vr_vout_user_settings_init(void)
{
	if (vr_vout_user_settings_get(&user_settings) == false) {
		LOG_ERR("get vout user settings fail");
		return false;
	}

	for (int i = 0; i < VR_RAIL_E_MAX; i++) {
		if (user_settings.vout[i] != 0xffff) {
			/* write vout */
			uint16_t millivolt = user_settings.vout[i];
			if (!plat_set_vout_command(i, &user_settings.vout[i], false, false)) {
				LOG_ERR("Can't set vout[%d]=%x by user settings", i, millivolt);
				return false;
			}
			LOG_INF("set [%d]%s: %dmV", i, vr_rail_table[i].sensor_name, millivolt);
		}
	}

	return true;
}

static bool vr_vout_default_settings_init(void)
{
	for (int i = 0; i < VR_RAIL_E_MAX; i++) {
		if ((get_board_type() == MINERVA_AEGIS_BD) && (i == 0)) {
			default_settings.vout[i] = 0xffff;
			continue; // skip osfp p3v3 on AEGIS BD
		}
		uint16_t vout = 0;
		if (!plat_get_vout_command(i, &vout)) {
			LOG_ERR("Can't find vout default by rail index: %d", i);
			return false;
		}
		default_settings.vout[i] = vout;
	}

	return true;
}

/* init the user & default settings value by shell command */
void user_settings_init(void)
{
	vr_vout_default_settings_init();
	vr_vout_user_settings_init();
}

bool plat_get_vout_command(uint8_t rail, uint16_t *millivolt)
{
	CHECK_NULL_ARG_WITH_RETURN(millivolt, false);

	bool ret = false;
	uint8_t sensor_id = vr_rail_table[rail].sensor_id;
	sensor_cfg *cfg = get_sensor_cfg_by_sensor_id(sensor_id);

	if (cfg == NULL) {
		LOG_ERR("Failed to get sensor config for sensor 0x%x", sensor_id);
		return false;
	}

	vr_pre_proc_arg *pre_proc_args = vr_pre_read_args + rail;

	if (cfg->pre_sensor_read_hook) {
		if (!cfg->pre_sensor_read_hook(cfg, cfg->pre_sensor_read_args)) {
			LOG_ERR("sensor id: 0x%x pre-read fail", sensor_id);
			goto err;
		}
	}

	switch (cfg->type) {
	case sensor_dev_isl69259:
		if (!isl69260_get_vout_command(cfg, pre_proc_args->vr_page, millivolt)) {
			LOG_ERR("The VR ISL69260 vout reading failed");
			goto err;
		}
		break;
	case sensor_dev_mp2971:
		if (!mp2971_get_vout_command(cfg, pre_proc_args->vr_page, millivolt)) {
			LOG_ERR("The VR MPS2971 vout reading failed");
			goto err;
		}
		break;
	case sensor_dev_mp29816a:
		if (!mp29816a_get_vout_command(cfg, pre_proc_args->vr_page, millivolt)) {
			LOG_ERR("The VR MPS29816a vout reading failed");
			goto err;
		}
		break;
	case sensor_dev_raa228249:
		if (!raa228249_get_vout_command(cfg, pre_proc_args->vr_page, millivolt)) {
			LOG_ERR("The VR RAA228249 vout reading failed");
			goto err;
		}
		break;
	default:
		LOG_ERR("Unsupport VR type(%x)", cfg->type);
		goto err;
	}

	ret = true;
err:
	if (cfg->post_sensor_read_hook) {
		if (cfg->post_sensor_read_hook(cfg, cfg->post_sensor_read_args, NULL) == false) {
			LOG_ERR("sensor id: 0x%x post-read fail", sensor_id);
		}
	}
	return ret;
}

bool plat_set_vout_command(uint8_t rail, uint16_t *millivolt, bool is_default, bool is_perm)
{
	CHECK_NULL_ARG_WITH_RETURN(millivolt, false);

	bool ret = false;
	uint8_t sensor_id = vr_rail_table[rail].sensor_id;
	sensor_cfg *cfg = get_sensor_cfg_by_sensor_id(sensor_id);

	if (cfg == NULL) {
		LOG_ERR("Failed to get sensor config for sensor 0x%x", sensor_id);
		return false;
	}

	vr_pre_proc_arg *pre_proc_args = vr_pre_read_args + rail;

	if (cfg->pre_sensor_read_hook) {
		if (!cfg->pre_sensor_read_hook(cfg, cfg->pre_sensor_read_args)) {
			LOG_ERR("sensor id: 0x%x pre-read fail", sensor_id);
			goto err;
		}
	}

	if (is_default) {
		*millivolt = default_settings.vout[rail];
	}

	switch (cfg->type) {
	case sensor_dev_isl69259:
		if (!isl69260_set_vout_command(cfg, pre_proc_args->vr_page, millivolt)) {
			LOG_ERR("The VR ISL69260 vout setting failed");
			goto err;
		}
		break;
	case sensor_dev_mp2971:
		if (!mp2971_set_vout_command(cfg, pre_proc_args->vr_page, millivolt)) {
			LOG_ERR("The VR MPS2971 vout setting failed");
			goto err;
		}
		break;
	case sensor_dev_mp29816a:
		if (!mp29816a_set_vout_command(cfg, pre_proc_args->vr_page, millivolt)) {
			LOG_ERR("The VR MPS29816a vout setting failed");
			goto err;
		}
		break;
	case sensor_dev_raa228249:
		if (!raa228249_set_vout_command(cfg, pre_proc_args->vr_page, millivolt)) {
			LOG_ERR("The VR RAA228249 vout setting failed");
			goto err;
		}
		break;
	default:
		LOG_ERR("Unsupport VR type(%x)", cfg->type);
		goto err;
	}

	if (is_perm) {
		user_settings.vout[rail] = *millivolt;
		vr_vout_user_settings_set(&user_settings);
	}

	ret = true;
err:
	if (cfg->post_sensor_read_hook) {
		if (cfg->post_sensor_read_hook(cfg, cfg->post_sensor_read_args, NULL) == false) {
			LOG_ERR("sensor id: 0x%x post-read fail", sensor_id);
		}
	}
	return ret;
}

#define PLAT_VOUT_GET_SET_HANDLER(get_set, min_max)                                                \
	bool plat_##get_set##_vout_##min_max(uint8_t rail, uint16_t *millivolt)                    \
	{                                                                                          \
		CHECK_NULL_ARG_WITH_RETURN(millivolt, false);                                      \
                                                                                                   \
		bool ret = false;                                                                  \
                                                                                                   \
		uint8_t sensor_id = vr_rail_table[rail].sensor_id;                                 \
                                                                                                   \
		sensor_cfg *cfg = get_sensor_cfg_by_sensor_id(sensor_id);                          \
                                                                                                   \
		if (cfg == NULL) {                                                                 \
			LOG_ERR("Failed to get sensor config for sensor 0x%x", sensor_id);         \
			return false;                                                              \
		}                                                                                  \
                                                                                                   \
		vr_pre_proc_arg *pre_proc_args = vr_pre_read_args + rail;                          \
                                                                                                   \
		if (cfg->pre_sensor_read_hook) {                                                   \
			if (!cfg->pre_sensor_read_hook(cfg, cfg->pre_sensor_read_args)) {          \
				goto err;                                                          \
			}                                                                          \
		}                                                                                  \
                                                                                                   \
		switch (cfg->type) {                                                               \
		case sensor_dev_mp2971:                                                            \
			if (!mp2971_##get_set##_vout_##min_max(cfg, pre_proc_args->vr_page,        \
							       millivolt)) {                       \
				LOG_ERR("The VR MPS2971 vout reading failed");                     \
				goto err;                                                          \
			}                                                                          \
			break;                                                                     \
		case sensor_dev_mp29816a:                                                          \
			if (!mp29816a_##get_set##_vout_##min_max(cfg, pre_proc_args->vr_page,      \
								 millivolt)) {                     \
				LOG_ERR("The VR MPS29816a vout reading failed");                   \
				goto err;                                                          \
			}                                                                          \
			break;                                                                     \
		case sensor_dev_isl69259:                                                          \
			if (!isl69260_##get_set##_vout_##min_max(cfg, pre_proc_args->vr_page,      \
								 millivolt)) {                     \
				LOG_ERR("The VR ISL69260 vout reading failed");                    \
				goto err;                                                          \
			}                                                                          \
			break;                                                                     \
		case sensor_dev_raa228249:                                                         \
			if (!raa228249_##get_set##_vout_##min_max(cfg, pre_proc_args->vr_page,     \
								  millivolt)) {                    \
				LOG_ERR("The VR RAA228249 vout reading failed");                   \
				goto err;                                                          \
			}                                                                          \
			break;                                                                     \
		default:                                                                           \
			LOG_ERR("Unsupport VR type(%x)", cfg->type);                               \
			goto err;                                                                  \
		}                                                                                  \
                                                                                                   \
		ret = true;                                                                        \
                                                                                                   \
	err:                                                                                       \
		if (cfg->post_sensor_read_hook) {                                                  \
			if (cfg->post_sensor_read_hook(cfg, cfg->post_sensor_read_args, NULL) ==   \
			    false) {                                                               \
				LOG_ERR("sensor id: 0x%x post-read fail", sensor_id);              \
			}                                                                          \
		}                                                                                  \
		return ret;                                                                        \
	}

PLAT_VOUT_GET_SET_HANDLER(get, min);
PLAT_VOUT_GET_SET_HANDLER(get, max);
PLAT_VOUT_GET_SET_HANDLER(set, min);
PLAT_VOUT_GET_SET_HANDLER(set, max);

#undef PLAT_VOUT_GET_SET_HANDLER
