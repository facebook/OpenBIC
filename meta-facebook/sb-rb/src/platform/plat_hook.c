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
#include <logging/log.h>
#include "plat_hook.h"
#include "pmbus.h"
#include "plat_gpio.h"
#include "plat_pldm_sensor.h"
#include "mp2971.h"
#include "mp29816a.h"
#include "raa228249.h"

LOG_MODULE_REGISTER(plat_hook);

static struct k_mutex vr_mutex[VR_MAX_NUM];

#define EEPROM_MAX_WRITE_TIME 5 // the BR24G512 eeprom max write time is 3.5 ms

vr_pre_proc_arg vr_pre_read_args[] = {
	{ .mutex = vr_mutex + 0, .vr_page = 0x0 },  { .mutex = vr_mutex + 0, .vr_page = 0x1 },
	{ .mutex = vr_mutex + 1, .vr_page = 0x0 },  { .mutex = vr_mutex + 1, .vr_page = 0x1 },
	{ .mutex = vr_mutex + 2, .vr_page = 0x0 },  { .mutex = vr_mutex + 2, .vr_page = 0x1 },
	{ .mutex = vr_mutex + 3, .vr_page = 0x0 },  { .mutex = vr_mutex + 3, .vr_page = 0x1 },
	{ .mutex = vr_mutex + 4, .vr_page = 0x0 },  { .mutex = vr_mutex + 4, .vr_page = 0x1 },
	{ .mutex = vr_mutex + 5, .vr_page = 0x0 },  { .mutex = vr_mutex + 5, .vr_page = 0x1 },
	{ .mutex = vr_mutex + 6, .vr_page = 0x0 },  { .mutex = vr_mutex + 6, .vr_page = 0x1 },
	{ .mutex = vr_mutex + 7, .vr_page = 0x0 },  { .mutex = vr_mutex + 7, .vr_page = 0x1 },
	{ .mutex = vr_mutex + 8, .vr_page = 0x0 },  { .mutex = vr_mutex + 8, .vr_page = 0x1 },
	{ .mutex = vr_mutex + 9, .vr_page = 0x0 },  { .mutex = vr_mutex + 9, .vr_page = 0x1 },
	{ .mutex = vr_mutex + 10, .vr_page = 0x0 }, { .mutex = vr_mutex + 10, .vr_page = 0x1 },
	{ .mutex = vr_mutex + 11, .vr_page = 0x0 }, { .mutex = vr_mutex + 11, .vr_page = 0x1 },
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
			LOG_ERR("0x%02x pre_vr_read, mutex lock fail", cfg->num);
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
		LOG_ERR("0x%02x pre_vr_read, set page fail", cfg->num);
		return false;
	}
	return true;
}

bool post_vr_read(sensor_cfg *cfg, void *args, int *const reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(args, false);

	vr_pre_proc_arg *pre_proc_args = (vr_pre_proc_arg *)args;

	/* mutex unlock */
	if (pre_proc_args->mutex) {
		LOG_DBG("%x u %p", cfg->num, pre_proc_args->mutex);
		if (k_mutex_unlock(pre_proc_args->mutex)) {
			LOG_ERR("0x%02x post_vr_read, mutex unlock fail", cfg->num);
			return false;
		}
	}

	/* set reading val to 0 if reading val is negative */
	sensor_val tmp_reading;
	tmp_reading.integer = (int16_t)(*reading & 0xFFFF);
	tmp_reading.fraction = (int16_t)((*reading >> 16) & 0xFFFF);

	/* sensor_value = 1000 times of true value */
	int32_t sensor_value = tmp_reading.integer * 1000 + tmp_reading.fraction;

	if (sensor_value < 0) {
		LOG_DBG("Original sensor reading: integer = %d, fraction = %d (combined value * 1000: %d)",
			tmp_reading.integer, tmp_reading.fraction, sensor_value);
		*reading = 0;
		LOG_DBG("Negative sensor reading detected. Set reading to 0x%x", *reading);
	}

	return true;
}

bool is_mb_dc_on()
{
	/* RST_IRIS_PWR_ON_PLD_R1_N is low active,
   * 1 -> power on
   * 0 -> power off
   */
	return gpio_get(RST_IRIS_PWR_ON_PLD_R1_N);
}

void vr_mutex_init(void)
{
	for (uint8_t i = 0; i < ARRAY_SIZE(vr_mutex); i++) {
		k_mutex_init(vr_mutex + i);
		LOG_DBG("vr_mutex[%d] %p init", i, vr_mutex + i);
	}
}

/* the order is following enum VR_RAIL_E */
vr_mapping_sensor vr_rail_table[] = {
	{ VR_RAIL_E_ASIC_P0V85_MEDHA0_VDD, SENSOR_NUM_ASIC_P0V85_MEDHA0_VDD_VOLT_V,
	  "CB_ASIC_P0V85_MEDHA0_VDD", 0xffffffff },
	{ VR_RAIL_E_ASIC_P0V85_MEDHA1_VDD, SENSOR_NUM_ASIC_P0V85_MEDHA1_VDD_VOLT_V,
	  "CB_ASIC_P0V85_MEDHA1_VDD", 0xffffffff },
	{ VR_RAIL_E_ASIC_P0V9_OWL_E_TRVDD, SENSOR_NUM_ASIC_P0V9_OWL_E_TRVDD_VOLT_V,
	  "CB_ASIC_P0V9_OWL_E_TRVDD", 0xffffffff },
	{ VR_RAIL_E_ASIC_P0V75_OWL_E_TRVDD, SENSOR_NUM_ASIC_P0V75_OWL_E_TRVDD_VOLT_V,
	  "CB_ASIC_P0V75_OWL_E_TRVDD", 0xffffffff },
	{ VR_RAIL_E_ASIC_P0V75_MAX_M_VDD, SENSOR_NUM_ASIC_P0V75_MAX_M_VDD_VOLT_V,
	  "CB_ASIC_P0V75_MAX_M_VDD", 0xffffffff },
	{ VR_RAIL_E_ASIC_P0V75_VDDPHY_HBM1357, SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM1357_VOLT_V,
	  "CB_ASIC_P0V75_VDDPHY_HBM1357", 0xffffffff },
	{ VR_RAIL_E_ASIC_P0V75_OWL_E_VDD, SENSOR_NUM_ASIC_P0V75_OWL_E_VDD_VOLT_V,
	  "CB_ASIC_P0V75_OWL_E_VDD", 0xffffffff },
	{ VR_RAIL_E_ASIC_P0V4_VDDQL_HBM1357, SENSOR_NUM_ASIC_P0V4_VDDQL_HBM1357_VOLT_V,
	  "CB_ASIC_P0V4_VDDQL_HBM1357", 0xffffffff },
	{ VR_RAIL_E_ASIC_P1V1_VDDQC_HBM1357, SENSOR_NUM_ASIC_P1V1_VDDQC_HBM1357_VOLT_V,
	  "CB_ASIC_P1V1_VDDQC_HBM1357", 0xffffffff },
	{ VR_RAIL_E_ASIC_P1V8_VPP_HBM1357, SENSOR_NUM_ASIC_P1V8_VPP_HBM1357_VOLT_V,
	  "CB_ASIC_P1V8_VPP_HBM1357", 0xffffffff },
	{ VR_RAIL_E_ASIC_P0V75_MAX_N_VDD, SENSOR_NUM_ASIC_P0V75_MAX_N_VDD_VOLT_V,
	  "CB_ASIC_P0V75_MAX_N_VDD", 0xffffffff },
	{ VR_RAIL_E_ASIC_P0V8_HAMSA_AVDD_PCIE, SENSOR_NUM_ASIC_P0V8_HAMSA_AVDD_PCIE_VOLT_V,
	  "CB_ASIC_P0V8_HAMSA_AVDD_PCIE", 0xffffffff },
	{ VR_RAIL_E_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE, SENSOR_NUM_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_VOLT_V,
	  "CB_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE", 0xffffffff },
	{ VR_RAIL_E_ASIC_P0V85_HAMSA_VDD, SENSOR_NUM_ASIC_P0V85_HAMSA_VDD_VOLT_V,
	  "CB_ASIC_P0V85_HAMSA_VDD", 0xffffffff },
	{ VR_RAIL_E_ASIC_P1V1_VDDQC_HBM0246, SENSOR_NUM_ASIC_P1V1_VDDQC_HBM0246_VOLT_V,
	  "CB_ASIC_P1V1_VDDQC_HBM0246", 0xffffffff },
	{ VR_RAIL_E_ASIC_P1V8_VPP_HBM0246, SENSOR_NUM_ASIC_P1V8_VPP_HBM0246_VOLT_V,
	  "CB_ASIC_P1V8_VPP_HBM0246", 0xffffffff },
	{ VR_RAIL_E_ASIC_P0V4_VDDQL_HBM0246, SENSOR_NUM_ASIC_P0V4_VDDQL_HBM0246_VOLT_V,
	  "CB_ASIC_P0V4_VDDQL_HBM0246", 0xffffffff },
	{ VR_RAIL_EASIC_P0V75_VDDPHY_HBM0246, SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM0246_VOLT_V,
	  "CB_ASIC_P0V75_VDDPHY_HBM0246", 0xffffffff },
	{ VR_RAIL_E_ASIC_P0V75_OWL_W_VDD, SENSOR_NUM_ASIC_P0V75_OWL_W_VDD_VOLT_V,
	  "CB_ASIC_P0V75_OWL_W_VDD", 0xffffffff },
	{ VR_RAIL_E_ASIC_P0V75_MAX_S_VDD, SENSOR_NUM_ASIC_P0V75_MAX_S_VDD_VOLT_V,
	  "CB_ASIC_P0V75_MAX_S_VDD", 0xffffffff },
	{ VR_RAIL_E_ASIC_P0V9_OWL_W_TRVDD, SENSOR_NUM_ASIC_P0V9_OWL_W_TRVDD_VOLT_V,
	  "CB_ASIC_P0V9_OWL_W_TRVDD", 0xffffffff },
	{ VR_RAIL_E_ASIC_P0V75_OWL_W_TRVDD, SENSOR_NUM_ASIC_P0V75_OWL_W_TRVDD_VOLT_V,
	  "CB_ASIC_P0V75_OWL_W_TRVDD", 0xffffffff },
};

vr_mapping_status vr_status_table[] = {
	{ VR_STAUS_E_STATUS_BYTE, PMBUS_STATUS_BYTE, "STATUS_BYTE" },
	{ VR_STAUS_E_STATUS_WORD, PMBUS_STATUS_WORD, "STATUS_WORD" },
	{ VR_STAUS_E_STATUS_VOUT, PMBUS_STATUS_VOUT, "STATUS_VOUT" },
	{ VR_STAUS_E_STATUS_IOUT, PMBUS_STATUS_IOUT, "STATUS_IOUT" },
	{ VR_STAUS_E_STATUS_INPUT, PMBUS_STATUS_INPUT, "STATUS_INPUT" },
	{ VR_STAUS_E_STATUS_TEMPERATURE, PMBUS_STATUS_TEMPERATURE, "STATUS_TEMPERATURE" },
	{ VR_STAUS_E_STATUS_CML, PMBUS_STATUS_CML, "STATUS_CML_PMBUS" },
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

bool vr_status_name_get(uint8_t rail, uint8_t **name)
{
	CHECK_NULL_ARG_WITH_RETURN(name, false);

	if (rail >= VR_STAUS_E_MAX) {
		*name = NULL;
		return false;
	}

	*name = (uint8_t *)vr_status_table[rail].vr_status_name;
	return true;
}

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

bool vr_status_enum_get(uint8_t *name, uint8_t *num)
{
	CHECK_NULL_ARG_WITH_RETURN(name, false);
	CHECK_NULL_ARG_WITH_RETURN(num, false);

	for (int i = 0; i < VR_STAUS_E_MAX; i++) {
		if (strcmp(name, vr_status_table[i].vr_status_name) == 0) {
			*num = i;
			return true;
		}
	}

	LOG_ERR("invalid vr status name %s", name);
	return false;
}

bool plat_get_vr_status(uint8_t rail, uint8_t vr_status_rail, uint16_t *vr_status)
{
	CHECK_NULL_ARG_WITH_RETURN(vr_status, false);

	bool ret = false;
	uint8_t sensor_id = vr_rail_table[rail].sensor_id;
	sensor_cfg *cfg = get_sensor_cfg_by_sensor_id(sensor_id);
	CHECK_NULL_ARG_WITH_RETURN(cfg, ret);

	if ((cfg->pre_sensor_read_hook)) {
		if ((cfg->pre_sensor_read_hook)(cfg, cfg->pre_sensor_read_args) == false) {
			LOG_DBG("%d read vr status pre hook fail!", sensor_id);
			return false;
		}
	};

	vr_pre_proc_arg *pre_proc_args = (vr_pre_proc_arg *)cfg->pre_sensor_read_args;

	uint16_t pmbus_reg_id = vr_status_table[vr_status_rail].pmbus_reg;

	switch (cfg->type) {
	case sensor_dev_mp2971:
		if (!mp2971_get_vr_status(cfg, pre_proc_args->vr_page, pmbus_reg_id, vr_status)) {
			LOG_ERR("The VR MPS2971 vr status reading failed");
			goto err;
		}
		break;
	case sensor_dev_mp29816a:
		if (!mp29816a_get_vr_status(cfg, pre_proc_args->vr_page, pmbus_reg_id, vr_status)) {
			LOG_ERR("The VR MPS29816a vr status reading failed");
			goto err;
		}
		break;
	case sensor_dev_raa228249:
		if (!raa228249_get_vr_status(cfg, pre_proc_args->vr_page, pmbus_reg_id,
					     vr_status)) {
			LOG_ERR("The VR RAA228249 vr status reading failed");
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
			LOG_ERR("%d read vr status post hook fail!", sensor_id);
		}
	}
	return ret;
}

bool plat_clear_vr_status(uint8_t rail)
{
	bool ret = false;
	uint8_t sensor_id = vr_rail_table[rail].sensor_id;
	sensor_cfg *cfg = get_sensor_cfg_by_sensor_id(sensor_id);

	if (cfg == NULL) {
		LOG_ERR("Failed to get sensor config for sensor 0x%x", sensor_id);
		return false;
	}

	vr_pre_proc_arg *pre_proc_args = (vr_pre_proc_arg *)cfg->pre_sensor_read_args;

	if (cfg->pre_sensor_read_hook) {
		if (!cfg->pre_sensor_read_hook(cfg, cfg->pre_sensor_read_args)) {
			LOG_ERR("%d clear vr status pre hook fail!", sensor_id);
			goto err;
		}
	}

	switch (cfg->type) {
	case sensor_dev_mp2971:
		if (!mp2971_clear_vr_status(cfg, pre_proc_args->vr_page)) {
			LOG_ERR("The VR MPS2971 vr status clear failed");
			goto err;
		}
		break;
	case sensor_dev_mp29816a:
		if (!mp29816a_clear_vr_status(cfg, pre_proc_args->vr_page)) {
			LOG_ERR("The VR MPS29816a vr status clear failed");
			goto err;
		}
		break;
	case sensor_dev_raa228249:
		if (!raa228249_clear_vr_status(cfg, pre_proc_args->vr_page)) {
			LOG_ERR("The VR RAA228249 vr status clear failed");
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
			LOG_ERR("%d clear vr status post hook fail!", sensor_id);
		}
	}
	return ret;
}