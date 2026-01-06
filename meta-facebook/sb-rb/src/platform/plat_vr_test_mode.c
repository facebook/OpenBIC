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
#include <logging/log.h>
#include "plat_util.h"
#include "plat_pldm_sensor.h"
#include "plat_vr_test_mode.h"
#include "plat_hook.h"

LOG_MODULE_REGISTER(plat_vr_test_mode);

struct k_thread vr_test_mode_thread;
K_KERNEL_STACK_MEMBER(vr_test_mode_thread_stack, 1024);
k_tid_t vr_test_mode_tid;
bool vr_test_mode_flag = false;

bool get_vr_test_mode_flag(void)
{
	return vr_test_mode_flag;
}

const vr_test_mode_setting_t vr_test_mode_table[] = {
	// vr_rail, fast ocp: x/10(A), slow ocp: x/10(A), uvp: 1(mV), ovp: 1(mV), v max: 1(mV), lcr(mV), ucr(mV)
	{ VR_RAIL_E_ASIC_P0V85_MEDHA0_VDD, 14300, 14300, 200, 940, 930, 595, 930 },
	{ VR_RAIL_E_ASIC_P0V85_MEDHA1_VDD, 14300, 14300, 200, 940, 930, 595, 930 },
	{ VR_RAIL_E_ASIC_P0V9_OWL_E_TRVDD, 170, 170, 200, 1130, 1120, 630, 1120 },
	{ VR_RAIL_E_ASIC_P0V75_OWL_E_TRVDD, 110, 110, 200, 1130, 975, 525, 975 },
	{ VR_RAIL_E_ASIC_P0V75_MAX_M_VDD, 1200, 1200, 200, 1130, 975, 525, 975 },
	{ VR_RAIL_E_ASIC_P0V75_VDDPHY_HBM1357, 1500, 1500, 200, 940, 930, 525, 930 },
	{ VR_RAIL_E_ASIC_P0V75_OWL_E_VDD, 1600, 1600, 200, 940, 930, 525, 930 },
	{ VR_RAIL_E_ASIC_P0V4_VDDQL_HBM1357, 530, 530, 200, 800, 520, 280, 520 },
	{ VR_RAIL_E_ASIC_P1V1_VDDQC_HBM1357, 3200, 3200, 200, 1320, 1310, 770, 1310 },
	{ VR_RAIL_E_ASIC_P1V8_VPP_HBM1357, 140, 140, 200, 1980, 1970, 1260, 1970 },
	{ VR_RAIL_E_ASIC_P0V75_MAX_N_VDD, 670, 670, 200, 1130, 975, 525, 975 },
	{ VR_RAIL_E_ASIC_P0V8_HAMSA_AVDD_PCIE, 500, 400, 200, 1130, 1040, 560, 1040 },
	{ VR_RAIL_E_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE, 100, 100, 200, 1320, 1310, 840, 1310 },
	{ VR_RAIL_E_ASIC_P0V85_HAMSA_VDD, 1000, 1000, 200, 1063, 1053, 595, 1053 },
	{ VR_RAIL_E_ASIC_P1V1_VDDQC_HBM0246, 3200, 3200, 200, 1320, 1310, 770, 1310 },
	{ VR_RAIL_E_ASIC_P1V8_VPP_HBM0246, 140, 140, 200, 1980, 1970, 1260, 1970 },
	{ VR_RAIL_E_ASIC_P0V4_VDDQL_HBM0246, 530, 530, 200, 800, 520, 280, 520 },
	{ VR_RAIL_E_ASIC_P0V75_VDDPHY_HBM0246, 1500, 1500, 200, 940, 930, 525, 930 },
	{ VR_RAIL_E_ASIC_P0V75_OWL_W_VDD, 1600, 1500, 200, 940, 930, 525, 930 },
	{ VR_RAIL_E_ASIC_P0V75_MAX_S_VDD, 660, 660, 200, 1130, 975, 525, 975 },
	{ VR_RAIL_E_ASIC_P0V9_OWL_W_TRVDD, 170, 170, 200, 1130, 1120, 630, 1120 },
	{ VR_RAIL_E_ASIC_P0V75_OWL_W_TRVDD, 110, 110, 200, 1130, 975, 525, 975 },
};

const vr_test_mode_setting_t vr_test_mode_table_default[] = {
	// vr_rail, fast ocp: x/10(A), slow ocp: x/10(A), uvp: 1(mV), ovp: 1(mV), v max: 1(mV), lcr(mV), ucr(mV)
	{ VR_RAIL_E_ASIC_P0V85_MEDHA0_VDD, 14300, 14300, 640, 940, 918, 782, 918 },
	{ VR_RAIL_E_ASIC_P0V85_MEDHA1_VDD, 14300, 14300, 640, 940, 918, 782, 918 },
	{ VR_RAIL_E_ASIC_P0V9_OWL_E_TRVDD, 170, 160, 700, 1010, 954, 846, 954 },
	{ VR_RAIL_E_ASIC_P0V75_OWL_E_TRVDD, 110, 100, 575, 910, 795, 705, 795 },
	{ VR_RAIL_E_ASIC_P0V75_MAX_M_VDD, 670, 570, 575, 910, 848, 690, 848 },
	{ VR_RAIL_E_ASIC_P0V75_VDDPHY_HBM1357, 1230, 1130, 575, 910, 795, 705, 795 },
	{ VR_RAIL_E_ASIC_P0V75_OWL_E_VDD, 1600, 1500, 575, 910, 810, 690, 810 },
	{ VR_RAIL_E_ASIC_P0V4_VDDQL_HBM1357, 460, 360, 325, 800, 440, 380, 440 },
	{ VR_RAIL_E_ASIC_P1V1_VDDQC_HBM1357, 2800, 2700, 900, 1260, 1177, 1067, 1177 },
	{ VR_RAIL_E_ASIC_P1V8_VPP_HBM1357, 140, 140, 1400, 1960, 1950, 1746, 1950 },
	{ VR_RAIL_E_ASIC_P0V75_MAX_N_VDD, 670, 570, 575, 910, 848, 690, 848 },
	{ VR_RAIL_E_ASIC_P0V8_HAMSA_AVDD_PCIE, 500, 400, 635, 940, 848, 752, 848 },
	{ VR_RAIL_E_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE, 100, 90, 1000, 1300, 1272, 1128, 1272 },
	{ VR_RAIL_E_ASIC_P0V85_HAMSA_VDD, 850, 750, 640, 940, 918, 782, 918 },
	{ VR_RAIL_E_ASIC_P1V1_VDDQC_HBM0246, 2800, 2700, 900, 1260, 1177, 1067, 1177 },
	{ VR_RAIL_E_ASIC_P1V8_VPP_HBM0246, 140, 140, 1400, 1960, 1950, 1746, 1950 },
	{ VR_RAIL_E_ASIC_P0V4_VDDQL_HBM0246, 460, 360, 325, 800, 440, 380, 440 },
	{ VR_RAIL_E_ASIC_P0V75_VDDPHY_HBM0246, 1230, 1130, 575, 910, 795, 705, 795 },
	{ VR_RAIL_E_ASIC_P0V75_OWL_W_VDD, 1600, 1500, 575, 910, 810, 690, 810 },
	{ VR_RAIL_E_ASIC_P0V75_MAX_S_VDD, 580, 480, 575, 910, 848, 690, 848 },
	{ VR_RAIL_E_ASIC_P0V9_OWL_W_TRVDD, 170, 160, 700, 1010, 954, 846, 954 },
	{ VR_RAIL_E_ASIC_P0V75_OWL_W_TRVDD, 110, 100, 575, 910, 795, 705, 795 },
};

const uint8_t vr_test_mode_table_size = ARRAY_SIZE(vr_test_mode_table);
const uint8_t vr_test_mode_table_dafault_size = ARRAY_SIZE(vr_test_mode_table_default);

static bool update_vr_reg(uint8_t rail, uint8_t reg, uint16_t val)
{
	uint8_t data[2];
	data[0] = val & 0xFF;
	data[1] = (val >> 8) & 0xFF;

	if (!plat_set_vr_reg(rail, reg, data, 2)) {
		LOG_ERR("fail to set vr rail %x, reg 0x%02x, val: %x", rail, reg, val);
		return false;
	}
	return true;
}

bool dma_write_vr(uint8_t rail, uint16_t reg, uint8_t *data, uint8_t len)
{
	if (!update_vr_reg(rail, 0xC7, reg)) {
		LOG_ERR("VR %x dma write 0x%04x to 0xC7 fail", rail, reg);
		return false;
	}

	if (!plat_set_vr_reg(rail, 0xC5, data, len)) {
		LOG_ERR("VR %x dma write 0xC5 fail", rail);
		return false;
	}

	return true;
}

bool dma_read_vr(uint8_t rail, uint16_t reg, uint8_t *data, uint8_t len)
{
	if (!update_vr_reg(rail, 0xC7, reg)) {
		LOG_ERR("VR %x dma write 0x%04x to 0xC7 fail", rail, reg);
		return false;
	}
	uint8_t sensor_num = vr_rail_table[rail].sensor_id;
	if (!get_raw_data_from_sensor_id(sensor_num, 0xC5, data, len)) {
		LOG_ERR("VR %x dma write 0x%04x to 0xC5 fail", rail, reg);
		return false;
	}
	return true;
}

// uvp = vout - offset uvp, uvp = vout + offset ovp,
bool get_vr_offset_uvp_ovp(uint8_t rail, uint16_t *uvp, uint16_t *ovp)
{
	CHECK_NULL_ARG_WITH_RETURN(uvp, false);
	CHECK_NULL_ARG_WITH_RETURN(ovp, false);

	uint8_t data[2];
	uint8_t sensor_num = vr_rail_table[rail].sensor_id;
	if (!get_raw_data_from_sensor_id(sensor_num, VR_VOUT_REG, data, sizeof(data))) {
		LOG_ERR("VR %x get Vout fail", rail);
		return false;
	}

	uint16_t vout = (data[1] << 8) | data[0];
	uint8_t page = get_vr_page(rail);

	uint16_t reg = page ? VR_UVP_1_DMA_ADDR : VR_UVP_0_DMA_ADDR;
	if (!dma_read_vr(rail, reg, data, 2)) {
		LOG_ERR("VR %x get dma uvp fail", rail);
		return false;
	}
	uint16_t uvp_offset = (data[1] << 8) | data[0];

	reg = page ? VR_OVP_1_DMA_ADDR : VR_OVP_0_DMA_ADDR;
	if (!dma_read_vr(rail, reg, data, 2)) {
		LOG_ERR("VR %x get dma ovp fail", rail);
		return false;
	}
	uint16_t ovp_offset = (data[1] << 8) | data[0];

	*uvp = (vout > uvp_offset) ? (vout - uvp_offset) : 0;
	*ovp = vout + ovp_offset;

	return true;
}

// check if offset uvp/ovp is used.
bool use_offset_uvp_ovp(uint8_t rail)
{
	switch (rail) {
	case VR_RAIL_E_ASIC_P0V85_MEDHA0_VDD:
	case VR_RAIL_E_ASIC_P0V85_MEDHA1_VDD:
		return false;
	default:
		return true;
	}
}
bool get_vr_fixed_uvp_ovp_enable(uint8_t rail)
{
	// no offset UVP/OVP setting
	if (!use_offset_uvp_ovp(rail))
		return true;

	uint8_t data[2];
	uint8_t sensor_num = vr_rail_table[rail].sensor_id;
	if (!get_raw_data_from_sensor_id(sensor_num, VR_LOOPCFG_REG, data, sizeof(data))) {
		LOG_ERR("VR %x get loopcfg fail", rail);
		return false;
	}

	if (data[1] & BIT(6))
		return true;
	return false;
}
bool set_vr_fixed_uvp_ovp_enable(uint8_t rail, uint8_t enable)
{
	// no offset UVP/OVP setting
	if (!use_offset_uvp_ovp(rail))
		return true;

	uint8_t data[2];
	uint8_t sensor_num = vr_rail_table[rail].sensor_id;
	if (!get_raw_data_from_sensor_id(sensor_num, VR_LOOPCFG_REG, data, sizeof(data))) {
		LOG_ERR("VR %x get loopcfg fail", rail);
		return false;
	}

	WRITE_BIT(data[1], 6, enable);
	if (!plat_set_vr_reg(rail, VR_LOOPCFG_REG, data, 2)) {
		LOG_ERR("VR %x set loopcfg fail: %x %x", rail, data[0], data[1]);
		return false;
	}

	return true;
}

static bool set_vr_test_mode_reg(bool is_default)
{
	bool ret = true;

	const vr_test_mode_setting_t *table =
		is_default ? vr_test_mode_table_default : vr_test_mode_table;
	uint8_t table_size = is_default ? ARRAY_SIZE(vr_test_mode_table_default) :
					  ARRAY_SIZE(vr_test_mode_table);

	for (uint8_t i = 0; i < table_size; i++) {
		const vr_test_mode_setting_t *cfg = &table[i];

		const struct {
			uint8_t offset;
			uint16_t val;
			const char *name;
		} regs[] = {
			{ VR_FAST_OCP_REG, cfg->fast_ocp, "FAST OCP" },
			{ VR_SLOW_OCP_REG, cfg->slow_ocp, "SLOW OCP" },
			{ VR_UVP_REG, cfg->uvp, "UVP" },
			{ VR_OVP_REG, cfg->ovp, "OVP" },
			{ VR_VOUT_MAX_REG, cfg->vout_max, "VOUT MAX" },
		};
		for (size_t j = 0; j < ARRAY_SIZE(regs); j++) {
			if (!update_vr_reg(cfg->vr_rail, regs[j].offset, regs[j].val)) {
				ret = false;
				LOG_ERR("VR rail %x set %s to %d failed", cfg->vr_rail,
					regs[j].name, regs[j].val);
			}
		}

		// vr range
		vout_range_user_settings.change_vout_min[i] = cfg->lcr;
		vout_range_user_settings.change_vout_max[i] = cfg->ucr;
	}
	return ret;
}

static bool check_vr_fast_ocp_match_test_mode(void)
{
	for (uint8_t i = 0; i < VR_RAIL_E_P3V3_OSFP_VOLT_V; i++) {
		uint8_t data[2] = { 0 };
		if (!get_raw_data_from_sensor_id(vr_rail_table[i].sensor_id, VR_FAST_OCP_REG, data,
						 2))
			return false;

		uint16_t raw_val = (data[1] << 8) | data[0];
		if (vr_test_mode_table[i].fast_ocp != raw_val)
			return false;
	}

	return true;
}

void vr_test_mode_enable(bool onoff)
{
	vr_test_mode_flag = onoff;

	set_vr_test_mode_reg((onoff ? false : true));
	// not include P3V3
	for (uint8_t i = 0; i < VR_RAIL_E_P3V3_OSFP_VOLT_V; i++) {
		if (!set_vr_fixed_uvp_ovp_enable(i, (onoff ? 1 : 0)))
			LOG_ERR("set vr %d fix uvp/ovp enable fail!", i);
	}
}

void vr_test_mode_handler(void *arg1, void *arg2, void *arg3)
{
	k_sleep(K_MSEC(5000)); // wait sensor thread ready

	while (1) {
		k_sleep(K_MINUTES(1));
		if (vr_test_mode_flag)
			LOG_INF("VR TEST MODE is running!");
	}
}

void init_vr_test_mode_polling(void)
{
	if (is_dc_on()) {
		if (check_vr_fast_ocp_match_test_mode()) {
			vr_test_mode_enable(true);
			LOG_INF("Fast OCP value is the same, start VR TEST MODE");
		}
	}

	vr_test_mode_tid = k_thread_create(&vr_test_mode_thread, vr_test_mode_thread_stack,
					   K_THREAD_STACK_SIZEOF(vr_test_mode_thread_stack),
					   vr_test_mode_handler, NULL, NULL, NULL,
					   CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&vr_test_mode_thread, "vr_test_mode_mode_thread");
}