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
	{ VR_RAIL_E_ASIC_P0V85_MEDHA0_VDD, 14500, 14500, 544, 940, 940, 782, 940 },
	{ VR_RAIL_E_ASIC_P0V85_MEDHA1_VDD, 14500, 14500, 544, 940, 940, 782, 940 },
	{ VR_RAIL_E_ASIC_P0V9_OWL_E_TRVDD, 170, 170, 595, 1111, 1111, 846, 1111 },
	{ VR_RAIL_E_ASIC_P0V75_OWL_E_TRVDD, 110, 110, 489, 1017, 1017, 705, 1017 },
	{ VR_RAIL_E_ASIC_P0V75_MAX_M_VDD, 1200, 1200, 489, 1017, 1017, 690, 1017 },
	{ VR_RAIL_E_ASIC_P0V75_VDDPHY_HBM1357, 1500, 1500, 489, 910, 910, 690, 910 },
	{ VR_RAIL_E_ASIC_P0V75_OWL_E_VDD, 1600, 1600, 489, 910, 910, 690, 910 },
	{ VR_RAIL_E_ASIC_P0V4_VDDQL_HBM1357, 530, 530, 277, 800, 800, 368, 800 },
	{ VR_RAIL_E_ASIC_P1V1_VDDQC_HBM1357, 3200, 3200, 765, 1260, 1260, 1034, 1260 },
	{ VR_RAIL_E_ASIC_P1V8_VPP_HBM1357, 140, 140, 1190, 1960, 1960, 1692, 1960 },
	{ VR_RAIL_E_ASIC_P0V75_MAX_N_VDD, 670, 670, 489, 1017, 1017, 690, 1017 },
	{ VR_RAIL_E_ASIC_P0V8_HAMSA_AVDD_PCIE, 500, 500, 540, 1034, 1034, 752, 1034 },
	{ VR_RAIL_E_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE, 100, 100, 850, 1300, 1300, 1128, 1300 },
	{ VR_RAIL_E_ASIC_P0V85_HAMSA_VDD, 1000, 1000, 544, 1034, 1034, 782, 1034 },
	{ VR_RAIL_E_ASIC_P1V1_VDDQC_HBM0246, 3200, 3200, 765, 1260, 1260, 1034, 1260 },
	{ VR_RAIL_E_ASIC_P1V8_VPP_HBM0246, 140, 140, 1190, 1960, 1960, 1692, 1960 },
	{ VR_RAIL_E_ASIC_P0V4_VDDQL_HBM0246, 530, 530, 277, 800, 800, 368, 800 },
	{ VR_RAIL_E_ASIC_P0V75_VDDPHY_HBM0246, 1500, 1500, 489, 910, 910, 690, 910 },
	{ VR_RAIL_E_ASIC_P0V75_OWL_W_VDD, 1600, 1600, 489, 910, 910, 690, 910 },
	{ VR_RAIL_E_ASIC_P0V75_MAX_S_VDD, 660, 660, 489, 1017, 1017, 690, 1017 },
	{ VR_RAIL_E_ASIC_P0V9_OWL_W_TRVDD, 170, 170, 595, 1111, 1111, 846, 1111 },
	{ VR_RAIL_E_ASIC_P0V75_OWL_W_TRVDD, 110, 110, 489, 1017, 1017, 705, 1017 },
};

const vr_test_mode_setting_t vr_test_mode_table_default[] = {
	// vr_rail, fast ocp: x/10(A), slow ocp: x/10(A), uvp: 1(mV), ovp: 1(mV), v max: 1(mV), lcr(mV), ucr(mV)
	{ VR_RAIL_E_ASIC_P0V85_MEDHA0_VDD, 13500, 13600, 640, 940, 900, 782, 918 },
	{ VR_RAIL_E_ASIC_P0V85_MEDHA1_VDD, 13500, 13600, 640, 940, 900, 782, 918 },
	{ VR_RAIL_E_ASIC_P0V9_OWL_E_TRVDD, 170, 160, 700, 1010, 954, 846, 954 },
	{ VR_RAIL_E_ASIC_P0V75_OWL_E_TRVDD, 110, 100, 575, 910, 795, 705, 795 },
	{ VR_RAIL_E_ASIC_P0V75_MAX_M_VDD, 670, 570, 575, 910, 848, 690, 848 },
	{ VR_RAIL_E_ASIC_P0V75_VDDPHY_HBM1357, 1230, 1130, 575, 910, 795, 690, 795 },
	{ VR_RAIL_E_ASIC_P0V75_OWL_E_VDD, 1600, 1500, 575, 910, 810, 690, 810 },
	{ VR_RAIL_E_ASIC_P0V4_VDDQL_HBM1357, 460, 360, 325, 800, 440, 368, 432 },
	{ VR_RAIL_E_ASIC_P1V1_VDDQC_HBM1357, 2800, 2700, 900, 1260, 1166, 1034, 1166 },
	{ VR_RAIL_E_ASIC_P1V8_VPP_HBM1357, 140, 140, 1400, 1960, 1900, 1692, 1908 },
	{ VR_RAIL_E_ASIC_P0V75_MAX_N_VDD, 670, 570, 575, 910, 848, 690, 847 },
	{ VR_RAIL_E_ASIC_P0V8_HAMSA_AVDD_PCIE, 500, 400, 635, 940, 940, 752, 848 },
	{ VR_RAIL_E_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE, 100, 90, 1000, 1300, 1250, 1128, 1272 },
	{ VR_RAIL_E_ASIC_P0V85_HAMSA_VDD, 850, 750, 640, 940, 900, 782, 900 },
	{ VR_RAIL_E_ASIC_P1V1_VDDQC_HBM0246, 2800, 2700, 900, 1260, 1166, 1034, 1166 },
	{ VR_RAIL_E_ASIC_P1V8_VPP_HBM0246, 140, 140, 1400, 1960, 1900, 1692, 1908 },
	{ VR_RAIL_E_ASIC_P0V4_VDDQL_HBM0246, 460, 360, 325, 800, 440, 368, 432 },
	{ VR_RAIL_E_ASIC_P0V75_VDDPHY_HBM0246, 1230, 1130, 575, 910, 795, 690, 795 },
	{ VR_RAIL_E_ASIC_P0V75_OWL_W_VDD, 1600, 1500, 575, 910, 810, 690, 810 },
	{ VR_RAIL_E_ASIC_P0V75_MAX_S_VDD, 580, 480, 575, 910, 848, 690, 847 },
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
			{ VR_SLOW_OCP_REG, cfg->fast_ocp, "SLOW OCP" },
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
void vr_test_mode_enable(bool onoff)
{
	vr_test_mode_flag = onoff;

	set_vr_test_mode_reg((onoff ? false : true));
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
	vr_test_mode_tid = k_thread_create(&vr_test_mode_thread, vr_test_mode_thread_stack,
					   K_THREAD_STACK_SIZEOF(vr_test_mode_thread_stack),
					   vr_test_mode_handler, NULL, NULL, NULL,
					   CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&vr_test_mode_thread, "vr_test_mode_mode_thread");
}