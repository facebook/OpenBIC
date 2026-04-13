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
#include "plat_vr_test_mode.h"
#include "plat_class.h"
#include "plat_hook.h"
#include "plat_hamsa_avdd_pcie.h"

LOG_MODULE_REGISTER(plat_hamsa_avdd_pcie);

const vr_rns_hamsa_avdd_pcie_mode_setting_t vr_rns_hamsa_avdd_pcie_mode_table[] = {
	// vr_rail, uvp: 1(mV), ovp: 1(mV), v max: 1(mV), lcr(mV), ucr(mV)
	{ VR_RAIL_E_ASIC_P0V8_HAMSA_AVDD_PCIE, 635, 940, 880, 770, 880 },
};

const vr_mps_hamsa_avdd_pcie_mode_setting_t vr_mps_hamsa_avdd_pcie_mode_table[] = {
	/*
	uint8_t vr_rail;
	uint16_t lcr;
	uint16_t ucr;
	uint16_t vout_max;
	*/
	{ VR_RAIL_E_ASIC_P0V8_HAMSA_AVDD_PCIE, 770, 880, 880 },
};

const uint8_t vr_rns_hamsa_avdd_pcie_mode_table_size =
	ARRAY_SIZE(vr_rns_hamsa_avdd_pcie_mode_table);

const uint8_t vr_mps_hamsa_avdd_pcie_mode_table_size =
	ARRAY_SIZE(vr_mps_hamsa_avdd_pcie_mode_table);

bool set_hamsa_avdd_pcie(uint16_t *millivolt, bool is_perm)
{
	uint8_t vr = get_vr_module();
	if (vr == VR_MODULE_MPS) {
		for (uint8_t i = 0; i < vr_mps_hamsa_avdd_pcie_mode_table_size; i++) {
			const vr_mps_hamsa_avdd_pcie_mode_setting_t *cfg =
				&vr_mps_hamsa_avdd_pcie_mode_table[i];

			uint16_t action = 0;
			uint16_t div_en = 0;
			action = LATCH_OFF;
			div_en = ENABLE;
			// set ovp2 action to no action
			if (set_vr_mp2971_reg(cfg->vr_rail, &action, OVP_2_ACTION))
				LOG_ERR("set vr %d ovp2 action fail!", cfg->vr_rail);
			// set divider enable/disable
			if (set_vr_mp2971_reg(cfg->vr_rail, &div_en, DIV_EN))
				LOG_ERR("set vr %d divider enable/disable fail!", cfg->vr_rail);

			const struct {
				uint8_t function;
				uint16_t val;
				const char *name;
			} regs[] = {
				{ VOUT_MAX, cfg->vout_max, "VOUT MAX" },
			};

			uint16_t set_val = 0;
			for (size_t j = 0; j < ARRAY_SIZE(regs); j++) {
				set_val = regs[j].val;
				if (set_vr_mp2971_reg(cfg->vr_rail, &set_val, regs[j].function)) {
					LOG_ERR("MPS2971 VR rail %x set %s to %d failed",
						cfg->vr_rail, regs[j].name, regs[j].val);
				}
			}

			if (!plat_set_vout_command(cfg->vr_rail, millivolt, is_perm)) {
				LOG_ERR("VR rail %x set vout to %d failed", cfg->vr_rail,
					*millivolt);
			}
		}
	} else if (vr == VR_MODULE_RNS) {
		for (uint8_t i = 0; i < vr_rns_hamsa_avdd_pcie_mode_table_size; i++) {
			const vr_rns_hamsa_avdd_pcie_mode_setting_t *cfg =
				&vr_rns_hamsa_avdd_pcie_mode_table[i];

			const struct {
				uint8_t offset;
				uint16_t val;
				const char *name;
			} regs[] = {
				{ VR_UVP_REG, cfg->uvp, "UVP" },
				{ VR_OVP_REG, cfg->ovp, "OVP" },
				{ VR_VOUT_MAX_REG, cfg->vout_max, "VOUT MAX" },
			};

			for (size_t j = 0; j < ARRAY_SIZE(regs); j++) {
				if (!update_vr_reg(cfg->vr_rail, regs[j].offset, regs[j].val)) {
					LOG_ERR("VR rail %x set %s to %d failed", cfg->vr_rail,
						regs[j].name, regs[j].val);
				}
			}

			if (!plat_set_vout_command(cfg->vr_rail, millivolt, is_perm)) {
				LOG_ERR("VR rail %x set vout to %d failed", cfg->vr_rail,
					*millivolt);
			}

			if (!set_vr_fixed_uvp_ovp_enable(cfg->vr_rail, true))
				LOG_ERR("set vr %d fix uvp/ovp enable fail!", i);
		}
	} else {
		LOG_ERR("VR module not support");
		return false;
	}

	return true;
}