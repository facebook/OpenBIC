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
#include <zephyr.h>
#include <logging/log.h>
#include "libutil.h"
#include "power_status.h"
#include "plat_class.h"
#include "plat_gpio.h"
#include "plat_isr.h"
#include "plat_power_seq.h"

LOG_MODULE_REGISTER(plat_power_seq);

K_WORK_DELAYABLE_DEFINE(set_dc_on_5s_work, set_DC_on_delayed_status);
K_WORK_DELAYABLE_DEFINE(cxl_ready_thread, cxl_ready_handler);
K_WORK_DELAYABLE_DEFINE(enable_asic1_rst_work, enable_asic1_rst);
K_WORK_DELAYABLE_DEFINE(enable_asic2_rst_work, enable_asic2_rst);

static bool is_cxl_power_on[MAX_CXL_ID] = { false, false };
static bool is_cxl_ready[MAX_CXL_ID] = { false, false };

cxl_power_control_gpio cxl_power_ctrl_pin[MAX_CXL_ID] = {
	[0] = {
	.enclk_100m_osc = EN_CLK_100M_ASIC1_OSC,
	.p075v_asic_en = EN_P0V75_BIC_ASIC1_R,
	.p08v_asic_en = EN_P0V8_BIC_ASIC1_R,
	.p085v_asic_en = EN_P0V85_BIC_ASIC1_R,
	.p1v2_asic_en = EN_P1V2_BIC_ASIC1_R,
	.p1v8_asic_en = EN_P1V8_BIC_ASIC1_R,
	.sys_rst = SYS_RST_ASIC1_N_R,
	.pwr_on_rst = PWR_ON_RST_ASIC1_N,
	.pvpp_ab_dimm_en = EN_PVPP_AB_ASIC1_2V5_R,
	.pvpp_cd_dimm_en = EN_PVPP_CD_ASIC1_2V5_R,
	.pvddq_ab_dimm_en = EN_PVDDQ_AB_ASIC1_R,
	.pvddq_cd_dimm_en = EN_PVDDQ_CD_ASIC1_R,
	.pvtt_ab_dimm_en = EN_PVTT_AB_ASIC1_0V6_R,
	.pvtt_cd_dimm_en = EN_PVTT_CD_ASIC1_0V6_R, },
	[1] = { 
	.enclk_100m_osc = EN_CLK_100M_ASIC2_OSC,
	.p075v_asic_en = EN_P0V75_BIC_ASIC2_R,
	.p08v_asic_en = EN_P0V8_BIC_ASIC2_R,
	.p085v_asic_en = EN_P0V85_BIC_ASIC2_R,
	.p1v2_asic_en = EN_P1V2_BIC_ASIC2_R,
	.p1v8_asic_en = EN_P1V8_BIC_ASIC2_R,
	.sys_rst = SYS_RST_ASIC2_N_R,
	.pwr_on_rst = PWR_ON_RST_ASIC2_N,
	.pvpp_ab_dimm_en = EN_PVPP_AB_ASIC2_2V5_R,
	.pvpp_cd_dimm_en = EN_PVPP_CD_ASIC2_2V5_R,
	.pvddq_ab_dimm_en = EN_PVDDQ_AB_ASIC2_R,
	.pvddq_cd_dimm_en = EN_PVDDQ_CD_ASIC2_R,
	.pvtt_ab_dimm_en = EN_PVTT_AB_ASIC2_0V6_R,
	.pvtt_cd_dimm_en = EN_PVTT_CD_ASIC2_0V6_R, },
};

cxl_power_good_gpio cxl_power_good_pin[MAX_CXL_ID] = {
	[0] = {
	.p075v_asic_pg = PWRGD_P0V75_ASIC1,
	.p08v_asic_pg = PWRGD_P0V8_ASIC1,
	.p085v_asic_pg = PWRGD_P0V85_ASIC1,
	.p1v2_asic_pg = PWRGD_P1V2_ASIC1,
	.p1v8_asic_pg = PWRGD_P1V8_ASIC1,
	.pvpp_ab_dimm_pg = PWRGD_PVPP_AB_ASIC1,
	.pvpp_cd_dimm_pg = PWRGD_PVPP_CD_ASIC1,
	.pvddq_ab_dimm_pg = PWRGD_PVDDQ_AB_ASIC1,
	.pvddq_cd_dimm_pg = PWRGD_PVDDQ_CD_ASIC1,
	.pvtt_ab_dimm_pg = PWRGD_PVTT_AB_ASIC1,
	.pvtt_cd_dimm_pg = PWRGD_PVTT_AB_ASIC1, },
	[1] = { 
	.p075v_asic_pg = PWRGD_P0V75_ASIC2,
	.p08v_asic_pg = PWRGD_P0V8_ASIC2,
	.p085v_asic_pg = PWRGD_P0V85_ASIC2,
	.p1v2_asic_pg = PWRGD_P1V2_ASIC2,
	.p1v8_asic_pg = PWRGD_P1V8_ASIC2,
	.pvpp_ab_dimm_pg = PWRGD_PVPP_AB_ASIC2,
	.pvpp_cd_dimm_pg = PWRGD_PVPP_CD_ASIC2,
	.pvddq_ab_dimm_pg = PWRGD_PVDDQ_AB_ASIC2,
	.pvddq_cd_dimm_pg = PWRGD_PVDDQ_CD_ASIC2,
	.pvtt_ab_dimm_pg = PWRGD_PVTT_AB_ASIC2,
	.pvtt_cd_dimm_pg = PWRGD_PVTT_AB_ASIC2, },
};

void enable_asic1_rst()
{
	gpio_set(SYS_RST_ASIC1_N_R, POWER_ON);
	gpio_set(PWR_ON_RST_ASIC1_N, POWER_ON);
}

void enable_asic2_rst()
{
	gpio_set(SYS_RST_ASIC2_N_R, POWER_ON);
	gpio_set(PWR_ON_RST_ASIC2_N, POWER_ON);
}

void set_mb_dc_status(uint8_t gpio_num)
{
	static bool is_mb_dc_on = false;

	is_mb_dc_on = (gpio_get(gpio_num) == POWER_ON) ? true : false;
	LOG_INF("MB DC (gpio num %d) status is %s", gpio_num, is_mb_dc_on ? "ON" : "OFF");
}

void execute_power_on_sequence()
{
	int ret = 0;

	// If CXL is on, doesn't need to power on again
	if (get_DC_status()) {
		LOG_INF("CXL DC status is ON");
		return;
	}

	// TODO: check E1S present
	if (get_board_revision() == BOARD_POC) {
		gpio_set(POC_EN_P3V3_E1S_0_R, POWER_ON);
	} else {
		gpio_set(EN_P3V3_E1S_0_R, POWER_ON);
	}
	gpio_set(EN_P12V_E1S_0_R, POWER_ON);

	ret = power_on_handler(CXL_ID_0, CLK_POWER_ON_STAGE);
	if (ret == 0) {
		is_cxl_power_on[CXL_ID_0] = true;
		LOG_INF("CXL 1 power on success");
	} else {
		is_cxl_power_on[CXL_ID_0] = false;
		LOG_ERR("CXL 1 power on fail");
	}

	ret = power_on_handler(CXL_ID_1, CLK_POWER_ON_STAGE);
	if (ret == 0) {
		is_cxl_power_on[CXL_ID_1] = true;
		LOG_INF("CXL 2 power on success");
	} else {
		is_cxl_power_on[CXL_ID_1] = false;
		LOG_ERR("CXL 2 power on fail");
	}

	if (is_cxl_power_on[CXL_ID_0] && is_cxl_power_on[CXL_ID_1]) {
		gpio_set(PG_CARD_OK, POWER_ON);
		set_DC_status(PG_CARD_OK);
		k_work_schedule(&set_dc_on_5s_work, K_SECONDS(DC_ON_DELAY5_SEC));
		k_work_schedule(&cxl_ready_thread, K_SECONDS(CXL_READY_SECONDS));
	}
}

int power_on_handler(int cxl_id, int power_stage)
{
	int ret = 0;
	int ctrl_stage = 0;

	for (ctrl_stage = power_stage; ctrl_stage < MAX_POWER_ON_STAGES; ctrl_stage++) {
		// Set power enable pin to enable power
		enable_powers(cxl_id, ctrl_stage);

		if (ctrl_stage != CLK_POWER_ON_STAGE) {
			k_msleep(CHK_PWR_DELAY_MSEC);
		}

		// Get power good pin to check power
		ret = check_powers_enabled(cxl_id, ctrl_stage);
		if (ret < 0) {
			break;
		}
	}

	return ret;
}

void enable_powers(int cxl_id, int pwr_stage)
{
	switch (pwr_stage) {
	case CLK_POWER_ON_STAGE:
		gpio_set(cxl_power_ctrl_pin[cxl_id].enclk_100m_osc, POWER_ON);
		k_msleep(SYS_CLK_STABLE_DELAY_MSEC);
		break;
	case ASIC_POWER_ON_STAGE_1:
		gpio_set(cxl_power_ctrl_pin[cxl_id].p075v_asic_en, POWER_ON);
		gpio_set(cxl_power_ctrl_pin[cxl_id].p08v_asic_en, POWER_ON);
		gpio_set(cxl_power_ctrl_pin[cxl_id].p085v_asic_en, POWER_ON);
		break;
	case ASIC_POWER_ON_STAGE_2:
		gpio_set(cxl_power_ctrl_pin[cxl_id].p1v2_asic_en, POWER_ON);
		gpio_set(cxl_power_ctrl_pin[cxl_id].p1v8_asic_en, POWER_ON);
		break;
	case DIMM_POWER_ON_STAGE_1:
		gpio_set(cxl_power_ctrl_pin[cxl_id].pvpp_ab_dimm_en, POWER_ON);
		gpio_set(cxl_power_ctrl_pin[cxl_id].pvpp_cd_dimm_en, POWER_ON);
		break;
	case DIMM_POWER_ON_STAGE_2:
		gpio_set(cxl_power_ctrl_pin[cxl_id].pvddq_ab_dimm_en, POWER_ON);
		gpio_set(cxl_power_ctrl_pin[cxl_id].pvddq_cd_dimm_en, POWER_ON);
		break;
	case DIMM_POWER_ON_STAGE_3:
		gpio_set(cxl_power_ctrl_pin[cxl_id].pvtt_ab_dimm_en, POWER_ON);
		gpio_set(cxl_power_ctrl_pin[cxl_id].pvtt_cd_dimm_en, POWER_ON);
		switch (cxl_id) {
		case CXL_ID_0:
			k_work_schedule(&enable_asic1_rst_work, K_MSEC(PWR_RST_DELAY_MSEC));
			break;
		case CXL_ID_1:
			k_work_schedule(&enable_asic2_rst_work, K_MSEC(PWR_RST_DELAY_MSEC));
			break;
		default:
			LOG_ERR("Unknown CXL id %d to enable PWR_ON_RST", cxl_id);
			break;
		}
		break;
	default:
		LOG_ERR("Unknown stage 0x%x to enable power", pwr_stage);
		break;
	}
}

int check_powers_enabled(int cxl_id, int pwr_stage)
{
	switch (pwr_stage) {
	case CLK_POWER_ON_STAGE:
		// Doesn't need to check
		break;
	case ASIC_POWER_ON_STAGE_1:
		if (!is_power_controlled(cxl_id, cxl_power_good_pin[cxl_id].p075v_asic_pg, POWER_ON,
					 "P0V75_ASIC")) {
			return -1;
		}

		if (!is_power_controlled(cxl_id, cxl_power_good_pin[cxl_id].p08v_asic_pg, POWER_ON,
					 "P0V8_ASIC")) {
			return -1;
		}

		if (!is_power_controlled(cxl_id, cxl_power_good_pin[cxl_id].p085v_asic_pg, POWER_ON,
					 "P0V85_ASIC")) {
			return -1;
		}
		break;
	case ASIC_POWER_ON_STAGE_2:
		if (!is_power_controlled(cxl_id, cxl_power_good_pin[cxl_id].p1v2_asic_pg, POWER_ON,
					 "P1V2_ASIC")) {
			return -1;
		}

		if (!is_power_controlled(cxl_id, cxl_power_good_pin[cxl_id].p1v8_asic_pg, POWER_ON,
					 "P1V8_ASIC")) {
			return -1;
		}
		break;
	case DIMM_POWER_ON_STAGE_1:
		if (!is_power_controlled(cxl_id, cxl_power_good_pin[cxl_id].pvpp_ab_dimm_pg,
					 POWER_ON, "PVPP_AB")) {
			return -1;
		}

		if (!is_power_controlled(cxl_id, cxl_power_good_pin[cxl_id].pvpp_cd_dimm_pg,
					 POWER_ON, "PVPP_CD")) {
			return -1;
		}
		break;
	case DIMM_POWER_ON_STAGE_2:
		if (!is_power_controlled(cxl_id, cxl_power_good_pin[cxl_id].pvddq_ab_dimm_pg,
					 POWER_ON, "PVDDQ_AB")) {
			return -1;
		}

		if (!is_power_controlled(cxl_id, cxl_power_good_pin[cxl_id].pvddq_cd_dimm_pg,
					 POWER_ON, "PVDDQ_CD")) {
			return -1;
		}
		break;
	case DIMM_POWER_ON_STAGE_3:
		if (!is_power_controlled(cxl_id, cxl_power_good_pin[cxl_id].pvtt_ab_dimm_pg,
					 POWER_ON, "PVTT_AB")) {
			return -1;
		}

		if (!is_power_controlled(cxl_id, cxl_power_good_pin[cxl_id].pvtt_cd_dimm_pg,
					 POWER_ON, "PVTT_CD")) {
			return -1;
		}
		break;
	default:
		LOG_ERR("Unknown stage 0x%x to check power on", pwr_stage);
		return -1;
	}

	return 0;
}

bool is_power_controlled(int cxl_id, int power_pin, uint8_t check_power_status, char *power_name)
{
	if (gpio_get(power_pin) == check_power_status) {
		return true;
	} else {
		// TODO: Add event to BMC
		LOG_ERR("Failed to power %s CXL %d %s (gpio num %d)",
			check_power_status ? "on" : "off", cxl_id + 1, power_name, power_pin);
		return false;
	}
}

void execute_power_off_sequence()
{
	int ret = 0;

	// If CXL is off, doesn't need to power off again
	if (!get_DC_status()) {
		LOG_INF("CXL DC status is OFF");
		return;
	}

	// TODO: check E1S present
	if (get_board_revision() == BOARD_POC) {
		gpio_set(POC_EN_P3V3_E1S_0_R, POWER_OFF);
	} else {
		gpio_set(EN_P3V3_E1S_0_R, POWER_OFF);
	}
	gpio_set(EN_P12V_E1S_0_R, POWER_OFF);

	gpio_set(PG_CARD_OK, POWER_OFF);
	set_DC_status(PG_CARD_OK);

	if (k_work_cancel_delayable(&set_dc_on_5s_work) != 0) {
		LOG_ERR("Cancel set dc off delay work fail");
	}

	if (k_work_cancel_delayable(&cxl_ready_thread) != 0) {
		LOG_ERR("Failed to cancel cxl_ready_thread");
	}

	set_DC_on_delayed_status();

	is_cxl_ready[CXL_ID_0] = false;
	ret = power_off_handler(CXL_ID_0, DIMM_POWER_OFF_STAGE_1);
	if (ret == 0) {
		is_cxl_power_on[CXL_ID_0] = false;
		LOG_INF("CXL 1 power off success");
	} else {
		is_cxl_power_on[CXL_ID_0] = true;
		LOG_ERR("CXL 1 power off fail");
	}

	is_cxl_ready[CXL_ID_1] = false;
	ret = power_off_handler(CXL_ID_1, DIMM_POWER_OFF_STAGE_1);
	if (ret == 0) {
		is_cxl_power_on[CXL_ID_1] = false;
		LOG_INF("CXL 2 power off success");
	} else {
		is_cxl_power_on[CXL_ID_1] = true;
		LOG_ERR("CXL 2 power off fail");
	}

	uint8_t ioe2_output_value = 0;

	if (get_ioe_value(ADDR_IOE2, TCA9555_OUTPUT_PORT_REG_0, &ioe2_output_value) == 0) {
		CLEARBITS(ioe2_output_value, IOE_P00,
			  IOE_P03) // Disable P0~P3 to switch mux to CXL.
		set_ioe_value(ADDR_IOE2, TCA9555_OUTPUT_PORT_REG_0, ioe2_output_value);
		set_vr_monitor_status(false);
	}
}

int power_off_handler(int cxl_id, int power_stage)
{
	int ret = 0;
	int ctrl_stage = 0;

	for (ctrl_stage = power_stage; ctrl_stage < MAX_POWER_OFF_STAGES; ctrl_stage++) {
		// Set power enable pin to enable power
		disable_powers(cxl_id, ctrl_stage);

		k_msleep(CHK_PWR_DELAY_MSEC);

		// Get power good pin to check power
		ret = check_powers_disabled(cxl_id, ctrl_stage);
		if (ret < 0) {
			break;
		}
	}

	return ret;
}

void disable_powers(int cxl_id, int pwr_stage)
{
	switch (pwr_stage) {
	case DIMM_POWER_OFF_STAGE_1:
		gpio_set(cxl_power_ctrl_pin[cxl_id].pvtt_ab_dimm_en, POWER_OFF);
		gpio_set(cxl_power_ctrl_pin[cxl_id].pvtt_cd_dimm_en, POWER_OFF);
		break;
	case DIMM_POWER_OFF_STAGE_2:
		gpio_set(cxl_power_ctrl_pin[cxl_id].pvddq_ab_dimm_en, POWER_OFF);
		gpio_set(cxl_power_ctrl_pin[cxl_id].pvddq_cd_dimm_en, POWER_OFF);
		break;
	case DIMM_POWER_OFF_STAGE_3:
		gpio_set(cxl_power_ctrl_pin[cxl_id].pvpp_ab_dimm_en, POWER_OFF);
		gpio_set(cxl_power_ctrl_pin[cxl_id].pvpp_cd_dimm_en, POWER_OFF);
		break;
	case ASIC_POWER_OFF_STAGE_1:
		gpio_set(cxl_power_ctrl_pin[cxl_id].sys_rst, POWER_OFF);
		gpio_set(cxl_power_ctrl_pin[cxl_id].pwr_on_rst, POWER_OFF);
		gpio_set(cxl_power_ctrl_pin[cxl_id].p08v_asic_en, POWER_OFF);
		gpio_set(cxl_power_ctrl_pin[cxl_id].p1v2_asic_en, POWER_OFF);
		gpio_set(cxl_power_ctrl_pin[cxl_id].p1v8_asic_en, POWER_OFF);
		break;
	case ASIC_POWER_OFF_STAGE_2: {
		// Check ASIC_1V8 discharge completed before power off P0V85_ASIC
		k_msleep(P1V8_POWER_OFF_DELAY_MSEC);

		gpio_set(cxl_power_ctrl_pin[cxl_id].p085v_asic_en, POWER_OFF);
		break;
	}
	case ASIC_POWER_OFF_STAGE_3:
		gpio_set(cxl_power_ctrl_pin[cxl_id].p075v_asic_en, POWER_OFF);
		break;
	case CLK_POWER_OFF_STAGE:
		gpio_set(cxl_power_ctrl_pin[cxl_id].enclk_100m_osc, POWER_OFF);
		break;
	default:
		LOG_ERR("Unknown stage 0x%x to disable power", pwr_stage);
		break;
	}
}

int check_powers_disabled(int cxl_id, int pwr_stage)
{
	switch (pwr_stage) {
	case DIMM_POWER_OFF_STAGE_1:
		if (!is_power_controlled(cxl_id, cxl_power_good_pin[cxl_id].pvtt_ab_dimm_pg,
					 POWER_OFF, "PVTT_AB")) {
			return -1;
		}

		if (!is_power_controlled(cxl_id, cxl_power_good_pin[cxl_id].pvtt_cd_dimm_pg,
					 POWER_OFF, "PVTT_CD")) {
			return -1;
		}
		break;
	case DIMM_POWER_OFF_STAGE_2:
		if (!is_power_controlled(cxl_id, cxl_power_good_pin[cxl_id].pvddq_ab_dimm_pg,
					 POWER_OFF, "PVDDQ_AB")) {
			return -1;
		}

		if (!is_power_controlled(cxl_id, cxl_power_good_pin[cxl_id].pvddq_cd_dimm_pg,
					 POWER_OFF, "PVDDQ_CD")) {
			return -1;
		}
		break;
	case DIMM_POWER_OFF_STAGE_3:
		if (!is_power_controlled(cxl_id, cxl_power_good_pin[cxl_id].pvpp_ab_dimm_pg,
					 POWER_OFF, "PVPP_AB")) {
			return -1;
		}

		if (!is_power_controlled(cxl_id, cxl_power_good_pin[cxl_id].pvpp_cd_dimm_pg,
					 POWER_OFF, "PVPP_CD")) {
			return -1;
		}
		break;
	case ASIC_POWER_OFF_STAGE_1:
		if (!is_power_controlled(cxl_id, cxl_power_good_pin[cxl_id].p08v_asic_pg, POWER_OFF,
					 "P0V8_ASIC")) {
			return -1;
		}

		if (!is_power_controlled(cxl_id, cxl_power_good_pin[cxl_id].p1v2_asic_pg, POWER_OFF,
					 "P1V2_ASIC")) {
			return -1;
		}

		if (!is_power_controlled(cxl_id, cxl_power_good_pin[cxl_id].p1v8_asic_pg, POWER_OFF,
					 "P1V8_ASIC")) {
			return -1;
		}
		break;
	case ASIC_POWER_OFF_STAGE_2:
		if (!is_power_controlled(cxl_id, cxl_power_good_pin[cxl_id].p085v_asic_pg,
					 POWER_OFF, "P0V85_ASIC")) {
			return -1;
		}
		break;
	case ASIC_POWER_OFF_STAGE_3:
		if (!is_power_controlled(cxl_id, cxl_power_good_pin[cxl_id].p075v_asic_pg,
					 POWER_OFF, "P0V75_ASIC")) {
			return -1;
		}
		break;
	case CLK_POWER_OFF_STAGE:
		break;
	default:
		LOG_ERR("Unknown stage 0x%x to check power on", pwr_stage);
		return -1;
	}

	return 0;
}

void cxl_ready_handler()
{
	/* TODO:
	 * In normal states, DIMM and PMIC muxs should be switch to BIC after checking CXL heartbeat is ready. However, WF's heartbeat is not ready yet
	 * ,so we need to make the workaround for switch muxs.
	 */

	uint8_t value = 0x0;

	if (get_ioe_value(ADDR_IOE2, TCA9555_OUTPUT_PORT_REG_0, &value) == 0) {
		value |= IOE_SWITCH_MUX_TO_BIC; // Enable P0~P3 to switch mux to BIC.
		set_ioe_value(ADDR_IOE2, TCA9555_OUTPUT_PORT_REG_0, value);
		set_vr_monitor_status(true);
	}

	is_cxl_ready[CXL_ID_0] = true;
	LOG_INF("CXL1 is ready");
	is_cxl_ready[CXL_ID_1] = true;
	LOG_INF("CXL2 is ready");

	return;
}

bool get_cxl_ready_status(uint8_t cxl_id)
{
	return is_cxl_ready[cxl_id];
}

bool cxl_ready_access(uint8_t sensor_num)
{
	uint8_t cxl_id = sensor_num / 4;
	return get_cxl_ready_status(cxl_id);
}
