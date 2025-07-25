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
#include <drivers/sensor.h>
#include <drivers/pwm.h>
#include "libutil.h"
#include "power_status.h"
#include "plat_class.h"
#include "plat_gpio.h"
#include "plat_isr.h"
#include "plat_power_seq.h"
#include "plat_mctp.h"
#include "plat_pldm_sensor.h"
#include "util_sys.h"

LOG_MODULE_REGISTER(plat_power_seq);

K_WORK_DELAYABLE_DEFINE(set_dc_on_5s_work, set_DC_on_delayed_status);
K_WORK_DELAYABLE_DEFINE(set_cxl1_vr_ready_work, set_cxl1_vr_access_delayed_status);
K_WORK_DELAYABLE_DEFINE(set_cxl2_vr_ready_work, set_cxl2_vr_access_delayed_status);
K_WORK_DELAYABLE_DEFINE(cxl1_ready_thread, cxl1_ready_handler);
K_WORK_DELAYABLE_DEFINE(cxl2_ready_thread, cxl2_ready_handler);
K_WORK_DELAYABLE_DEFINE(cxl1_hb_monitor_work, cxl1_heartbeat_monitor_handler);
K_WORK_DELAYABLE_DEFINE(cxl2_hb_monitor_work, cxl2_heartbeat_monitor_handler);
K_TIMER_DEFINE(enable_asic1_rst_timer, enable_asic1_rst, NULL);
K_TIMER_DEFINE(enable_asic2_rst_timer, enable_asic2_rst, NULL);

#define CXL_READY_HANDLER_STACK_SIZE 1024
#define VR_EVENT_WAIT_BIC_RESET_DELAY_MS (30 * 1000) // 30 seconds
K_THREAD_STACK_DEFINE(cxl1_stack_area, CXL_READY_HANDLER_STACK_SIZE);
struct k_thread cxl1_thread_data;

K_THREAD_STACK_DEFINE(cxl2_stack_area, CXL_READY_HANDLER_STACK_SIZE);
struct k_thread cxl2_thread_data;

K_MUTEX_DEFINE(switch_ioe_mux_mutex);

static bool is_cxl_power_on[MAX_CXL_ID] = { false, false };
static bool is_cxl_ready[MAX_CXL_ID] = { false, false };
static bool is_cxl_vr_accessible[MAX_CXL_ID] = { false, false };
bool is_cxl_power_on_success = false;

static k_tid_t cxl1_tid = NULL;
static k_tid_t cxl2_tid = NULL;

enum { HB_STATE_UNKNOWN = 0, HB_STATE_OK, HB_STATE_LOW };
uint8_t cxl1_hb_state = HB_STATE_UNKNOWN;
uint8_t cxl2_hb_state = HB_STATE_UNKNOWN;

extern struct k_work_q plat_work_q;
void init_cxl_heartbeat_monitor_work()
{
	k_work_init_delayable(&cxl1_hb_monitor_work, cxl1_heartbeat_monitor_handler);
	k_work_init_delayable(&cxl2_hb_monitor_work, cxl2_heartbeat_monitor_handler);
}

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

add_sel_info cxl_event_work_items[] = {
	{
		.is_init = false,
		.assert_type = EVENT_DEASSERTED,
	},
	{
		.is_init = false,
		.assert_type = EVENT_DEASSERTED,
	},
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

uint32_t get_uptime_secs(void)
{
	return k_uptime_get_32() / 1000U;
}

void execute_power_on_sequence()
{
	int ret = 0;

	// If CXL is on, doesn't need to power on again
	if (gpio_get(PG_CARD_OK) == POWER_ON) {
		LOG_INF("CXL DC status is ON");
		return;
	}

	// TODO: check E1S present
	if (get_board_revision() == BOARD_POC) {
		gpio_set(POC_EN_P3V3_E1S_0_R, POWER_ON);
		set_P3V3_E1S_power_status(POC_PWRGD_P3V3_E1S_0_R);
	} else {
		gpio_set(EN_P3V3_E1S_0_R, POWER_ON);
		set_P3V3_E1S_power_status(PWRGD_P3V3_E1S_0_R);
	}
	gpio_set(EN_P12V_E1S_0_R, POWER_ON);
	set_P12V_E1S_power_status(PWRGD_P12V_E1S_0_R);

	ret = power_on_handler(CXL_ID_1, ASIC_POWER_ON_STAGE_1);
	if (ret == 0) {
		is_cxl_power_on[CXL_ID_1] = true;
		LOG_INF("CXL 1 power on success");
	} else {
		is_cxl_power_on[CXL_ID_1] = false;
		LOG_ERR("CXL 1 power on fail");
	}

	ret = power_on_handler(CXL_ID_2, ASIC_POWER_ON_STAGE_1);
	if (ret == 0) {
		is_cxl_power_on[CXL_ID_2] = true;
		LOG_INF("CXL 2 power on success");
	} else {
		is_cxl_power_on[CXL_ID_2] = false;
		LOG_ERR("CXL 2 power on fail");
	}

	if (is_cxl_power_on[CXL_ID_1] && is_cxl_power_on[CXL_ID_2]) {
		is_cxl_power_on_success = true;
		gpio_set(PG_CARD_OK, POWER_ON);
		set_DC_status(PG_CARD_OK);
		k_work_schedule(&set_dc_on_5s_work, K_SECONDS(DC_ON_DELAY5_SEC));

		create_check_cxl_ready_thread();
	} else {
		LOG_INF("CXL power on failed, switch IOE mux to BIC");
		switch_mux_to_bic(IOE_SWITCH_CXL1_VR_TO_BIC);
		switch_mux_to_bic(IOE_SWITCH_CXL2_VR_TO_BIC);
		uint32_t uptime = get_uptime_secs();
		uint32_t delay_ms = VR_EVENT_DELAY_MS;

		if (uptime < 30) { //wait longer for BIC reset
			delay_ms = VR_EVENT_WAIT_BIC_RESET_DELAY_MS;
			LOG_INF("pldm not ready yet, wait for %u seconds before checking VR",
				(delay_ms / 1000U));
		}

		k_work_schedule_for_queue(&plat_work_q,
					  &vr_event_work_items[PMBUS_VR_IOE1_INT].add_sel_work,
					  K_MSEC(delay_ms));
	}
}

void create_check_cxl_ready_thread()
{
	if ((cxl1_tid != NULL) && ((strcmp(k_thread_state_str(cxl1_tid), "dead") != 0) &&
				   (strcmp(k_thread_state_str(cxl1_tid), "unknown") != 0))) {
		k_thread_abort(cxl1_tid);
	}

	cxl1_tid = k_thread_create(&cxl1_thread_data, cxl1_stack_area,
				   K_THREAD_STACK_SIZEOF(cxl1_stack_area), cxl1_ready_handler, NULL,
				   NULL, NULL, CONFIG_MAIN_THREAD_PRIORITY + 1, 0, K_NO_WAIT);
	k_thread_name_set(cxl1_tid, "cxl1_ready_thread");
	k_thread_start(cxl1_tid);

	if ((cxl2_tid != NULL) && ((strcmp(k_thread_state_str(cxl2_tid), "dead") != 0) &&
				   (strcmp(k_thread_state_str(cxl2_tid), "unknown") != 0))) {
		k_thread_abort(cxl2_tid);
	}

	cxl2_tid = k_thread_create(&cxl2_thread_data, cxl2_stack_area,
				   K_THREAD_STACK_SIZEOF(cxl2_stack_area), cxl2_ready_handler, NULL,
				   NULL, NULL, CONFIG_MAIN_THREAD_PRIORITY + 1, 0, K_NO_WAIT);
	k_thread_name_set(cxl2_tid, "cxl2_ready_thread");
	k_thread_start(cxl2_tid);
}

int power_on_handler(int cxl_id, int power_stage)
{
	int ret = 0;
	int ctrl_stage = 0;

	for (ctrl_stage = power_stage; ctrl_stage < MAX_POWER_ON_STAGES; ctrl_stage++) {
		// Set power enable pin to enable power
		enable_powers(cxl_id, ctrl_stage);

		// Get power good pin to check power
		ret = check_powers_enabled(cxl_id, ctrl_stage);
		if (ret < 0) {
			LOG_ERR("CXL %d PwrOn:%d Fail", cxl_id, ctrl_stage);
			break;
		}
	}

	return ret;
}

void enable_powers(int cxl_id, int pwr_stage)
{
	switch (pwr_stage) {
	case ASIC_POWER_ON_STAGE_1:
		gpio_set(cxl_power_ctrl_pin[cxl_id].p075v_asic_en, POWER_ON);
		gpio_set(cxl_power_ctrl_pin[cxl_id].p085v_asic_en, POWER_ON);
		gpio_set(cxl_power_ctrl_pin[cxl_id].p1v8_asic_en, POWER_ON);
		break;
	case CLK_POWER_ON_STAGE:
		gpio_set(cxl_power_ctrl_pin[cxl_id].enclk_100m_osc, POWER_ON);
		break;
	case ASIC_POWER_ON_STAGE_2:
		gpio_set(cxl_power_ctrl_pin[cxl_id].p08v_asic_en, POWER_ON);
		break;
	case ASIC_POWER_ON_STAGE_3:
		gpio_set(cxl_power_ctrl_pin[cxl_id].p1v2_asic_en, POWER_ON);
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
		case CXL_ID_1:
			k_timer_start(&enable_asic1_rst_timer, K_MSEC(PWR_RST_DELAY_MSEC),
				      K_NO_WAIT);
			break;
		case CXL_ID_2:
			k_timer_start(&enable_asic2_rst_timer, K_MSEC(PWR_RST_DELAY_MSEC),
				      K_NO_WAIT);
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
	case ASIC_POWER_ON_STAGE_1:
		if (!is_power_controlled(cxl_id, cxl_power_good_pin[cxl_id].p075v_asic_pg, POWER_ON,
					 "P0V75_ASIC")) {
			return -1;
		}
		if (!is_power_controlled(cxl_id, cxl_power_good_pin[cxl_id].p085v_asic_pg, POWER_ON,
					 "P0V85_ASIC")) {
			return -1;
		}
		if (!is_power_controlled(cxl_id, cxl_power_good_pin[cxl_id].p1v8_asic_pg, POWER_ON,
					 "P1V8_ASIC")) {
			return -1;
		}
		break;
	case CLK_POWER_ON_STAGE:
		// Doesn't need to check
		break;
	case ASIC_POWER_ON_STAGE_2:
		if (!is_power_controlled(cxl_id, cxl_power_good_pin[cxl_id].p08v_asic_pg, POWER_ON,
					 "P0V8_ASIC")) {
			return -1;
		}
		break;
	case ASIC_POWER_ON_STAGE_3:
		if (!is_power_controlled(cxl_id, cxl_power_good_pin[cxl_id].p1v2_asic_pg, POWER_ON,
					 "P1V2_ASIC")) {
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
	int retry_times = 5, i = 0;
	for (i = 0; i < retry_times; i++) {
		k_msleep(CHK_PWR_DELAY_MSEC);
		// Get power good pin to check power
		if (gpio_get(power_pin) == check_power_status) {
			break;
		}
	}

	if (i >= retry_times) {
		// TODO: Add event to BMC
		LOG_ERR("Failed to power %s CXL %d %s (gpio num %d)",
			check_power_status ? "on" : "off", cxl_id + 1, power_name, power_pin);
		return false;
	} else {
		return true;
	}
}

void execute_power_off_sequence()
{
	int ret = 0;

	// If CXL is off, doesn't need to power off again
	if (gpio_get(PG_CARD_OK) == POWER_OFF) {
		LOG_INF("CXL DC status is OFF");
		return;
	}

	// TODO: check E1S present
	if (get_board_revision() == BOARD_POC) {
		gpio_set(POC_EN_P3V3_E1S_0_R, POWER_OFF);
		set_P3V3_E1S_power_status(POC_PWRGD_P3V3_E1S_0_R);
	} else {
		gpio_set(EN_P3V3_E1S_0_R, POWER_OFF);
		set_P3V3_E1S_power_status(PWRGD_P3V3_E1S_0_R);
	}
	gpio_set(EN_P12V_E1S_0_R, POWER_OFF);
	set_P12V_E1S_power_status(PWRGD_P12V_E1S_0_R);

	is_cxl_ready[CXL_ID_1] = false;
	is_cxl_ready[CXL_ID_2] = false;

	gpio_set(PG_CARD_OK, POWER_OFF);
	set_DC_status(PG_CARD_OK);

	if (k_work_cancel_delayable(&set_dc_on_5s_work) != 0) {
		LOG_ERR("Cancel set dc off delay work fail");
	}

	if (k_work_cancel_delayable(&cxl1_ready_thread) != 0) {
		LOG_ERR("Failed to cancel cxl1_ready_thread");
	}

	if (k_work_cancel_delayable(&cxl2_ready_thread) != 0) {
		LOG_ERR("Failed to cancel cxl2_ready_thread");
	}

	if (k_work_cancel_delayable(&set_cxl1_vr_ready_work) != 0) {
		LOG_WRN("Failed to cancel set_cxl1_vr_ready_work");
	}

	if (k_work_cancel_delayable(&set_cxl2_vr_ready_work) != 0) {
		LOG_WRN("Failed to cancel set_cxl2_vr_ready_work");
	}

	if (k_work_cancel_delayable(&cxl1_hb_monitor_work) != 0) {
		LOG_ERR("Failed to cancel cxl1_hb_monitor_work");
	}

	cxl1_hb_state = HB_STATE_UNKNOWN;

	if (k_work_cancel_delayable(&cxl2_hb_monitor_work) != 0) {
		LOG_ERR("Failed to cancel cxl2_hb_monitor_work");
	}

	cxl2_hb_state = HB_STATE_UNKNOWN;

	set_DC_on_delayed_status_with_value(false);

	ret = power_off_handler(CXL_ID_1, DIMM_POWER_OFF_STAGE_1);
	if (ret == 0) {
		is_cxl_power_on[CXL_ID_1] = false;
		LOG_INF("CXL 1 power off success");
	} else {
		is_cxl_power_on[CXL_ID_1] = true;
		LOG_ERR("CXL 1 power off fail");
	}

	ret = power_off_handler(CXL_ID_2, DIMM_POWER_OFF_STAGE_1);
	if (ret == 0) {
		is_cxl_power_on[CXL_ID_2] = false;
		LOG_INF("CXL 2 power off success");
	} else {
		is_cxl_power_on[CXL_ID_2] = true;
		LOG_ERR("CXL 2 power off fail");
	}

	uint8_t ioe2_output_value = 0;

	if (get_ioe_value(ADDR_IOE2, TCA9555_OUTPUT_PORT_REG_0, &ioe2_output_value) == 0) {
		CLEARBITS(ioe2_output_value, IOE_P00,
			  IOE_P03) // Disable P0~P3 to switch mux to CXL.
		set_ioe_value(ADDR_IOE2, TCA9555_OUTPUT_PORT_REG_0, ioe2_output_value);
		set_cxl_vr_access(MAX_CXL_ID, false);
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
			LOG_ERR("CXL %d PwrOff:%d Fail", cxl_id, ctrl_stage);
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

static bool is_wf_ASIC1_vr_clear_fault_done = false;
static bool is_wf_ASIC2_vr_clear_fault_done = false;

static bool is_wf_IOE_INT_reset_done = false;
extern uint8_t ioe_list[4];

void clear_wf_ASIC1_vr_faults()
{
	//Clear VR_P0V85_ASIC1 fault bit
	plat_pldm_sensor_clear_vr_fault(ADDR_VR_P0V85_ASIC1, I2C_BUS8, 2);
	//Clear VR_P0V8_ASIC1 fault bit
	plat_pldm_sensor_clear_vr_fault(ADDR_VR_P0V8_ASIC1, I2C_BUS8, 2);
}

void clear_wf_ASIC2_vr_faults()
{
	//Clear VR_P0V85_ASIC2 fault bit
	plat_pldm_sensor_clear_vr_fault(ADDR_VR_P0V85_ASIC2, I2C_BUS3, 2);
	//Clear VR_P0V8_ASIC2 fault bit
	plat_pldm_sensor_clear_vr_fault(ADDR_VR_P0V8_ASIC2, I2C_BUS3, 2);
}

void switch_mux_to_bic(uint8_t value_to_write)
{
	uint8_t value = 0x0;

	k_mutex_lock(&switch_ioe_mux_mutex, K_SECONDS(SWITCH_IOE_MUX_TIMEOUT_SECONDS));
	if (get_ioe_value(ADDR_IOE2, TCA9555_OUTPUT_PORT_REG_0, &value) == 0) {
		value |= value_to_write; // Enable P0~P3 to switch mux to BIC.
		set_ioe_value(ADDR_IOE2, TCA9555_OUTPUT_PORT_REG_0, value);
	}
	k_mutex_unlock(&switch_ioe_mux_mutex);

	// Check if AC lost and ASIC1 VR fault bit not clear ,then do clear fault command
	if (is_ac_lost() && (!is_wf_ASIC1_vr_clear_fault_done) &&
	    value_to_write == IOE_SWITCH_CXL1_VR_TO_BIC) {
		is_wf_ASIC1_vr_clear_fault_done = true;
		k_msleep(1000);
		clear_wf_ASIC1_vr_faults();
	}

	// Check if AC lost and ASIC2 VR fault bit not clear ,then do clear fault command
	if (is_ac_lost() && (!is_wf_ASIC2_vr_clear_fault_done) &&
	    value_to_write == IOE_SWITCH_CXL2_VR_TO_BIC) {
		is_wf_ASIC2_vr_clear_fault_done = true;
		k_msleep(1000);
		clear_wf_ASIC2_vr_faults();
	}

	if (!is_wf_IOE_INT_reset_done) {
		// Reset IO Expander INT#
		reset_IOE_INT();
		is_wf_IOE_INT_reset_done = true;
	}

	return;
}

bool get_cxl_heartbeat(char *label)
{
	CHECK_NULL_ARG_WITH_RETURN(label, false);

	const struct device *heartbeat = NULL;
	struct sensor_value sensor_value;
	int ret = 0;

	heartbeat = device_get_binding(label);
	if (heartbeat == NULL) {
		LOG_ERR("[%s] %s device not found", __func__, label);
		return false;
	}

	ret = sensor_sample_fetch(heartbeat);
	if (ret < 0) {
		LOG_ERR("[%s] Failed to read %s due to sensor_sample_fetch failed, ret: %d",
			__func__, label, ret);
		return false;
	}

	ret = sensor_channel_get(heartbeat, SENSOR_CHAN_RPM, &sensor_value);
	if (ret < 0) {
		LOG_ERR("[%s] Failed to read %s due to sensor_channel_get failed, ret: %d",
			__func__, label, ret);
		return false;
	}

	if (sensor_value.val1 <= 0) {
		LOG_ERR("[%s] %s value <= 0, val: %d", __func__, label, sensor_value.val1);
		return false;
	}

	return true;
}

void cxl1_heartbeat_monitor_handler()
{
	if (!get_DC_status()) {
		LOG_INF("[%s] Stop monitor CXL1 HB due to DC power off", __func__);
		return;
	}

	bool is_cxl_perst = (gpio_get(RST_PCIE_MB_EXP_N) == GPIO_LOW) ? true : false;

	if (is_cxl_perst == true) {
		if (cxl1_hb_state != HB_STATE_UNKNOWN) {
			LOG_INF("[%s] Waiting for CXL1 PERST+", __func__);
			cxl1_hb_state = HB_STATE_UNKNOWN;
		}
	} else {
		struct pldm_addsel_data sel_msg = { 0 };
		bool is_heartbeat_blinking = get_cxl_heartbeat(CXL1_HEART_BEAT_LABEL);

		if (is_heartbeat_blinking == true) {
			// Check again to avoid timing issue
			if (!get_DC_status()) {
				LOG_INF("[%s] Stop monitor CXL1 HB due to DC power off", __func__);
				return;
			}
			is_cxl_perst = (gpio_get(RST_PCIE_MB_EXP_N) == GPIO_LOW) ? true : false;
			if (is_cxl_perst == true) {
				if (cxl1_hb_state != HB_STATE_UNKNOWN) {
					LOG_INF("[%s] Waiting for CXL1 PERST+", __func__);
					cxl1_hb_state = HB_STATE_UNKNOWN;
				}
			} else {
				if (cxl1_hb_state != HB_STATE_OK) {
					LOG_INF("[%s] Assert: CXL1 HB blinking", __func__);
					cxl1_hb_state = HB_STATE_OK;
					sel_msg.event_type = CXL1_HB;
					sel_msg.assert_type = EVENT_ASSERTED;
					if (send_event_log_to_bmc(sel_msg) != PLDM_SUCCESS) {
						LOG_ERR("[%s] Failed to send assert event log",
							__func__);
					}
				}
			}
		} else {
			// Check again to avoid timing issue
			if (!get_DC_status()) {
				LOG_INF("[%s] Stop monitor CXL1 HB due to DC power off", __func__);
				return;
			}
			is_cxl_perst = (gpio_get(RST_PCIE_MB_EXP_N) == GPIO_LOW) ? true : false;
			if (is_cxl_perst == true) {
				if (cxl1_hb_state != HB_STATE_UNKNOWN) {
					LOG_INF("[%s] Waiting for CXL1 PERST+", __func__);
					cxl1_hb_state = HB_STATE_UNKNOWN;
				}
			} else {
				if (cxl1_hb_state != HB_STATE_LOW) {
					LOG_INF("[%s] Deassert: CXL1 HB not blinking", __func__);
					cxl1_hb_state = HB_STATE_LOW;
					sel_msg.event_type = CXL1_HB;
					sel_msg.assert_type = EVENT_DEASSERTED;
					if (send_event_log_to_bmc(sel_msg) != PLDM_SUCCESS) {
						LOG_ERR("[%s] Failed to send deassert event log",
							__func__);
					}
				}
			}
		}
	}
	// Reschedule self
	k_work_schedule_for_queue(&plat_work_q, &cxl1_hb_monitor_work,
				  K_SECONDS(MONITOR_INTERVAL_SECONDS));
}

void cxl2_heartbeat_monitor_handler()
{
	if (!get_DC_status()) {
		LOG_INF("[%s] Stop monitor CXL2 HB due to DC power off", __func__);
		return;
	}

	bool is_cxl_perst = (gpio_get(RST_PCIE_MB_EXP_N) == GPIO_LOW) ? true : false;

	if (is_cxl_perst == true) {
		if (cxl2_hb_state != HB_STATE_UNKNOWN) {
			LOG_INF("[%s] Waiting for CXL2 PERST+", __func__);
			cxl2_hb_state = HB_STATE_UNKNOWN;
		}
	} else {
		struct pldm_addsel_data sel_msg = { 0 };
		bool is_heartbeat_blinking = get_cxl_heartbeat(CXL2_HEART_BEAT_LABEL);

		if (is_heartbeat_blinking == true) {
			// Check again to avoid timing issue
			if (!get_DC_status()) {
				LOG_INF("[%s] Stop monitor CXL2 HB due to DC power off", __func__);
				return;
			}
			is_cxl_perst = (gpio_get(RST_PCIE_MB_EXP_N) == GPIO_LOW) ? true : false;
			if (is_cxl_perst == true) {
				if (cxl2_hb_state != HB_STATE_UNKNOWN) {
					LOG_INF("[%s] Waiting for CXL2 PERST+", __func__);
					cxl2_hb_state = HB_STATE_UNKNOWN;
				}
			} else {
				if (cxl2_hb_state != HB_STATE_OK) {
					LOG_INF("[%s] Assert: CXL2 HB blinking", __func__);
					cxl2_hb_state = HB_STATE_OK;
					sel_msg.event_type = CXL2_HB;
					sel_msg.assert_type = EVENT_ASSERTED;
					if (send_event_log_to_bmc(sel_msg) != PLDM_SUCCESS) {
						LOG_ERR("[%s] Failed to send assert event log",
							__func__);
					}
				}
			}
		} else {
			// Check again to avoid timing issue
			if (!get_DC_status()) {
				LOG_INF("[%s] Stop monitor CXL2 HB due to DC power off", __func__);
				return;
			}
			is_cxl_perst = (gpio_get(RST_PCIE_MB_EXP_N) == GPIO_LOW) ? true : false;
			if (is_cxl_perst == true) {
				if (cxl2_hb_state != HB_STATE_UNKNOWN) {
					LOG_INF("[%s] Waiting for CXL2 PERST+", __func__);
					cxl2_hb_state = HB_STATE_UNKNOWN;
				}
			} else {
				if (cxl2_hb_state != HB_STATE_LOW) {
					LOG_INF("[%s] Deassert: CXL2 HB not blinking", __func__);
					cxl2_hb_state = HB_STATE_LOW;
					sel_msg.event_type = CXL2_HB;
					sel_msg.assert_type = EVENT_DEASSERTED;
					if (send_event_log_to_bmc(sel_msg) != PLDM_SUCCESS) {
						LOG_ERR("[%s] Failed to send deassert event log",
							__func__);
					}
				}
			}
		}
	}
	// Reschedule self
	k_work_schedule_for_queue(&plat_work_q, &cxl2_hb_monitor_work,
				  K_SECONDS(MONITOR_INTERVAL_SECONDS));
}

void cxl1_ready_handler()
{
	const struct device *heartbeat = NULL;
	struct sensor_value hb_val;
	int ret = 0;

	k_msleep(30000);
	LOG_INF("Start monitor CXL1 ready");

	heartbeat = device_get_binding(CXL1_HEART_BEAT_LABEL);
	if (heartbeat == NULL) {
		LOG_ERR("%s device not found", CXL1_HEART_BEAT_LABEL);
		return;
	}

	for (int times = 0; times < CXL_READY_RETRY_TIMES; times++) {
		ret = sensor_sample_fetch(heartbeat);
		if (ret < 0) {
			k_sleep(K_SECONDS(CXL_READY_INTERVAL_SECONDS));
			continue;
		}

		ret = sensor_channel_get(heartbeat, SENSOR_CHAN_RPM, &hb_val);
		if (ret < 0) {
			k_sleep(K_SECONDS(CXL_READY_INTERVAL_SECONDS));
			continue;
		}

		if (hb_val.val1 <= 0) {
			k_sleep(K_SECONDS(CXL_READY_INTERVAL_SECONDS));
			continue;
		}

		is_cxl_ready[CXL_ID_1] = true;
		LOG_INF("CXL1 is ready");
		/* Switch muxs to BIC*/
		switch_mux_to_bic(IOE_SWITCH_CXL1_VR_TO_BIC);
		k_work_schedule(&set_cxl1_vr_ready_work, K_SECONDS(VR_READY_DELAY_SEC));

		goto exit;
	}
	LOG_ERR("CXL1 is not ready, check %s timeout, ret: %d", CXL1_HEART_BEAT_LABEL, ret);
	switch_mux_to_bic(IOE_SWITCH_CXL1_VR_TO_BIC);
	set_cxl_vr_access(CXL_ID_1, true);

exit:
	// Start delayable heartbeat monitor
	if (!k_work_delayable_is_pending(&cxl1_hb_monitor_work)) {
		LOG_INF("Start to monitor CXL1 HB");
		k_work_schedule_for_queue(&plat_work_q, &cxl1_hb_monitor_work, K_NO_WAIT);
	} else {
		LOG_INF("CXL1 HB monitor already scheduled");
	}
	return;
}

void cxl2_ready_handler()
{
	const struct device *heartbeat = NULL;
	struct sensor_value hb_val;
	int ret = 0;

	k_msleep(30000);
	LOG_INF("Start monitor CXL2 ready");

	heartbeat = device_get_binding(CXL2_HEART_BEAT_LABEL);
	if (heartbeat == NULL) {
		LOG_ERR("%s device not found", CXL2_HEART_BEAT_LABEL);
		return;
	}

	for (int times = 0; times < CXL_READY_RETRY_TIMES; times++) {
		ret = sensor_sample_fetch(heartbeat);
		if (ret < 0) {
			k_sleep(K_SECONDS(CXL_READY_INTERVAL_SECONDS));
			continue;
		}

		ret = sensor_channel_get(heartbeat, SENSOR_CHAN_RPM, &hb_val);
		if (ret < 0) {
			k_sleep(K_SECONDS(CXL_READY_INTERVAL_SECONDS));
			continue;
		}

		if (hb_val.val1 <= 0) {
			k_sleep(K_SECONDS(CXL_READY_INTERVAL_SECONDS));
			continue;
		}

		is_cxl_ready[CXL_ID_2] = true;
		LOG_INF("CXL2 is ready");
		/* Switch muxs to BIC*/
		switch_mux_to_bic(IOE_SWITCH_CXL2_VR_TO_BIC);
		k_work_schedule(&set_cxl2_vr_ready_work, K_SECONDS(VR_READY_DELAY_SEC));

		goto exit;
	}
	LOG_ERR("CXL2 is not ready, check %s timeout, ret: %d", CXL2_HEART_BEAT_LABEL, ret);
	switch_mux_to_bic(IOE_SWITCH_CXL2_VR_TO_BIC);
	set_cxl_vr_access(CXL_ID_2, true);

exit:
	// Start delayable heartbeat monitor
	if (!k_work_delayable_is_pending(&cxl2_hb_monitor_work)) {
		LOG_INF("Start to monitor CXL2 HB");
		k_work_schedule_for_queue(&plat_work_q, &cxl2_hb_monitor_work, K_NO_WAIT);
	} else {
		LOG_INF("CXL2 HB monitor already scheduled");
	}
	return;
}

void set_cxl_vr_access(uint8_t cxl_id, bool value)
{
	// Set all vr access if cxl_id == MAX_CXL_ID
	if (cxl_id == MAX_CXL_ID) {
		is_cxl_vr_accessible[CXL_ID_1] = value;
		is_cxl_vr_accessible[CXL_ID_2] = value;
		return;
	}
	is_cxl_vr_accessible[cxl_id] = value;
	return;
}

void set_cxl1_vr_access_delayed_status()
{
	set_cxl_vr_access(CXL_ID_1, true);
}

void set_cxl2_vr_access_delayed_status()
{
	set_cxl_vr_access(CXL_ID_2, true);
}

bool cxl1_vr_access(uint8_t sensor_num)
{
	return is_cxl_vr_accessible[CXL_ID_1];
}

bool cxl2_vr_access(uint8_t sensor_num)
{
	return is_cxl_vr_accessible[CXL_ID_2];
}

void set_cxl_ready_status(uint8_t cxl_id, bool value)
{
	is_cxl_ready[cxl_id] = value;
	return;
}

bool get_cxl_ready_status(uint8_t cxl_id)
{
	return is_cxl_ready[cxl_id];
}

bool cxl1_ready_access(uint8_t sensor_num)
{
	return get_cxl_ready_status(CXL_ID_1);
}

bool cxl2_ready_access(uint8_t sensor_num)
{
	return get_cxl_ready_status(CXL_ID_2);
}
