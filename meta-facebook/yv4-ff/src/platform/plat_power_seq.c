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
#include <logging/log.h>
#include <drivers/sensor.h>
#include <drivers/pwm.h>

#include "power_status.h"
#include "vistara.h"
#include "plat_class.h"
#include "plat_gpio.h"
#include "plat_power_seq.h"

LOG_MODULE_REGISTER(plat_power_seq);

static bool is_cxl_ready = false;

K_WORK_DELAYABLE_DEFINE(set_dc_on_5s_work, set_DC_on_delayed_status);
K_WORK_DELAYABLE_DEFINE(cxl_ready_thread, cxl_ready_handler);
K_WORK_DELAYABLE_DEFINE(enable_power_on_rst_work, enable_power_on_rst);

void enable_power_on_rst()
{
	gpio_set(PWR_ON_RST_N_R, POWER_ON);
}

void set_mb_dc_status(uint8_t gpio_num)
{
	static bool is_mb_dc_on = false;

	is_mb_dc_on = (gpio_get(gpio_num) == POWER_ON) ? true : false;
	LOG_INF("MB DC (gpio num 0x%x) status is %s", gpio_num, is_mb_dc_on ? "ON" : "OFF");
}

void execute_power_on_sequence()
{
	int ret = 0;

	// If CXL is on, doesn't need to power on again
	if (get_DC_status()) {
		return;
	}
	/* Switch muxs to CXL before power on*/
	gpio_set(SEL_SMB_MUX_PMIC_R, GPIO_LOW);
	gpio_set(SEL_SMB_MUX_DIMM_R, GPIO_LOW);
	set_vr_monitor_status(false);

	ret = power_on_handler(CLK_POWER_ON_STAGE);
	if (ret == 0) {
		gpio_set(PG_CARD_OK, POWER_ON);
		gpio_set(LED_CXL_POWER, POWER_ON);
		set_DC_status(PG_CARD_OK);
		k_work_schedule(&set_dc_on_5s_work, K_SECONDS(DC_ON_DELAY5_SEC));
		k_work_schedule(&cxl_ready_thread, K_SECONDS(CXL_READY_INTERVAL_SECONDS));
		LOG_INF("CXL power on success");
	} else {
		LOG_ERR("CXL power on fail");
	}
}

void execute_power_off_sequence()
{
	int ret = 0;

	// If CXL is off, doesn't need to power off again
	if (!get_DC_status()) {
		return;
	}

	gpio_set(PG_CARD_OK, POWER_OFF);
	gpio_set(LED_CXL_POWER, GPIO_LOW);
	set_DC_status(PG_CARD_OK);

	if (k_work_cancel_delayable(&set_dc_on_5s_work) != 0) {
		LOG_ERR("Cancel set dc off delay work fail");
	}

	if (k_work_cancel_delayable(&cxl_ready_thread) != 0) {
		LOG_ERR("Failed to cancel cxl_ready_thread");
	}

	set_DC_on_delayed_status();

	is_cxl_ready = false;
	ret = power_off_handler(DIMM_POWER_OFF_STAGE_1);
	if (ret == 0) {
		LOG_INF("CXL power off success");
	} else {
		LOG_ERR("CXL power off fail");
	}
}

int power_on_handler(uint8_t power_stage)
{
	int ret = 0;
	int ctrl_stage = 0;

	for (ctrl_stage = power_stage; ctrl_stage < MAX_POWER_ON_STAGES; ctrl_stage++) {
		// Set power enable pin to enable power
		enable_powers(ctrl_stage);

		if (ctrl_stage != CLK_POWER_ON_STAGE) {
			k_msleep(CHK_PWR_DELAY_MSEC);
		}

		// Get power good pin to check power
		ret = check_powers_enabled(ctrl_stage);
		if (ret < 0) {
			break;
		}
	}

	return ret;
}

int power_off_handler(uint8_t power_stage)
{
	int ret = 0;
	int ctrl_stage = 0;

	for (ctrl_stage = power_stage; ctrl_stage < MAX_POWER_OFF_STAGES; ctrl_stage++) {
		// Set power enable pin to enable power
		disable_powers(ctrl_stage);

		k_msleep(CHK_PWR_DELAY_MSEC);

		// Get power good pin to check power
		ret = check_powers_disabled(ctrl_stage);
		if (ret < 0) {
			break;
		}
	}

	return ret;
}

void enable_powers(uint8_t pwr_stage)
{
	switch (pwr_stage) {
	case CLK_POWER_ON_STAGE:
		gpio_set(EN_CLK_100M_OSC, POWER_ON);
		k_msleep(SYS_CLK_STABLE_DELAY_MSEC);
		break;
	case ASIC_POWER_ON_STAGE_1:
		gpio_set(P0V75_ASIC_EN_BIC, POWER_ON);
		gpio_set(P0V8_BIC_ASIC_EN, POWER_ON);
		gpio_set(P0V85_BIC_ASIC_EN, POWER_ON);
		break;
	case ASIC_POWER_ON_STAGE_2:
		gpio_set(P1V2_BIC_ASIC_EN, POWER_ON);
		gpio_set(P1V8_BIC_ASIC_EN, POWER_ON);
		// set PWR_ON_RST_N after P1V8 enable 25ms
		k_work_schedule(&enable_power_on_rst_work, K_MSEC(PWR_ON_RST_DELAY_MSEC));
		gpio_set(SYS_RST_N_R, POWER_ON);
		break;
	case DIMM_POWER_ON_STAGE_1:
		gpio_set(PVPP_AB_BIC_DIMM_EN, POWER_ON);
		gpio_set(PVPP_CD_BIC_DIMM_EN, POWER_ON);
		break;
	case DIMM_POWER_ON_STAGE_2:
		gpio_set(PVDDQ_AB_EN_BIC, POWER_ON);
		gpio_set(PVDDQ_CD_EN_BIC, POWER_ON);
		break;
	case DIMM_POWER_ON_STAGE_3:
		gpio_set(PVTT_AB_BIC_DIMM_EN, POWER_ON);
		gpio_set(PVTT_CD_BIC_DIMM_EN, POWER_ON);
		break;
	default:
		LOG_ERR("Unknown stage 0x%x to enable power", pwr_stage);
		break;
	}
}

void disable_powers(uint8_t pwr_stage)
{
	float p1v8_adc = 0.0;

	switch (pwr_stage) {
	case DIMM_POWER_OFF_STAGE_1:
		gpio_set(PVTT_AB_BIC_DIMM_EN, POWER_OFF);
		gpio_set(PVTT_CD_BIC_DIMM_EN, POWER_OFF);
		break;
	case DIMM_POWER_OFF_STAGE_2:
		gpio_set(PVDDQ_AB_EN_BIC, POWER_OFF);
		gpio_set(PVDDQ_CD_EN_BIC, POWER_OFF);
		break;
	case DIMM_POWER_OFF_STAGE_3:
		gpio_set(PVPP_AB_BIC_DIMM_EN, POWER_OFF);
		gpio_set(PVPP_CD_BIC_DIMM_EN, POWER_OFF);
		break;
	case ASIC_POWER_OFF_STAGE_1:
		gpio_set(SYS_RST_N_R, POWER_OFF);
		gpio_set(PWR_ON_RST_N_R, POWER_OFF);
		gpio_set(P0V8_BIC_ASIC_EN, POWER_OFF);
		gpio_set(P1V2_BIC_ASIC_EN, POWER_OFF);
		gpio_set(P1V8_BIC_ASIC_EN, POWER_OFF);
		break;
	case ASIC_POWER_OFF_STAGE_2: {
		// Check ASIC_1V8 discharge completed before power off P0V85_ASIC
		bool success = get_adc_voltage(CHANNEL_5, &p1v8_adc);
		if ((!success) || (p1v8_adc != 0)) {
			k_msleep(P1V8_POWER_OFF_DELAY_MSEC);
		}

		gpio_set(P0V85_BIC_ASIC_EN, POWER_OFF);
		break;
	}
	case ASIC_POWER_OFF_STAGE_3:
		gpio_set(P0V75_ASIC_EN_BIC, POWER_OFF);
		break;
	case CLK_POWER_OFF_STAGE:
		gpio_set(EN_CLK_100M_OSC, POWER_OFF);
		break;
	default:
		LOG_ERR("Unknown stage 0x%x to disable power", pwr_stage);
		break;
	}
}

int check_powers_enabled(uint8_t pwr_stage)
{
	switch (pwr_stage) {
	case CLK_POWER_ON_STAGE:
		// Doesn't need to check
		break;
	case ASIC_POWER_ON_STAGE_1:
		if (!is_power_controlled(PWRGD_P0V75_ASIC_BIC, POWER_ON, "P0V75_ASIC")) {
			return -1;
		}

		if (!is_power_controlled(PWRGD_P0V8_ASIC, POWER_ON, "P0V8_ASIC")) {
			return -1;
		}

		if (!is_power_controlled(PWRGD_P0V85_ASIC, POWER_ON, "P0V85_ASIC")) {
			return -1;
		}
		break;
	case ASIC_POWER_ON_STAGE_2:
		if (!is_power_controlled(PWRGD_P1V2_ASIC, POWER_ON, "P1V2_ASIC")) {
			return -1;
		}

		if (!is_power_controlled(PWRGD_P1V8_ASIC, POWER_ON, "P1V8_ASIC")) {
			return -1;
		}
		break;
	case DIMM_POWER_ON_STAGE_1:
		if (!is_power_controlled(PWRGD_PVPP_AB, POWER_ON, "PVPP_AB")) {
			return -1;
		}

		if (!is_power_controlled(PWRGD_PVPP_CD, POWER_ON, "PVPP_CD")) {
			return -1;
		}
		break;
	case DIMM_POWER_ON_STAGE_2:
		if (!is_power_controlled(PWRGD_PVDDQ_AB_BIC, POWER_ON, "PVDDQ_AB")) {
			return -1;
		}

		if (!is_power_controlled(PWRGD_PVDDQ_CD_BIC, POWER_ON, "PVDDQ_CD")) {
			return -1;
		}
		break;
	case DIMM_POWER_ON_STAGE_3:
		if (!is_power_controlled(PWRGD_PVTT_AB, POWER_ON, "PVTT_AB")) {
			return -1;
		}

		if (!is_power_controlled(PWRGD_PVTT_CD, POWER_ON, "PVTT_CD")) {
			return -1;
		}
		break;
	default:
		LOG_ERR("Unknown stage 0x%x to check power on", pwr_stage);
		return -1;
	}

	return 0;
}

int check_powers_disabled(uint8_t pwr_stage)
{
	switch (pwr_stage) {
	case DIMM_POWER_OFF_STAGE_1:
		if (!is_power_controlled(PWRGD_PVTT_AB, POWER_OFF, "PVTT_AB")) {
			return -1;
		}

		if (!is_power_controlled(PWRGD_PVTT_CD, POWER_OFF, "PVTT_CD")) {
			return -1;
		}
		break;
	case DIMM_POWER_OFF_STAGE_2:
		if (!is_power_controlled(PWRGD_PVDDQ_AB_BIC, POWER_OFF, "PVDDQ_AB")) {
			return -1;
		}

		if (!is_power_controlled(PWRGD_PVDDQ_CD_BIC, POWER_OFF, "PVDDQ_CD")) {
			return -1;
		}
		break;
	case DIMM_POWER_OFF_STAGE_3:
		if (!is_power_controlled(PWRGD_PVPP_AB, POWER_OFF, "PVPP_AB")) {
			return -1;
		}

		if (!is_power_controlled(PWRGD_PVPP_CD, POWER_OFF, "PVPP_CD")) {
			return -1;
		}
		break;
	case ASIC_POWER_OFF_STAGE_1:
		if (!is_power_controlled(PWRGD_P0V8_ASIC, POWER_OFF, "P0V8_ASIC")) {
			return -1;
		}

		if (!is_power_controlled(PWRGD_P1V2_ASIC, POWER_OFF, "P1V2_ASIC")) {
			return -1;
		}

		if (!is_power_controlled(PWRGD_P1V8_ASIC, POWER_OFF, "P1V8_ASIC")) {
			return -1;
		}
		break;
	case ASIC_POWER_OFF_STAGE_2:
		if (!is_power_controlled(PWRGD_P0V85_ASIC, POWER_OFF, "P0V85_ASIC")) {
			return -1;
		}
		break;
	case ASIC_POWER_OFF_STAGE_3:
		if (!is_power_controlled(PWRGD_P0V75_ASIC_BIC, POWER_OFF, "P0V75_ASIC")) {
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

bool is_power_controlled(uint8_t power_pin, uint8_t check_power_status, char *power_name)
{
	if (gpio_get(power_pin) == check_power_status) {
		return true;
	} else {
		// TODO: Add event to BMC
		LOG_ERR("Failed to power %s %s (0x%x)", check_power_status ? "on" : "off",
			power_name, power_pin);
		return false;
	}
}

void cxl_ready_handler()
{
	const struct device *heartbeat = NULL;
	int heartbeat_status = 0;

	heartbeat = device_get_binding(CXL_HEART_BEAT_LABEL);
	if (heartbeat == NULL) {
		LOG_ERR("%s device not found", CXL_HEART_BEAT_LABEL);
		return;
	}

	for (int times = 0; times < CXL_READY_RETRY_TIMES; times++) {
		heartbeat_status = sensor_sample_fetch(heartbeat);
		if (heartbeat_status < 0) {
			k_sleep(K_SECONDS(CXL_READY_INTERVAL_SECONDS));
			continue;
		}

		is_cxl_ready = true;
		LOG_INF("CXL is ready");

		/* Switch muxs to BIC*/
		gpio_set(SEL_SMB_MUX_PMIC_R, GPIO_HIGH);
		set_vr_monitor_status(true);
		return;
	}
	LOG_ERR("Failed to read %s due to sensor_sample_fetch failed, ret: %d",
		CXL_HEART_BEAT_LABEL, heartbeat_status);
	return;
}

bool get_cxl_ready_status()
{
	return is_cxl_ready;
}

bool cxl_ready_access(uint8_t sensor_num)
{
	return get_cxl_ready_status();
}
