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

#include <zephyr.h>
#include <stdio.h>
#include <logging/log.h>

#include "libipmi.h"
#include "power_status.h"
#include "sensor.h"

#include "plat_m2.h"
#include "plat_gpio.h"
#include "plat_i2c.h"
#include "plat_class.h"
#include "plat_power_seq.h"
#include "plat_util.h"
#include "plat_isr.h"
#include "plat_hwmon.h"

extern uint8_t ina230_init(sensor_cfg *cfg);

LOG_MODULE_REGISTER(plat_isr);

void dev_12v_fault_handler(void)
{
	const uint8_t all_12v_pwrgd = check_12v_dev_pwrgd();

	gpio_set(LED_PWRGD_P12V_E1S_ALL, all_12v_pwrgd);
}

void pwrgd_p12v_aux_int_handler(void)
{
	const uint8_t val = get_e1s_pwrgd();

	if (val)
		gpio_set(FM_CLKBUF_EN, val);

	uint8_t i;
	for (i = M2_IDX_E_A; i < M2_IDX_E_MAX; i++) {
		if (m2_prsnt(i))
			m2_dev_power_switch(i, val);
	}

	set_exp_pwrgd_pin(); // control PWRGD_EXP_PWROK
	dev_12v_fault_handler(); // control LED_PWRGD_P12V_E1S_ALL
	delay_function((val ? 110 : 1), pwrgd_p12v_aux_100ms_set, val, 0);
}

void power_en_int_handler(void)
{
	set_exp_pwrgd_pin(); // control PWRGD_EXP_PWROK

	/* disable 12V switch first when the system power off */
	if (!gpio_get(FM_POWER_EN)) {
		plat_set_dc_status(FM_POWER_EN, 0);
		uint8_t i;
		for (i = M2_IDX_E_A; i < M2_IDX_E_MAX; i++)
			m2_dev_power_switch(i, 0);
	} else {
		delay_function(100, plat_set_dc_status, FM_POWER_EN, 0);
		if (get_e1s_hsc_config() != CONFIG_HSC_BYPASS)
			gpio_set(FM_P12V_EDGE_EN, gpio_get(FM_POWER_EN));
		/* Bypass config doesn't has HSC, so direct call p12v aux handler here*/
		else
			pwrgd_p12v_aux_int_handler();
	}
}

#define INA231_ALERT_HANDLER_M2(DEV)                                                               \
	void ina231_alert_handler_m2_dev##DEV(void)                                                \
	{                                                                                          \
		if (m2_prsnt(DEV) && get_fm_p12v_sw_en(DEV))                                       \
			assert_func(DEASSERT_CHK_TYPE_E_INA231_ALERT_##DEV);                       \
	}

void dev_rst(void)
{
	const uint8_t val = pwrgd_p12v_aux_100ms_get() & gpio_get(RST_MB_N);

	uint8_t i;
	for (i = M2_IDX_E_A; i < M2_IDX_E_MAX; i++)
		rst_edsff(i, val);
}

uint8_t get_p12v_flt_status(uint8_t idx)
{
	const uint8_t pin = (idx == M2_IDX_E_A) ? IRQ_P12V_E1S_0_FLT_N :
			    (idx == M2_IDX_E_B) ? IRQ_P12V_E1S_1_FLT_N :
			    (idx == M2_IDX_E_C) ? IRQ_P12V_E1S_2_FLT_N :
			    (idx == M2_IDX_E_D) ? IRQ_P12V_E1S_3_FLT_N :
						  0xFF;

	if (pin == 0xFF)
		return 0;

	return !gpio_get(pin);
}

uint8_t check_12v_dev_pwrgd(void)
{
	uint8_t i = 0;
	if (get_e1s_pwrgd()) {
		for (i = 0; i < M2_IDX_E_MAX; i++) {
			if (!m2_prsnt(i))
				continue;
			if (get_p12v_flt_status(i))
				break;
		}
	}
	return (i == M2_IDX_E_MAX);
}

void aux_pwr_en_int_handler(void)
{
	const uint8_t val = gpio_get(FM_AUX_PWR_EN);

	uint8_t i;
	for (i = M2_IDX_E_A; i < M2_IDX_E_MAX; i++) {
		if (m2_prsnt(i))
			fm_p3v3_sw_en(i, val);
	}
}

void rst_mb_n_int_handler(void)
{
	dev_rst();
}

static void m2_presen_evt(uint32_t dev, uint32_t status)
{
	uint8_t assert;

	assert = (!status) ? IPMI_EVENT_TYPE_SENSOR_SPECIFIC : IPMI_OEM_EVENT_TYPE_DEASSERT;
	add_sel(IPMI_OEM_SENSOR_TYPE_OEM, assert, SENSOR_NUM_SYS_STA,
		IPMI_EVENT_OFFSET_SYS_M2PRESENT, E1S_BOARD_TYPE, dev);
}

void prsnt_int_handler(uint32_t idx, uint32_t arg1)
{
	const uint8_t is_prsnt = m2_prsnt(idx);
	uint8_t i, retry = 3;

	for (i = 0; i < retry; i++) {
		if (!is_prsnt)
			break;

		const uint8_t sen_num = m2_idx2sensornum(idx);
		if (sen_num == 0xFF)
			break;

		sensor_cfg *cfg = &sensor_config[sensor_config_index_map[sen_num]];
		if (!cfg)
			break;

		if (ina230_init(cfg) == SENSOR_INIT_SUCCESS)
			break;
		k_msleep(10); // retry delay time 10ms
	}

	m2_presen_evt(idx, is_prsnt);

	uint8_t val = DEV_PWR_CTRL | DEV_PRSNT_SET | DEV_PCIE_RST | DEV_CHK_DISABLE |
		      (is_prsnt ? DEV_PWR_ON : 0);

	device_all_power_set((uint8_t)idx, val);
}

void irq_fault_sel(uint8_t idx, uint8_t type, uint8_t is_check)
{
	uint8_t pin = 0xFF;
	uint8_t event_data1 = 0xFF;
	uint8_t en_pin = 0xFF;

	switch (type) {
	case P12V_E1S:
		if (!m2_prsnt(idx))
			return;
		event_data1 = IPMI_EVENT_OFFSET_SYS_IRQ_P12V_E1S_FLT;
		pin = (idx == 0) ? IRQ_P12V_E1S_0_FLT_N :
		      (idx == 1) ? IRQ_P12V_E1S_1_FLT_N :
		      (idx == 2) ? IRQ_P12V_E1S_2_FLT_N :
		      (idx == 3) ? IRQ_P12V_E1S_3_FLT_N :
				   0xFF;
		en_pin = (idx == 0) ? FM_P12V_E1S_0_EN :
			 (idx == 1) ? FM_P12V_E1S_1_EN :
			 (idx == 2) ? FM_P12V_E1S_2_EN :
			 (idx == 3) ? FM_P12V_E1S_3_EN :
				      0xFF;
		break;
	case P3V3_E1S:
		if (!m2_prsnt(idx))
			return;
		event_data1 = IPMI_EVENT_OFFSET_SYS_IRQ_P3V3_E1S_FLT;
		pin = (idx == 0) ? IRQ_P3V3_E1S_0_FLT_N :
		      (idx == 1) ? IRQ_P3V3_E1S_1_FLT_N :
		      (idx == 2) ? IRQ_P3V3_E1S_2_FLT_N :
		      (idx == 3) ? IRQ_P3V3_E1S_3_FLT_N :
				   0xFF;
		en_pin = (idx == 0) ? FM_P3V3_E1S_0_SW_EN :
			 (idx == 1) ? FM_P3V3_E1S_1_SW_EN :
			 (idx == 2) ? FM_P3V3_E1S_2_SW_EN :
			 (idx == 3) ? FM_P3V3_E1S_3_SW_EN :
				      0xFF;
		break;
	case P12V_EDGE:
		event_data1 = IPMI_EVENT_OFFSET_SYS_IRQ_P12V_EDGE_FLT;
		pin = IRQ_P12V_EDGE_FLT_N;
		en_pin = FM_P12V_EDGE_EN;
		break;
	default:
		LOG_DBG("invaild irq fault type: %d!", type);
		return;
	}

	if (is_check) {
		// run in init, if irq_pin still low
		if (!gpio_get(pin)) {
			add_sel(IPMI_OEM_SENSOR_TYPE_OEM, IPMI_EVENT_TYPE_SENSOR_SPECIFIC,
				SENSOR_NUM_SYS_STA, event_data1, E1S_BOARD_TYPE, idx);
		}
		return;
	}

	// check enable pin
	if (!gpio_get(en_pin)) {
		LOG_DBG("enable pin %d does not set high!", en_pin);
		return;
	}

	uint8_t event_type =
		gpio_get(pin) ? IPMI_OEM_EVENT_TYPE_DEASSERT : IPMI_EVENT_TYPE_SENSOR_SPECIFIC;

	add_sel(IPMI_OEM_SENSOR_TYPE_OEM, event_type, SENSOR_NUM_SYS_STA, event_data1,
		E1S_BOARD_TYPE, idx);
}

void check_irq_fault(void)
{
	uint8_t i;
	for (i = 0; i < M2_IDX_E_MAX; i++)
		irq_fault_sel(i, P12V_E1S, 1);
	for (i = 0; i < M2_IDX_E_MAX; i++)
		irq_fault_sel(i, P3V3_E1S, 1);
	irq_fault_sel(0, P12V_EDGE, 1);
}

bool plat_gpio_immediate_int_cb(uint8_t gpio_num)
{
	bool ret = false;

	switch (gpio_num) {
	case IRQ_P12V_E1S_0_FLT_N:
	case IRQ_P12V_E1S_1_FLT_N:
	case IRQ_P12V_E1S_2_FLT_N:
	case IRQ_P12V_E1S_3_FLT_N:
		ret = true;
		break;
	default:
		break;
	}

	return ret;
}

#define DEV_PRSNT_HANDLER(idx)                                                                     \
	void prsnt_int_handler_dev##idx(void)                                                      \
	{                                                                                          \
		static int64_t pre_time;                                                           \
		int64_t current_time = k_uptime_get();                                             \
		if ((current_time - pre_time) < 10) {                                              \
			return;                                                                    \
		}                                                                                  \
		pre_time = current_time;                                                           \
		delay_function(10, prsnt_int_handler, idx, 0);                                     \
	}

#define DEV_FAULT_HANDLER(idx)                                                                     \
	void dev_12v_fault_handler_dev##idx(void)                                                  \
	{                                                                                          \
		static int64_t pre_time[M2_IDX_E_MAX];                                             \
		int64_t current_time = k_uptime_get();                                             \
		if ((current_time - pre_time[idx]) < 10) {                                         \
			return;                                                                    \
		}                                                                                  \
		pre_time[idx] = current_time;                                                      \
		if (gpio_get(IRQ_P12V_E1S_##idx##_FLT_N) == 0) {                                   \
			rst_edsff(idx, 0);                                                         \
			clkbuf_oe_en(idx, 0);                                                      \
		}                                                                                  \
		dev_12v_fault_handler();                                                           \
		irq_fault_sel(idx, P12V_E1S, 0);                                                   \
	}

#define DEV_3V3_FAULT_HANDLER(idx)                                                                 \
	void dev_3v3_fault_handler_dev##idx(void)                                                  \
	{                                                                                          \
		static int64_t pre_time[M2_IDX_E_MAX];                                             \
		int64_t current_time = k_uptime_get();                                             \
		if ((current_time - pre_time[idx]) < 10) {                                         \
			return;                                                                    \
		}                                                                                  \
		pre_time[idx] = current_time;                                                      \
		irq_fault_sel(idx, P3V3_E1S, 0);                                                   \
	}

void p12v_edge_fault_sel(void)
{
	static int64_t pre_time;
	int64_t current_time = k_uptime_get();
	if ((current_time - pre_time) < 10) {
		return;
	}
	pre_time = current_time;
	irq_fault_sel(0, P12V_EDGE, 0);
}

DEV_PRSNT_HANDLER(0);
DEV_PRSNT_HANDLER(1);
DEV_PRSNT_HANDLER(2);
DEV_PRSNT_HANDLER(3);

DEV_FAULT_HANDLER(0);
DEV_FAULT_HANDLER(1);
DEV_FAULT_HANDLER(2);
DEV_FAULT_HANDLER(3);

DEV_3V3_FAULT_HANDLER(0);
DEV_3V3_FAULT_HANDLER(1);
DEV_3V3_FAULT_HANDLER(2);
DEV_3V3_FAULT_HANDLER(3);

INA231_ALERT_HANDLER_M2(0);
INA231_ALERT_HANDLER_M2(1);
INA231_ALERT_HANDLER_M2(2);
INA231_ALERT_HANDLER_M2(3);
