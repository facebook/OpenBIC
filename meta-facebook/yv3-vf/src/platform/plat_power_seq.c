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
#include <stdbool.h>
#include "ipmi.h"
#include "ipmb.h"
#include "hal_gpio.h"
#include "power_status.h"
#include "plat_m2.h"
#include "plat_class.h"
#include "plat_util.h"
#include "plat_isr.h"
#include "plat_i2c.h"
#include "plat_hwmon.h"

#include "plat_power_seq.h"

static uint8_t is_pwrgd_p12v_aux_100ms;
static uint8_t sensor_pwrgd_1s[M2_IDX_E_MAX];
static uint8_t nvme_dev_ready_15s[M2_IDX_E_MAX];

void pwr_related_pin_init(void)
{
	uint8_t i;

	/* init FM_P3V3_E1S_X_SW_EN */
	for (i = M2_IDX_E_A; i < M2_IDX_E_MAX; i++)
		fm_p3v3_sw_en(i, m2_prsnt(i) & gpio_get(FM_AUX_PWR_EN));

	/* init FM_P12V_EDGE_EN */
	gpio_set(FM_P12V_EDGE_EN, gpio_get(FM_POWER_EN));

	/* Set the default value for FM_P12V_E1S_X_EN, FM_CLKBUF_EN, CLKBUF_E1S_X_OE_N and RST_BIC_E1S_X_N */
	pwrgd_p12v_aux_int_handler();
}

uint8_t get_dev_pwrgd(uint8_t idx)
{
	return sensor_pwrgd_1s[idx];
}

static void set_sensor_pwrgd_1s(uint8_t idx)
{
	sensor_pwrgd_1s[idx] = 1;
}

uint8_t get_nvme_dev_ready_15s(uint8_t idx)
{
	return nvme_dev_ready_15s[idx];
}

static void set_nvme_dev_ready_15s(uint8_t idx)
{
	nvme_dev_ready_15s[idx] = 1;
}

uint8_t fm_p3v3_sw_en(uint8_t idx, uint8_t val)
{
	const uint8_t pin = (idx == M2_IDX_E_A) ? FM_P3V3_E1S_0_SW_EN :
			    (idx == M2_IDX_E_B) ? FM_P3V3_E1S_1_SW_EN :
			    (idx == M2_IDX_E_C) ? FM_P3V3_E1S_2_SW_EN :
			    (idx == M2_IDX_E_D) ? FM_P3V3_E1S_3_SW_EN :
						  0xFF;

	if (pin == 0xFF)
		return 1;

	gpio_set(pin, val);

	return 0;
}

uint8_t fm_p12v_sw_en(uint8_t idx, uint8_t val)
{
	const uint8_t pin = (idx == M2_IDX_E_A) ? FM_P12V_E1S_0_EN :
			    (idx == M2_IDX_E_B) ? FM_P12V_E1S_1_EN :
			    (idx == M2_IDX_E_C) ? FM_P12V_E1S_2_EN :
			    (idx == M2_IDX_E_D) ? FM_P12V_E1S_3_EN :
						  0xFF;

	if (pin == 0xFF)
		return 1;

	gpio_set(pin, val);
	dev_pwrgd_handler(idx);

	return 0;
}

uint8_t get_fm_p12v_sw_en(uint8_t idx)
{
	const uint8_t pin = (idx == M2_IDX_E_A) ? FM_P12V_E1S_0_EN :
			    (idx == M2_IDX_E_B) ? FM_P12V_E1S_1_EN :
			    (idx == M2_IDX_E_C) ? FM_P12V_E1S_2_EN :
			    (idx == M2_IDX_E_D) ? FM_P12V_E1S_3_EN :
						  0xFF;

	if (pin == 0xFF)
		return 0;

	return gpio_get(pin);
}

uint8_t clkbuf_oe_en(uint8_t idx, uint8_t val)
{
	const uint8_t pin = (idx == M2_IDX_E_A) ? CLKBUF_E1S_0_OE_N :
			    (idx == M2_IDX_E_B) ? CLKBUF_E1S_1_OE_N :
			    (idx == M2_IDX_E_C) ? CLKBUF_E1S_2_OE_N :
			    (idx == M2_IDX_E_D) ? CLKBUF_E1S_3_OE_N :
						  0xFF;

	if (pin == 0xFF)
		return 1;

	gpio_set(pin, !val);
	return 0;
}

uint8_t get_fm_pwrdis_status(uint8_t idx)
{
	const uint8_t pin = (idx == M2_IDX_E_A) ? FM_PWRDIS_E1S_0 :
			    (idx == M2_IDX_E_B) ? FM_PWRDIS_E1S_1 :
			    (idx == M2_IDX_E_C) ? FM_PWRDIS_E1S_2 :
			    (idx == M2_IDX_E_D) ? FM_PWRDIS_E1S_3 :
						  0xFF;

	if (pin == 0xFF)
		return 0;

	return gpio_get(pin);
}

uint8_t fm_pwrdis_en(uint8_t idx, uint8_t val)
{
	const uint8_t pin = (idx == M2_IDX_E_A) ? FM_PWRDIS_E1S_0 :
			    (idx == M2_IDX_E_B) ? FM_PWRDIS_E1S_1 :
			    (idx == M2_IDX_E_C) ? FM_PWRDIS_E1S_2 :
			    (idx == M2_IDX_E_D) ? FM_PWRDIS_E1S_3 :
						  0xFF;

	if (pin == 0xFF)
		return 1;

	gpio_set(pin, val);
	return 0;
}

void check_dc_off_process(void)
{
	uint8_t i;

	for (i = 0; i < M2_IDX_E_MAX; i++) {
		if (get_fm_p12v_sw_en(i))
			break;
	}

	if (i == M2_IDX_E_MAX) {
		dev_12v_fault_handler(); // control PWRGD_EXP_PWROK & LED_PWRGD_P12V_E1S_ALL
		gpio_set(FM_P12V_EDGE_EN, 0);
		gpio_set(FM_CLKBUF_EN, 0);
	}
}

/*
 * +--------+---------------+------------------------------------------+
 * |        | AC            | ON                                       |
 * |        +---------------+----------------+-------------------------+
 * |        | DC            | OFF            | ON                      |
 * |        +---------------+-------+--------+-------+--------+--------+
 * |        | PRSNT         | ON    | OFF    | ON    | ON     | OFF    |
 * |        +---------------+-------+--------+-------+--------+--------+
 * |        | DRIVE PWR     | OFF            | ON    | OFF    | OFF    |
 * +--------+---------------+-------+--------+-------+--------+--------+
 * | FM_P12V_E1S_X_EN       | L     | L      | H     | L      | L      |
 * +------------------------+-------+--------+-------+--------+--------+
 * | FM_P3V3_E1S_X_SW_EN    | H     | L      | H     | H      | L      |
 * +------------------------+-------+--------+-------+--------+--------+
 * | CLKBUF_E1S_X_OE_N      | H     | H      | L     | L      | H      |
 * +------------------------+-------+--------+-------+--------+--------+
 */

uint8_t m2_dev_power_switch_with_pwrdis_chk(uint8_t idx, uint8_t enable, uint8_t chk_pwrdis,
					    uint8_t force_ctl_3v3)
{
	static uint8_t is_force_disable_3v3[M2_IDX_E_MAX];

	const uint8_t dc_en = gpio_get(FM_POWER_EN);
	const uint8_t hsc_pwrgd = get_e1s_pwrgd();
	const uint8_t prsnt = m2_prsnt(idx);

	const uint8_t en_12v = dc_en && hsc_pwrgd && prsnt && enable;
	const uint8_t en_3v3 = force_ctl_3v3 ? enable : prsnt;
	const uint8_t en_clk = enable && prsnt;

	/* if the PWRDIS is enable by user, don't control the drive power by 12V/3V3 SW. */
	if (chk_pwrdis) {
		if (get_fm_pwrdis_status(idx))
			return 1;
	}

	/* for the drive power off, the 12v should be disable before 3v3 */
	if (!enable) {
		clkbuf_oe_en(idx, en_clk);
		delay_function(3, fm_p12v_sw_en, idx, en_12v);
	}

	if (force_ctl_3v3) {
		is_force_disable_3v3[idx] = !enable;
		if (!enable) {
			fm_p3v3_sw_en(idx, en_3v3);
		}
	}

	// check whether force disable
	if (!is_force_disable_3v3[idx])
		fm_p3v3_sw_en(idx, en_3v3);

	if (enable) {
		fm_p12v_sw_en(idx, en_12v);

		//delay 40 ms to enable clkbuf after enable 12V
		delay_function(50, clkbuf_oe_en, idx, en_clk);
	}

	if (!dc_en) {
		delay_function(20, check_dc_off_process, 0, 0);
	}

	return 0;
}

uint8_t m2_dev_power_switch(uint8_t idx, uint8_t enable)
{
	return m2_dev_power_switch_with_pwrdis_chk(idx, enable, 1, 0);
}

uint8_t device_all_power_set(uint8_t idx, uint8_t set_val)
{
	uint8_t is_on = ((set_val & DEV_PWR_ON) ? 1 : 0);
	uint8_t chk_pwrdis = ((set_val & DEV_CHK_DISABLE) ? 1 : 0);
	uint8_t force_ctl_3v3 = ((set_val & DEV_FORCE_3V3) ? 1 : 0);

	if (is_on) {
		if (set_val & DEV_PWR_CTRL)
			m2_dev_power_switch_with_pwrdis_chk(idx, is_on, chk_pwrdis, force_ctl_3v3);
		if (set_val & DEV_PWRDIS_EN) {
			fm_pwrdis_en(idx, !is_on);
			delay_function(40, clkbuf_oe_en, idx, is_on);
			delay_function(120, rst_edsff, idx, is_on);
		}
		if (set_val & DEV_PCIE_RST)
			delay_function(120, rst_edsff, idx, is_on);
		if (set_val & DEV_PRSNT_SET)
			delay_function(140, mb_cpld_dev_prsnt_set, idx, is_on);
		return 0;
	}

	if (set_val & DEV_PRSNT_SET)
		mb_cpld_dev_prsnt_set(idx, is_on);

	if (set_val & DEV_PCIE_RST)
		rst_edsff(idx, is_on);

	if (set_val & DEV_PWRDIS_EN) {
		fm_pwrdis_en(idx, !is_on);
		delay_function(4, rst_edsff, idx, is_on);
		delay_function(8, clkbuf_oe_en, idx, is_on);
	}

	if (set_val & DEV_PWR_CTRL) {
		if (force_ctl_3v3)
			m2_dev_power_switch_with_pwrdis_chk(idx, is_on, chk_pwrdis, force_ctl_3v3);
		else
			delay_function(3, m2_dev_power_switch, idx, is_on);
	}

	return 0;
}

void dev_pwrgd_handler(uint8_t idx)
{
	const uint8_t pin = (idx == M2_IDX_E_A) ? FM_P12V_E1S_0_EN :
			    (idx == M2_IDX_E_B) ? FM_P12V_E1S_1_EN :
			    (idx == M2_IDX_E_C) ? FM_P12V_E1S_2_EN :
			    (idx == M2_IDX_E_D) ? FM_P12V_E1S_3_EN :
						  0xFF;
	if (pin == 0xFF)
		return;

	if (get_fm_p12v_sw_en(idx)) {
		delay_function(1000, set_sensor_pwrgd_1s, idx, 0);
		delay_function(15000, set_nvme_dev_ready_15s, idx, 0);
	} else {
		sensor_pwrgd_1s[idx] = 0;
		nvme_dev_ready_15s[idx] = 0;
	}
}

void pwrgd_p12v_aux_100ms_set(uint32_t val, uint32_t unused1)
{
	is_pwrgd_p12v_aux_100ms = val;
	dev_rst();
}

uint8_t pwrgd_p12v_aux_100ms_get(void)
{
	return is_pwrgd_p12v_aux_100ms;
}

void plat_set_dc_status(uint32_t dc_pin, uint32_t unused) // unused para for delay function
{
	set_DC_status(dc_pin);
	set_DC_on_delayed_status();
}

#define DEV_PWRGD_HANDLER(idx)                                                                     \
	void dev_pwrgd_handler_dev##idx(void)                                                      \
	{                                                                                          \
		dev_pwrgd_handler(idx);                                                            \
	}

DEV_PWRGD_HANDLER(0);
DEV_PWRGD_HANDLER(1);
DEV_PWRGD_HANDLER(2);
DEV_PWRGD_HANDLER(3);
