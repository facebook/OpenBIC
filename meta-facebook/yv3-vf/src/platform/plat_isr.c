#include <zephyr.h>
#include <stdio.h>

#include "libipmi.h"
#include "power_status.h"

#include "plat_m2.h"
#include "plat_gpio.h"
#include "plat_i2c.h"
#include "plat_class.h"
#include "plat_power_seq.h"
#include "plat_util.h"
#include "plat_isr.h"

void dev_12v_fault_hander(void)
{
	const uint8_t all_12v_pwrgd = check_12v_dev_pwrgd();

	gpio_set(PWRGD_EXP_PWROK, all_12v_pwrgd);
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

	dev_12v_fault_hander(); // control PWRGD_EXP_PWROK & LED_PWRGD_P12V_E1S_ALL
	delay_function((val ? 110 : 1), pwrgd_p12v_aux_100ms_set, val, 0);
}

void power_en_int_handler(void)
{
	set_DC_status(FM_POWER_EN);
	set_DC_on_delayed_status();
	/* disable 12V switch first when the system power off */
	if (!gpio_get(FM_POWER_EN)) {
		uint8_t i;
		for (i = M2_IDX_E_A; i < M2_IDX_E_MAX; i++)
			m2_dev_power_switch(i, 0);
	} else {
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
		assert_func(DEASSERT_CHK_TYPE_E_INA231_ALERT_##DEV);                               \
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

	assert = (!status) ? IPMI_EVENT_TYPE_SENSOR_SPECIFIC : IPMI_OEM_EVENT_TYPE_DEASSART;
	add_sel(IPMI_OEM_SENSOR_TYPE_OEM, assert, SENSOR_NUM_SYS_STA,
		IPMI_EVENT_OFFSET_SYS_M2PRESENT, E1S_BOARD_TYPE, dev);
}

void prsnt_int_handler(uint32_t idx, uint32_t arg1)
{
	const uint8_t is_prsnt = m2_prsnt(idx);

	m2_presen_evt(idx, is_prsnt);

	uint8_t val = DEV_PWR_CTRL | DEV_PRSNT_SET | DEV_PCIE_RST | DEV_CHK_DISABLE |
		      (is_prsnt ? DEV_PWR_ON : 0);

	device_all_power_set((uint8_t)idx, val);
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
	void dev_12v_fault_hander_dev##idx(void)                                                   \
	{                                                                                          \
		dev_12v_fault_hander();                                                            \
	}

DEV_PRSNT_HANDLER(0);
DEV_PRSNT_HANDLER(1);
DEV_PRSNT_HANDLER(2);
DEV_PRSNT_HANDLER(3);

DEV_FAULT_HANDLER(0);
DEV_FAULT_HANDLER(1);
DEV_FAULT_HANDLER(2);
DEV_FAULT_HANDLER(3);

INA231_ALERT_HANDLER_M2(0);
INA231_ALERT_HANDLER_M2(1);
INA231_ALERT_HANDLER_M2(2);
INA231_ALERT_HANDLER_M2(3);
