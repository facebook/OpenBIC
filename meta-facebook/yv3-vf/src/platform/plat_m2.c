#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "plat_gpio.h"
#include "plat_m2.h"
#include "plat_power_seq.h"
#include "plat_i2c.h"

uint8_t m2_bus2idx(uint8_t bus)
{
	return (bus == I2C_BUS_M2A) ? M2_IDX_E_A :
	       (bus == I2C_BUS_M2B) ? M2_IDX_E_B :
	       (bus == I2C_BUS_M2C) ? M2_IDX_E_C :
	       (bus == I2C_BUS_M2D) ? M2_IDX_E_D :
					    M2_IDX_E_MAX;
}

uint8_t m2_bus2rst(uint8_t bus)
{
	return (bus == I2C_BUS_M2A) ? RST_SMB_E1S_0_N :
	       (bus == I2C_BUS_M2B) ? RST_SMB_E1S_1_N :
	       (bus == I2C_BUS_M2C) ? RST_SMB_E1S_2_N :
	       (bus == I2C_BUS_M2D) ? RST_SMB_E1S_3_N :
					    0xFF;
}

uint8_t m2_idx2bus(uint8_t idx)
{
	return (idx == M2_IDX_E_A) ? I2C_BUS_M2A :
	       (idx == M2_IDX_E_B) ? I2C_BUS_M2B :
	       (idx == M2_IDX_E_C) ? I2C_BUS_M2C :
	       (idx == M2_IDX_E_D) ? I2C_BUS_M2D :
					   0xFF;
}

uint8_t m2_pwrgd(uint8_t idx)
{
	uint8_t p12v_en_pin, flt_pin;
	uint8_t pwrdis;
	switch (idx) {
	case M2_IDX_E_A:
		flt_pin = IRQ_P12V_E1S_0_FLT_N;
		p12v_en_pin = FM_P12V_E1S_0_EN;
		pwrdis = FM_PWRDIS_E1S_0;
		break;

	case M2_IDX_E_B:
		flt_pin = IRQ_P12V_E1S_1_FLT_N;
		p12v_en_pin = FM_P12V_E1S_1_EN;
		pwrdis = FM_PWRDIS_E1S_1;
		break;

	case M2_IDX_E_C:
		flt_pin = IRQ_P12V_E1S_2_FLT_N;
		p12v_en_pin = FM_P12V_E1S_2_EN;
		pwrdis = FM_PWRDIS_E1S_2;
		break;

	case M2_IDX_E_D:
		flt_pin = IRQ_P12V_E1S_3_FLT_N;
		p12v_en_pin = FM_P12V_E1S_3_EN;
		pwrdis = FM_PWRDIS_E1S_3;
		break;

	default:
		flt_pin = 0xFF;
		p12v_en_pin = 0xFF;
		pwrdis = 0xFF;
		break;
	}

	return ((flt_pin == 0xFF) || (p12v_en_pin == 0xFF) || (pwrdis == 0xFF)) ?
			     0 :
			     (get_fm_p12v_sw_en(idx) && gpio_get(flt_pin) && !gpio_get(pwrdis));
}

uint8_t m2_get_prefix_sen_num(uint8_t idx)
{
	return (idx == M2_IDX_E_A) ? PREFIX_M2A :
	       (idx == M2_IDX_E_B) ? PREFIX_M2B :
	       (idx == M2_IDX_E_C) ? PREFIX_M2C :
	       (idx == M2_IDX_E_D) ? PREFIX_M2D :
					   0xFF;
}

uint8_t m2_prsnt(uint8_t idx)
{
	const uint8_t pin = (idx == M2_IDX_E_A) ? FM_PRSNT_E1S_0_N :
			    (idx == M2_IDX_E_B) ? FM_PRSNT_E1S_1_N :
			    (idx == M2_IDX_E_C) ? FM_PRSNT_E1S_2_N :
			    (idx == M2_IDX_E_D) ? FM_PRSNT_E1S_3_N :
							0xFF;

	if (pin == 0xFF)
		return 0;

	return !gpio_get(pin);
}

uint8_t rst_edsff(uint8_t idx, uint8_t val)
{
	printf("%s()[%d][%d] idx: %d, val: %d\n", __func__, __LINE__, k_uptime_get_32(), idx, val);
	const uint8_t pin = (idx == M2_IDX_E_A) ? RST_BIC_E1S_0_N :
			    (idx == M2_IDX_E_B) ? RST_BIC_E1S_1_N :
			    (idx == M2_IDX_E_C) ? RST_BIC_E1S_2_N :
			    (idx == M2_IDX_E_D) ? RST_BIC_E1S_3_N :
							0xFF;

	if (pin == 0xFF)
		return 1;

	gpio_set(pin, val);
	return 0;
}

bool is_m2_sen_readable(uint8_t sen_num)
{
	uint8_t prefix = sen_num & PREFIX_MASK;

	uint8_t idx = (prefix == PREFIX_M2A) ? M2_IDX_E_A :
		      (prefix == PREFIX_M2B) ? M2_IDX_E_B :
		      (prefix == PREFIX_M2C) ? M2_IDX_E_C :
		      (prefix == PREFIX_M2D) ? M2_IDX_E_D :
						     M2_IDX_E_MAX;

	return (m2_pwrgd(idx) && get_dev_pwrgd(idx)) ? true : false;
}

uint8_t exchange_m2_idx(uint8_t idx) // exchange m2 idx 0/1/2/3 to 3/2/1/0
{
	return ((M2_IDX_E_MAX - 1) - idx);
}
