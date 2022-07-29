#ifndef PLAT_CLASS_H
#define PLAT_CLASS_H

#include "hal_gpio.h"
#include "plat_gpio.h"

enum E1S_CONFIG_HSC {
	CONFIG_HSC_ADM1278 = 0x00,
	CONFIG_HSC_MAXIN,
	CONFIG_HSC_MPS,
	CONFIG_HSC_BYPASS,
};

enum E1S_CONFIG_ADC {
	CONFIG_ADC_INA231 = 0x00,
	CONFIG_ADC_ISL28022,
};

void init_e1s_config();
uint8_t get_e1s_hsc_config();
uint8_t get_e1s_adc_config();
uint8_t get_e1s_pwrgd();

#endif
