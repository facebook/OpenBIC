#ifndef PLAT_CLASS_H
#define PLAT_CLASS_H

#include "hal_gpio.h"
#include "plat_gpio.h"

enum PALTFORM_BOARD_ID {
	FM_BOARD_ID_0 = BOARD_ID0,
	FM_BOARD_ID_1 = BOARD_ID1,
	FM_BOARD_ID_2 = BOARD_ID2,
	FM_BOARD_ID_3 = BOARD_ID3,
};

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

void init_platform_config();
void init_e1s_config();
void init_sys_board_id(uint8_t board_id);
uint8_t get_board_id();
uint8_t get_e1s_hsc_config();
uint8_t get_e1s_adc_config();
uint8_t get_e1s_pwrgd();

#endif
