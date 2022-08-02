#include <stdio.h>
#include "hal_gpio.h"
#include "plat_class.h"

static uint8_t system_board_id = 0;
static uint8_t e1s_hsc_config = 0;
static uint8_t e1s_adc_config = 0;

#define VERNAL_FALLS_BOARD_TYPE 0x07

void init_sys_board_id(uint8_t board_id)
{
	switch (board_id) {
	case RAINBOW_FALLS:
		system_board_id = RAINBOW_FALLS;
		break;
	case WAIMANO_FALLS:
		system_board_id = WAIMANO_FALLS;
		break;
	case VERNAL_FALLS:
		system_board_id = VERNAL_FALLS_BOARD_TYPE;
		break;
	default:
		printf("[%s] input board id not support: 0x%x\n", __func__, board_id);
		system_board_id = UNKNOWN_BOARD;
	}
}

void init_platform_config()
{
	uint8_t board_id = 0;
	board_id = ((gpio_get(BOARD_ID_BIT_3) << 3) | (gpio_get(BOARD_ID_BIT_2) << 2) |
		    (gpio_get(BOARD_ID_BIT_1) << 1) | (gpio_get(BOARD_ID_BIT_0) << 0));

	init_sys_board_id(board_id);
	printf("[%s] board id 0x%x\n", __func__, system_board_id);
}

uint8_t get_board_id()
{
	return system_board_id;
}

void init_e1s_config()
{
	uint8_t config = 0;
	config = ((gpio_get(HSC_SEL_ID0)) | (gpio_get(HSC_SEL_ID1) << 1) |
		  (gpio_get(HSC_SEL_ID2) << 2));

	e1s_hsc_config = config & 0x03;
	e1s_adc_config = config >> 2;
	printf("[%s] hsc config 0x%x\n", __func__, e1s_hsc_config);
	printf("[%s] adc config 0x%x\n", __func__, e1s_adc_config);
}

uint8_t get_e1s_hsc_config()
{
	return e1s_hsc_config;
}

uint8_t get_e1s_adc_config()
{
	return e1s_adc_config;
}

uint8_t get_e1s_pwrgd()
{
	switch (get_e1s_hsc_config()) {
	case CONFIG_HSC_ADM1278:
	case CONFIG_HSC_MAXIN:
	case CONFIG_HSC_MPS:
		return gpio_get(PWRGD_P12V_AUX);

	case CONFIG_HSC_BYPASS:
	default:
		return gpio_get(FM_POWER_EN);
	}
}