#ifndef PLAT_CLASS_H
#define PLAT_CLASS_H

#include <stdbool.h>
#include <stdint.h>

#define AST1030_ADC_BASE_ADDR 0x7e6e9000

enum SLOT_EID {
	SLOT1_EID = 0x0A,
	SLOT2_EID = 0x14,
	SLOT3_EID = 0x1E,
	SLOT4_EID = 0x28,
	SLOT5_EID = 0x32,
	SLOT6_EID = 0x3C,
	SLOT7_EID = 0x46,
	SLOT8_EID = 0x50,
};

enum SLOT_PID {
	SLOT1_PID = 0x0000,
	SLOT2_PID = 0x0005,
	SLOT3_PID = 0x000A,
	SLOT4_PID = 0x000F,
	SLOT5_PID = 0x0014,
	SLOT6_PID = 0x0019,
	SLOT7_PID = 0x001E,
	SLOT8_PID = 0x0023,
};

enum BOARD_REV_ID {
	BOARD_REV_POC = 0x00,
	BOARD_REV_EVT = 0x01,
	BOARD_REV_DVT = 0x03,
	BOARD_REV_PVT = 0x05,
	BOARD_REV_MP = 0x06,
};

enum ADC_CHANNEL {
	ADC_CHANNEL_2 = 2,
	ADC_CHANNEL_12 = 12,
	ADC_CHANNEL_13 = 13,
	NUMBER_OF_ADC_CHANNEL = 16,
};

enum BLADE_CONFIG {
	BLADE_CONFIG_T1M = 0x00,
	BLADE_CONFIG_T1C = 0x10,
	BLADE_CONFIG_UNKNOWN = 0xff,
};

enum RETIMER_TYPE {
	RETIMER_TYPE_ASTERALABS,
	RETIMER_TYPE_NO_RETIMER,
	RETIMER_TYPE_KANDOU,
	RETIMER_TYPE_BROADCOM,
};

bool get_adc_voltage(int channel, float *voltage);
bool get_board_rev(uint8_t *board_rev);
uint8_t get_slot_eid();
uint8_t get_slot_id();
bool get_blade_config(uint8_t *blade_config);
void init_platform_config();
uint8_t get_retimer_type();
void init_retimer_type();

#endif
