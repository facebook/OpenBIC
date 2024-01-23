#ifndef PLAT_CLASS_H
#define PLAT_CLASS_H

#include <stdbool.h>
#include <stdint.h>

#define CHANNEL_2 2
#define CHANNEL_13 13
#define NUMBER_OF_ADC_CHANNEL 16
#define AST1030_ADC_BASE_ADDR 0x7e6e9000

enum SLOT_EID {
	SLOT1 = 0x0A,
	SLOT2 = 0x14,
	SLOT3 = 0x1E,
	SLOT4 = 0x28,
	SLOT5 = 0x32,
	SLOT6 = 0x3C,
	SLOT7 = 0x46,
	SLOT8 = 0x50,
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

bool get_adc_voltage(int channel, float *voltage);
uint8_t get_slot_eid();
void init_platform_config();

#endif
