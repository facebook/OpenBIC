#ifndef PLAT_SYS_H
#define PLAT_SYS_H

#include "stdint.h"
#include <stdbool.h>

enum BUTTON_ID {
	BASEBOARD_SLED_BUTTON = 0x00,
	SLOT1_SLOT_BUTTON = 0x01,
	SLOT3_SLOT_BUTTON = 0x03,
};

enum SLOT_12V_STATUS {
	SLOT_12V_OFF = 0x00,
	SLOT_12V_ON = 0x01,
};

void control_slot_12V_power(uint8_t slot_id, uint8_t control_mode);
void submit_button_event(uint8_t button_id, uint8_t target_slot, uint8_t event_type);

#endif
