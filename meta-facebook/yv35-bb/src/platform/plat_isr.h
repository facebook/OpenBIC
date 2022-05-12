#ifndef PLAT_FUNC_H
#define PLAT_FUNC_H

#include <stdint.h>

#define MAX_PRESS_SLED_BTN_TIME_s 4
#define SYSTEM_PRESENT 0x0
#define SYSTEM_ABSENT 0x1
#define CABLE_PRESENT 0x0
#define CABLE_ABSENT 0x1

void ISR_PWROK_SLOT1();
void ISR_PWROK_SLOT3();
void ISR_SLED_CYCLE();
void ISR_SLOT1_PRESENT();
void ISR_SLOT3_PRESENT();
void set_BIC_slot_isolator(uint8_t pwr_state_gpio_num, uint8_t isolator_gpio_num);

void set_sled_cycle();

#endif
