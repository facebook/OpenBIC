#ifndef PLAT_FUNC_H
#define PLAT_FUNC_H
#include <stdbool.h>


void ISR_PWROK_SLOT1();
void ISR_PWROK_SLOT3();

void set_BIC_slot_isolator(uint8_t pwr_state_gpio_num, uint8_t isolator_gpio_num);

#endif
