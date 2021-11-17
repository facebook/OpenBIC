#ifndef PLAT_FUNC_H
#define PLAT_FUNC_H
#include <stdbool.h>
#include <stdint.h>

void ISR_usbhub();
void ISR_pltrst();
void ISR_slp3();
void ISR_DC_on();
void ISR_BMC_PRDY();
void ISR_PWRGD_CPU();
void ISR_PLTRST();
void ISR_DBP_PRSNT();
void ISR_post_complete();

void set_DC_status();
bool get_DC_status();
void set_post_status();
bool get_post_status();
void send_gpio_interrupt(uint8_t gpio_num);
#endif
