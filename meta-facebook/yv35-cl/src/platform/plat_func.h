#ifndef PLAT_FUNC_H
#define PLAT_FUNC_H
#include <stdbool.h>

void ISR_usbhub();
void ISR_pltrst();
void ISR_slp3();
void ISR_DC_on();
void ISR_post_complete();

bool get_DC_status();
bool get_post_status();
#endif
