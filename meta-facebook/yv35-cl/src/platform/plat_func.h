#ifndef PLAT_FUNC_H
#define PLAT_FUNC_H

void ISR_usbhub();
void ISR_pltrst();
void ISR_slp3();
void ISR_DC_on();

bool get_DC_status();

#endif
