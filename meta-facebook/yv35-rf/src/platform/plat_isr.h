#ifndef PLAT_ISR_H
#define PLAT_ISR_H

void control_power_sequence();
void init_power_on_thread();
void init_power_off_thread();
void abort_power_thread();
void ISR_MB_DC_STATE();

#endif
