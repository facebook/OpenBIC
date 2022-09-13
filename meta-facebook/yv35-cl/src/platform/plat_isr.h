#ifndef PLAT_FUNC_H
#define PLAT_FUNC_H

#include <stdint.h>

void send_gpio_interrupt(uint8_t gpio_num);
void ISR_PLTRST();
void ISR_SLP3();
void ISR_DC_ON();
void ISR_BMC_PRDY();
void ISR_PWRGD_CPU();
void ISR_CATERR();
void ISR_DBP_PRSNT();
void ISR_POST_COMPLETE();
void ISR_SOC_THMALTRIP();
void ISR_SYS_THROTTLE();
void ISR_PCH_THMALTRIP();
void ISR_HSC_OC();
void ISR_CPU_MEMHOT();
void ISR_CPUVR_HOT();
void ISR_PCH_PWRGD();
void ISR_MB_THROTTLE();
void ISR_HSC_THROTTLE();
void ISR_FM_THROTTLE();
void ISR_RMCA();
void ISR_CPU_VPP_INT();

#endif
