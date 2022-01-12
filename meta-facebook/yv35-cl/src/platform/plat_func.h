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
void ISR_CATERR();
void ISR_PLTRST();
void ISR_DBP_PRSNT();
void ISR_post_complete();
void ISR_SOC_THMALTRIP();
void ISR_SYS_THROTTLE();
void ISR_PCH_THMALTRIP();
void ISR_HSC_OC();
void ISR_CPU_MEMHOT();
void ISR_CPUVR_HOT();
void ISR_PCH_PWRGD();

void set_SCU_setting();
void set_DC_status();
bool get_DC_status();
void set_post_status();
bool get_post_status();
void send_gpio_interrupt(uint8_t gpio_num);
void enable_PRDY_interrupt();
void disable_PRDY_interrupt();
#endif
