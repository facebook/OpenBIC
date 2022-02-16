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
void ISR_MB_THROTTLE();
void ISR_HSC_THROTTLE();
void ISR_FM_THROTTLE();
void ISR_RMCA();

void set_SCU_setting();
void set_DC_status();
bool get_DC_status();
void set_post_status();
bool get_post_status();
void set_DC_on_5s_status();
bool get_DC_on_5s_status();
void set_DC_off_10s_status();
void set_sys_config();
void set_post_thread();
bool get_bic_class();
bool get_1ou_status();
bool get_2ou_status();
uint8_t get_2ou_cardtype();
void send_gpio_interrupt(uint8_t gpio_num);
void enable_PRDY_interrupt();
void disable_PRDY_interrupt();
void set_ME_restore();
void pal_warm_reset_prepare();
#endif
