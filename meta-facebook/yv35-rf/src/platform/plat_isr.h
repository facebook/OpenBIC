#ifndef PLAT_ISR_H
#define PLAT_ISR_H

void control_power_sequence();
void init_power_on_thread();
void init_power_off_thread();
void abort_power_thread();
void check_power_abnormal(uint8_t power_good_gpio_num, uint8_t control_power_gpio_num);
void ISR_MB_DC_STATE();
void ISR_DC_STATE();
void ISR_MB_RST();
void ISR_P0V8_ASICA_POWER_GOOD_LOST();
void ISR_P0V8_ASICD_POWER_GOOD_LOST();
void ISR_P0V9_ASICA_POWER_GOOD_LOST();
void ISR_P1V8_ASIC_POWER_GOOD_LOST();
void ISR_PVPP_AB_POWER_GOOD_LOST();
void ISR_PVPP_CD_POWER_GOOD_LOST();
void ISR_PVDDQ_AB_POWER_GOOD_LOST();
void ISR_PVDDQ_CD_POWER_GOOD_LOST();
void ISR_PVTT_AB_POWER_GOOD_LOST();
void ISR_PVTT_CD_POWER_GOOD_LOST();

#endif
