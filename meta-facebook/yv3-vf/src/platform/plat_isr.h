#ifndef PLAT_ISR_H
#define PLAT_ISR_H

void dev_12v_fault_hander(void);
void pwrgd_p12v_aux_int_handler(void);
void dev_rst(void);
uint8_t check_12v_dev_pwrgd(void);
void dev_12v_fault_hander(void);
void aux_pwr_en_int_handler(void);
void power_en_int_handler(void);
void rst_mb_n_int_handler(void);

void prsnt_int_handler_dev0();
void prsnt_int_handler_dev1();
void prsnt_int_handler_dev2();
void prsnt_int_handler_dev3();

void dev_12v_fault_hander_dev0(void);
void dev_12v_fault_hander_dev1(void);
void dev_12v_fault_hander_dev2(void);
void dev_12v_fault_hander_dev3(void);

void ina231_alert_handler_m2_dev0(void);
void ina231_alert_handler_m2_dev1(void);
void ina231_alert_handler_m2_dev2(void);
void ina231_alert_handler_m2_dev3(void);

#endif
