/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef PLAT_ISR_H
#define PLAT_ISR_H

enum IRQ_FLT_TYPE {
	P12V_E1S = 0,
	P3V3_E1S,
	P12V_EDGE,
};

void dev_12v_fault_handler(void);
void pwrgd_p12v_aux_int_handler(void);
void dev_rst(void);
uint8_t check_12v_dev_pwrgd(void);
void dev_12v_fault_handler(void);
void aux_pwr_en_int_handler(void);
void power_en_int_handler(void);
void rst_mb_n_int_handler(void);
void check_irq_fault(void);

void prsnt_int_handler_dev0();
void prsnt_int_handler_dev1();
void prsnt_int_handler_dev2();
void prsnt_int_handler_dev3();

void dev_12v_fault_handler_dev0(void);
void dev_12v_fault_handler_dev1(void);
void dev_12v_fault_handler_dev2(void);
void dev_12v_fault_handler_dev3(void);

void ina231_alert_handler_m2_dev0(void);
void ina231_alert_handler_m2_dev1(void);
void ina231_alert_handler_m2_dev2(void);
void ina231_alert_handler_m2_dev3(void);

void dev_3v3_fault_handler_dev0(void); // IRQ_P3V3_E1S_0_FLT_N
void dev_3v3_fault_handler_dev1(void); // IRQ_P3V3_E1S_1_FLT_N
void dev_3v3_fault_handler_dev2(void); // IRQ_P3V3_E1S_2_FLT_N
void dev_3v3_fault_handler_dev3(void); // IRQ_P3V3_E1S_3_FLT_N
void p12v_edge_fault_sel(void); // IRQ_P12V_EDGE_FLT_N

#endif
