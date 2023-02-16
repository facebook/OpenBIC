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

#define IPMI_EVENT_OFFSET_SYS_INA233_ALERT 0x86

enum INA233_ALERT_INDEX {
	INA233_ALERT_E1S_0 = 0,
	INA233_ALERT_E1S_1,
	INA233_ALERT_E1S_2,
	INA233_ALERT_E1S_3,
	INA233_ALERT_E1S_4,
	INA233_ALERT_E1S_P12V_EDGE,
	INA233_ALERT_E1S_P12V_MAIN,
};

void control_power_sequence();
void init_power_on_thread();
void init_power_off_thread();
void abort_power_thread();
void ISR_FM_EXP_MAIN_PWR_EN();
void ISR_E1S_0_INA233_ALERT();
void ISR_E1S_1_INA233_ALERT();
void ISR_E1S_2_INA233_ALERT();
void ISR_E1S_3_INA233_ALERT();
void ISR_E1S_4_INA233_ALERT();
void ISR_E1S_P12V_EDGE_INA233_ALERT();
void ISR_E1S_P12V_MAIN_INA233_ALERT();

#endif
