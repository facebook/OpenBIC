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
