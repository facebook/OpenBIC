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

void ISR_GPIO_ALL_VR_PM_ALERT_R_N();
void ISR_GPIO_FM_PLD_UBC_EN_R();
void ISR_GPIO_RST_IRIS_PWR_ON_PLD_R1_N();
void set_pwr_steps_on_flag(uint8_t flag_value);
uint8_t get_pwr_steps_on_flag(void);
void ISR_GPIO_SMB_HAMSA_MMC_LVC33_ALERT_N();
#endif
