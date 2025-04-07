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

void ISR_GPIO_FM_ASIC_0_THERMTRIP_R_N();
void ISR_GPIO_RST_ATH_PWR_ON_PLD_R1_N();
void ISR_GPIO_ATH_CURRENT_SENSE_0_NPCM_R();
void ISR_GPIO_ATH_CURRENT_SENSE_1_NPCM_R();
void ISR_GPIO_FM_ATH_HBM3_CATTRIP_ALARM_LV33_R();
void ISR_GPIO_ALL_VR_PM_ALERT_R_N();
void ISR_GPIO_ATH_SMB_ALERT_NPCM_LVC33_R_N();
void ISR_GPIO_FM_PLD_UBC_EN_R();
bool plat_i2c_read(uint8_t bus, uint8_t addr, uint8_t offset, uint8_t *data, uint8_t len);
bool plat_i2c_write(uint8_t bus, uint8_t addr, uint8_t offset, uint8_t *data, uint8_t len);

void plat_clock_init();
void plat_eusb_init();
bool plat_power_control(bool is_power_on);

#endif
