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

#ifndef PLAT_FUNC_H
#define PLAT_FUNC_H

void ISR_DC_ON();
void ISR_NIC_ADC_ALERT();
void ISR_SSD_0_7_ADC_ALERT();
void ISR_SSD_8_15_ADC_ALERT();
void ISR_PEX_ADC_ALERT();
void ISR_SMB_FPGA_ALERT();
void ISR_VR_PMBUS_ALERT();
void ISR_HSC_SMB_ALERT();

void ISR_SSD0_PRESENT();
void ISR_SSD1_PRESENT();
void ISR_SSD2_PRESENT();
void ISR_SSD3_PRESENT();
void ISR_SSD4_PRESENT();
void ISR_SSD5_PRESENT();
void ISR_SSD6_PRESENT();
void ISR_SSD7_PRESENT();
void ISR_SSD8_PRESENT();
void ISR_SSD9_PRESENT();
void ISR_SSD10_PRESENT();
void ISR_SSD11_PRESENT();
void ISR_SSD12_PRESENT();
void ISR_SSD13_PRESENT();
void ISR_SSD14_PRESENT();
void ISR_SSD15_PRESENT();

#endif
