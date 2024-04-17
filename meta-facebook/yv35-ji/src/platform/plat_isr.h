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

#include <stdint.h>

void ISR_E1S_ALERT();
void ISR_RTC_ALERT();
void ISR_GPIOA5();
void ISR_GPIOA6();
void ISR_GPIOB0();
void ISR_GPIOB7();
void ISR_HSC_OC();
void ISR_GPIOC1();
void ISR_GPIOC3();
void ISR_GPIOC4();
void ISR_GPIOD0();
void ISR_PWRGD_CPU();
void ISR_GPIOE2();
void ISR_HSC_THROTTLE();
void ISR_GPIOE4();
void ISR_GPIOE5();
void ISR_GPIOE6();
void ISR_GPIOE7();
void ISR_CPU_HIGHTEMP();
void ISR_CPU_FAULT_ALERT();
void ISR_GPIOF3();
void ISR_GPIOF4();
void ISR_GPIOF6();
void ISR_GPIOF7();
void ISR_MB_THROTTLE();
void ISR_CPU_OVERTEMP();
void ISR_GPIOH0();
void ISR_GPIOH1();
void ISR_SYS_THROTTLE();

#endif
