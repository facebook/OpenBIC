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

#define ADDR_IOE4 (0x46 >> 1)

#define E1S_PRESENT_BIT BIT(2)
#define ASIC_CLK_BIT BIT(4)
#define E1S_CLK_BIT BIT(5)
#define E1S_PE_RESET_BIT BIT(6)

#define IOE4_CONFIGUTATION_PINS 0x8f

enum set_ioe4_cmd {
	SET_CLK = 0,
	SET_PE_RST,
};

void ISR_MB_DC_STAGUS_CHAGNE();
void ISR_MB_PCIE_RST();
void ISR_E1S_PWR_ON();

void set_ioe4_pin();

#endif
