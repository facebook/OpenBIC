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

#include <kernel.h>
#include "ioexp_tca9555.h"

#ifndef PLAT_ISR_H
#define PLAT_ISR_H

#define ADDR_IOE1 (0x40 >> 1)
#define ADDR_IOE2 (0x42 >> 1)
#define ADDR_IOE3 (0x44 >> 1)
#define ADDR_IOE4 (0x46 >> 1)

#define E1S_PRESENT_BIT BIT(2)
#define ASIC_CLK_BIT BIT(4)
#define E1S_CLK_BIT BIT(5)
#define E1S_PE_RESET_BIT BIT(6)
#define CXL_LED_BIT BIT(2)
// Only switch VR MUX
#define IOE_SWITCH_MUX_TO_BIC 0x05

#define IOE_READY_MSEC 1000

enum set_ioe4_cmd {
	SET_CLK = 0,
	SET_PE_RST,
};

typedef struct {
	uint8_t addr;
	uint8_t conf_reg;
	uint8_t conf_dir;
	uint8_t output_reg;
	uint8_t output_val;
} IOE_CFG;

void ISR_MB_DC_STAGUS_CHAGNE();
void ISR_MB_PCIE_RST();
void ISR_E1S_PWR_ON();
void ISR_CXL_PG_ON();
void ISR_SET_CXL_LED();

#endif
