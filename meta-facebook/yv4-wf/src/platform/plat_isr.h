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
#include "plat_i2c.h"

#ifndef PLAT_ISR_H
#define PLAT_ISR_H

#define ADDR_IOE1 (0x40 >> 1)
#define ADDR_IOE2 (0x42 >> 1)
#define ADDR_IOE3 (0x44 >> 1)
#define ADDR_IOE4 (0x46 >> 1)

#define CLK_BUFFER_ADDR 0x6B
#define CLK_BUFFER_BUS I2C_BUS6
#define PLL_OPERATING_OFFSET 0x00

#define E1S_PRESENT_BIT BIT(2)
#define ASIC_CLK_BIT BIT(4)
#define E1S_CLK_BIT BIT(5)
#define E1S_PE_RESET_BIT BIT(6)
#define CXL_LED_BIT BIT(2)
// Only switch VR MUX
#define IOE_SWITCH_MUX_TO_BIC 0x05
#define IOE_SWITCH_CXL1_VR_TO_BIC 0x01
#define IOE_SWITCH_CXL2_VR_TO_BIC 0x04

#define IOE_READY_MSEC 1000

#define SET_CLK_BUF_DELAY_MS 100

#define VR_EVENT_DELAY_MS 10

extern bool is_cxl_power_on_success;

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

typedef struct _add_vr_sel_info {
	bool is_init;
	uint8_t vr_num;
	uint8_t vr_i2c_bus;
	uint8_t vr_addr;
	uint8_t page_cnt;
	uint8_t gpio_num;
	uint8_t vr_source;
	struct k_work_delayable add_sel_work;
} add_vr_sel_info;
extern add_vr_sel_info vr_event_work_items[];

#define PMBUS_DRMOS_FAULT 0x80 // MPS MP2971 PMBus Command

typedef struct {
	uint8_t vr_source;
	uint8_t ioe_pin_num;
	uint8_t vr_pwrgd_gpio;
	uint8_t vr_mux_sel;
	uint8_t vr_i2c_bus;
	uint8_t vr_addr;
	uint8_t vr_page;
} vr_fault_info;

typedef enum {
	PMBUS_VR_IOE1_INT = 0,
	NON_PMBUS_VR_PVTT_AB_ASIC1,
	NON_PMBUS_VR_PVTT_AB_ASIC2,
	NON_PMBUS_VR_PVTT_CD_ASIC1,
	NON_PMBUS_VR_PVTT_CD_ASIC2,
	NON_PMBUS_VR_PVPP_AB_ASIC1,
	NON_PMBUS_VR_PVPP_AB_ASIC2,
	NON_PMBUS_VR_PVPP_CD_ASIC1,
	NON_PMBUS_VR_PVPP_CD_ASIC2,
	NON_PMBUS_VR_P12V_E1S_0_R,
	NON_PMBUS_VR_P3V3_E1S_0_R,
} vr_event_index_t;

void ISR_MB_DC_STAGUS_CHAGNE();
void ISR_MB_PCIE_RST();
void ISR_P3V3_E1S_PWR_CHANGE();
void ISR_P12V_E1S_PWR_CHANGE();
void ISR_CXL_PG_ON();
void ISR_PVTT_CD_ASIC1_PWR_CHANGE();
void ISR_PVTT_CD_ASIC2_PWR_CHANGE();
void ISR_IOE1_INT();
void ISR_PVTT_AB_ASIC1_VR_FAULT();
void ISR_PVTT_AB_ASIC2_VR_FAULT();
void ISR_PVPP_AB_ASIC1_VR_FAULT();
void ISR_PVPP_AB_ASIC2_VR_FAULT();
void ISR_PVPP_CD_ASIC1_VR_FAULT();
void ISR_PVPP_CD_ASIC2_VR_FAULT();

void init_vr_event_work();
void process_pmbus_vr_event_handler(struct k_work *work_item);
void process_non_pmbus_vr_event_handler(struct k_work *work_item);

#endif
