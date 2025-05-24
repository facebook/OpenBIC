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

#include <stdint.h>
#include "pldm.h"

extern uint8_t hw_event_register[13];

#define VR_FAULT_STATUS_LSB_MASK 0xFD
#define VR_FAULT_STATUS_MSB_MASK 0xFF
#define VR_IOUT_FAULT_MASK 0x40
#define VR_TPS_OCW_MASK 0x20

typedef struct _add_vr_sel_info {
	bool is_init;
	uint8_t gpio_num;
	uint8_t vr_addr;
	uint8_t page_cnt;
	uint8_t is_asserted;
	struct k_work_delayable add_sel_work;
} add_vr_sel_info;

typedef struct _vr_fault_info {
	uint8_t vr_source;
	uint8_t cpld_reg_data_idx;
	uint8_t cpld_reg_bit;
	bool is_pmbus_vr;
	uint8_t vr_i2c_bus;
	uint8_t vr_addr;
	uint8_t vr_page;
} vr_fault_info;

typedef struct _cpld_reg_info {
	uint8_t cpld_reg_i2c_bus;
	uint8_t cpld_reg_addr;
	uint8_t cpld_reg_offset;
} cpld_reg_info;

enum cpld_reg_info_idx {
	CPLD_REG_INFO_IDX_0 = 0,
	CPLD_REG_INFO_IDX_1,
	CPLD_REG_INFO_IDX_2,
	CPLD_REG_INFO_IDX_MAX
};

typedef struct _add_sel_info {
	bool is_init;
	uint8_t gpio_num;
	volatile uint8_t event_type;
	volatile uint8_t assert_type;
	struct k_work_delayable add_sel_work;
} add_sel_info;

typedef struct {
	struct k_work_delayable work;
	struct pldm_addsel_data sel_data;
} sel_work_wrapper;

void ISR_DC_ON();
void ISR_POST_COMPLETE();
void ISR_BMC_READY();
void ISR_WF_BIC_READY();
void sync_bmc_ready_pin();
void reinit_i3c_hub();
void set_ffwf_eid();
void ISR_SLP3();
void ISR_DBP_PRSNT();
void ISR_MB_THROTTLE();
void ISR_SOC_THMALTRIP();
void ISR_SYS_THROTTLE();
void ISR_HSC_OC();
void ISR_VR_PWR_FAULT();
void ISR_UV_DETECT();
void IST_PLTRST();
void ISR_APML_ALERT();
void ISR_CPU_SMERR_BIC();

void init_vr_event_work();
void process_vr_power_fault_sel(struct k_work *work_item);
void init_event_work();
void addsel_work_handler(struct k_work *work_item);
void init_throttle_work_q();
void init_fastprochot_work_q();

#endif
