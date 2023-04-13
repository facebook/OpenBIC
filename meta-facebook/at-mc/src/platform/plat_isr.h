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

#define CXL_IOEXP_U14_ADDR (0x42 >> 1)
#define CXL_IOEXP_U15_ADDR (0x44 >> 1)
#define CXL_IOEXP_U16_ADDR (0x46 >> 1)
#define CXL_IOEXP_U17_ADDR (0x48 >> 1)
#define CXL_IOEXP_MUX_CHANNEL 0x10
#define CXL_IOEXP_MB_RESET_BIT BIT(0)
#define CXL_IOEXP_DEV_RESET_BIT BIT(2)
#define CXL_IOEXP_ASIC_PERESET_BIT BIT(6)
#define CXL_IOEXP_CONTROLLER_PWRGD_VAL 0x7F
#define CXL_IOEXP_DIMM_PWRGD_VAL 0x07
#define CXL_NOT_ALL_POWER_GOOD 0
#define CXL_ALL_POWER_GOOD 1

#define CXL_CONTROLLER_MUX_CHANNEL 0x01
#define CXL_DRIVE_READY_DELAY_MS 1000
#define CXL_POWER_GOOD_DELAY_MS 12

enum IOEXP_NAME {
	IOEXP_U14,
	IOEXP_U15,
	IOEXP_U16,
	IOEXP_U17,
};

typedef struct _cxl_work_info {
	bool is_init;
	uint8_t cxl_card_id;
	uint8_t cxl_channel;
	bool is_device_reset;
	bool is_pe_reset;
	struct k_work_delayable device_reset_work;
	struct k_work_delayable set_eid_work;
} cxl_work_info;

extern cxl_work_info cxl_work_item[];
extern bool is_interrupt_ongoing;

void ISR_NORMAL_PWRGD();
void ISR_CXL_IOEXP_ALERT0();
void ISR_CXL_IOEXP_ALERT1();
void ISR_CXL_IOEXP_ALERT2();
void ISR_CXL_IOEXP_ALERT3();
void ISR_CXL_IOEXP_ALERT4();
void ISR_CXL_IOEXP_ALERT5();
void ISR_CXL_IOEXP_ALERT6();
void ISR_CXL_IOEXP_ALERT7();

void init_cxl_work();
int check_cxl_power_status(bool is_mutex);
int set_cxl_device_reset_pin(uint8_t val, bool is_mutex);
void cxl_ioexp_alert_handler(struct k_work *work_item);

#endif
