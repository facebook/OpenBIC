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

#ifndef UTIL_SYS_H
#define UTIL_SYS_H

#include <sys/reboot.h>
#include "stdbool.h"
#include "stdint.h"

#define INTEL_IANA 0x000157

enum CC_12V_CYCLE_SLOT {
	SUCCESS_12V_CYCLE_SLOT,
	NOT_SUPPORT_12V_CYCLE_SLOT,
	SLOT_OFF_FAILED,
	SLOT_ON_FAILED
};

enum ME_MODE {
	ME_INIT_MODE = 0x00,
	ME_NORMAL_MODE = 0x01,
	ME_RECOVERY_MODE = 0x02,
};

enum FORCE_ME_RECOVERY_CMD {
	ME_FW_RECOVERY = 0x01,
	ME_FW_RESTORE = 0x02,
};

enum SYSTEM_RESET_TYPE {
	// Aspeed system warm reset default setting is SOC reset, and system cold reset is full chip reset
	SOC_RESET = SYS_REBOOT_WARM,
	FULL_CHIP_RESET = SYS_REBOOT_COLD,
};

typedef enum {
	VENDOR_RENESAS = 0,
	VENDOR_TI,
	VENDOR_INFINEON,
	VENDOR_VISHAY,
	VENDOR_ONSEMI,
	VENDOR_MPS,
} VR_VENDOR_ID;

typedef enum {
	POWER_CTL_ON,
	POWER_CTL_OFF,
	POWER_CTL_RESET,
	MAX_POWER_CTL_COUNT,
} power_ctl_t;

extern uint8_t ISL69254_DEVICE_ID[5];
extern uint8_t XDPE12284C_DEVICE_ID[3];
extern uint8_t ISL69259_DEVICE_ID[5];

void submit_bic_cold_reset();
void bic_cold_reset();
void submit_bic_warm_reset();
void bic_warm_reset();
void check_ac_lost();
bool is_ac_lost();
void pal_warm_reset_prepare();
void pal_cold_reset_prepare();
int pal_submit_bmc_cold_reset();
int pal_host_power_control(power_ctl_t ctl_type);
bool pal_is_bmc_present();
bool pal_is_bmc_ready();
int submit_12v_cycle_slot();

int set_me_firmware_mode(uint8_t me_fw_mode);
void init_me_firmware();
uint8_t get_me_mode();

int pal_submit_bmc_cold_reset();
int pal_submit_12v_cycle_slot();

int pal_clear_cmos();

void set_sys_ready_pin(uint8_t ready_gpio_name);

uint8_t get_system_class();

int pal_get_set_add_debug_sel_mode_status(uint8_t options, uint8_t *status);

#endif
