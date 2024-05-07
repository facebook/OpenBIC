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

#ifndef PLAT_PWRSEQ_H
#define PLAT_PWRSEQ_H

#include "hal_gpio.h"
#include "plat_gpio.h"

#define MAX_E1S_IDX 5
#define OPA_MAX_E1S_IDX 3
#define ALL_E1S 0xFF
#define POWER_SEQ_CTRL_STACK_SIZE 1000
#define CHKPWR_DELAY_MSEC 100
#define RETIMER_DELAY_MSEC 2000
#define DEV_RESET_DELAY_USEC 100

enum CONTROL_POWER_MODE {
	ENABLE_POWER_MODE = 0x00,
	LOW_ENABLE_POWER_MODE,
	DISABLE_POWER_MODE,
	HIGH_DISABLE_POWER_MODE,
};

enum POWER_ON_STAGE {
	BOARD_POWER_ON_STAGE0 = 0x00,
	BOARD_POWER_ON_STAGE1,
	BOARD_POWER_ON_STAGE2,
	RETIMER_POWER_ON_STAGE0,
	RETIMER_POWER_ON_STAGE1,
	E1S_POWER_ON_STAGE0,
	E1S_POWER_ON_STAGE1,
	E1S_POWER_ON_STAGE2,
	E1S_POWER_ON_STAGE3,
};

enum POWER_OFF_STAGE {
	E1S_POWER_OFF_STAGE0 = 0x00,
	E1S_POWER_OFF_STAGE1,
	E1S_POWER_OFF_STAGE2,
	E1S_POWER_OFF_STAGE3,
	RETIMER_POWER_OFF_STAGE0,
	RETIMER_POWER_OFF_STAGE1,
	RETIMER_POWER_OFF_STAGE2,
	BOARD_POWER_OFF_STAGE0,
	BOARD_POWER_OFF_STAGE1,
	BOARD_POWER_OFF_STAGE2,
};

enum CHECK_POWER_SEQ_NUM_MAPPING {
	CHECK_POWER_SEQ_01 = FM_EXP_MAIN_PWR_EN,
	CHECK_POWER_SEQ_02 = PWRGD_P12V_MAIN,
	CHECK_POWER_SEQ_03 = OPA_PWRGD_P1V8_VR,
	CHECK_POWER_SEQ_04 = OPA_PWRGD_P0V9_VR,
	CHECK_POWER_SEQ_05 = OPA_PWRGD_EXP_PWR,
	CHECK_POWER_SEQ_06 = OPA_CLKBUF_RTM_OE_N,
	CHECK_POWER_SEQ_07 = OPA_RESET_BIC_RTM_N,
	CHECK_POWER_SEQ_08 = OPA_PERST_BIC_RTM_N,
};

typedef enum {
	E1S_POWER_SUCCESS = 0,
	E1S_PERST_SUCCESS = 1,
} E1S_POWER_ON_STATUS;

typedef struct _e1s_power_control_gpio {
	uint8_t present;
	uint8_t p12v_efuse_enable;
	uint8_t p12v_efuse_power_good;
	uint8_t p3v3_efuse_enable;
	uint8_t p3v3_efuse_power_good;
	uint8_t clkbuf_oe_en;
	uint8_t cpu_pcie_reset;
	uint8_t e1s_pcie_reset;
} e1s_power_control_gpio;

extern e1s_power_control_gpio opa_e1s_power_control_gpio[];
extern e1s_power_control_gpio opb_e1s_power_control_gpio[];

bool get_e1s_present(uint8_t index);
bool get_e1s_power_good(uint8_t index);
bool get_edge_power_good();
uint8_t get_e1s_pcie_reset_status(uint8_t index);
void init_sequence_status();
void set_sequence_status(uint8_t index, bool status);
bool is_all_sequence_done(uint8_t status);
bool is_retimer_done(void);
void abort_e1s_power_thread(uint8_t index);
void e1s_power_on_thread(uint8_t index, uint8_t initial_stage);
void e1s_power_off_thread(uint8_t index);
void control_power_on_sequence(void *initial_stage, void *arvg0, void *arvg1);
void control_power_off_sequence();
void control_power_stage(uint8_t control_mode, uint8_t control_seq);
int check_power_stage(uint8_t check_mode, uint8_t check_seq);
bool e1s_power_on_handler(uint8_t initial_stage, e1s_power_control_gpio *e1s_gpio,
			  uint8_t device_index);
bool e1s_power_off_handler(uint8_t initial_stage, e1s_power_control_gpio *e1s_gpio,
			   uint8_t device_index);
bool power_on_handler(uint8_t initial_stage);
bool power_off_handler(uint8_t initial_stage);
bool notify_cpld_e1s_present(uint8_t index, uint8_t present);
void abort_cpu_perst_low_thread();
void cpu_perst_low_thread();

#endif
