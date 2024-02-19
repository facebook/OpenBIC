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

#ifndef PLAT_PWR_SEQ_H
#define PLAT_PWR_SEQ_H

#include "plat_gpio.h"

#define DC_ON_DELAY5_SEC 5
#define CXL_READY_SECONDS 30
#define CHK_PWR_DELAY_MSEC 100
#define SYS_CLK_STABLE_DELAY_MSEC 25
#define PWR_RST_DELAY_MSEC 25
#define P1V8_POWER_OFF_DELAY_MSEC 3500

#define POWER_SEQ_CTRL_STACK_SIZE 1000

typedef struct _cxl_power_control_gpio {
	int enclk_100m_osc;
	int p075v_asic_en;
	int p08v_asic_en;
	int p085v_asic_en;
	int p1v2_asic_en;
	int p1v8_asic_en;
	int sys_rst;
	int pwr_on_rst;
	int pvpp_ab_dimm_en;
	int pvpp_cd_dimm_en;
	int pvddq_ab_dimm_en;
	int pvddq_cd_dimm_en;
	int pvtt_ab_dimm_en;
	int pvtt_cd_dimm_en;
} cxl_power_control_gpio;

typedef struct _cxl_power_good_gpio {
	int p075v_asic_pg;
	int p08v_asic_pg;
	int p085v_asic_pg;
	int p1v2_asic_pg;
	int p1v8_asic_pg;
	int pvpp_ab_dimm_pg;
	int pvpp_cd_dimm_pg;
	int pvddq_ab_dimm_pg;
	int pvddq_cd_dimm_pg;
	int pvtt_ab_dimm_pg;
	int pvtt_cd_dimm_pg;
} cxl_power_good_gpio;

enum CXL_NUM {
	CXL_ID_0 = 0,
	CXL_ID_1,
	MAX_CXL_ID,
};

enum POWER_ON_STAGE {
	CLK_POWER_ON_STAGE = 0,
	ASIC_POWER_ON_STAGE_1,
	ASIC_POWER_ON_STAGE_2,
	DIMM_POWER_ON_STAGE_1,
	DIMM_POWER_ON_STAGE_2,
	DIMM_POWER_ON_STAGE_3,
	MAX_POWER_ON_STAGES,
};

enum POWER_OFF_STAGE {
	DIMM_POWER_OFF_STAGE_1 = 0,
	DIMM_POWER_OFF_STAGE_2,
	DIMM_POWER_OFF_STAGE_3,
	ASIC_POWER_OFF_STAGE_1,
	ASIC_POWER_OFF_STAGE_2,
	ASIC_POWER_OFF_STAGE_3,
	CLK_POWER_OFF_STAGE,
	MAX_POWER_OFF_STAGES,
};

void set_mb_dc_status(uint8_t gpio_num);
void enable_asic1_rst();
void enable_asic2_rst();
bool is_power_controlled(int cxl_id, int power_pin, uint8_t check_power_status, char *power_name);
int check_powers_enabled(int cxl_id, int pwr_stage);
int check_powers_disabled(int cxl_id, int pwr_stage);
void enable_powers(int cxl_id, int pwr_stage);
void disable_powers(int cxl_id, int pwr_stage);
int power_on_handler(int cxl_id, int power_stage);
int power_off_handler(int cxl_id, int power_stage);
void execute_power_on_sequence();
void execute_power_off_sequence();
void cxl_ready_handler();
bool get_cxl_ready_status(uint8_t cxl_id);
bool cxl_ready_access(uint8_t sensor_num);

#endif
