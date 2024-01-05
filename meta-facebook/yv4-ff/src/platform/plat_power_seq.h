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
// BIC checks for CXL readiness every three seconds, with a maximum of 10 attempts
#define CXL_READY_RETRY_TIMES 10
#define CXL_READY_INTERVAL_SECONDS 3
#define CXL_HEART_BEAT_LABEL "HB0"
#define CHK_PWR_DELAY_MSEC 100
#define SYS_CLK_STABLE_DELAY_MSEC 25
#define PWR_ON_RST_DELAY_MSEC 25
#define P1V8_POWER_OFF_DELAY_MSEC 3500

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
void enable_power_on_rst();
bool is_power_controlled(uint8_t power_pin, uint8_t check_power_status, char *power_name);
int check_powers_enabled(uint8_t pwr_stage);
int check_powers_disabled(uint8_t pwr_stage);
void enable_powers(uint8_t pwr_stage);
void disable_powers(uint8_t pwr_stage);
int power_on_handler(uint8_t power_stage);
int power_off_handler(uint8_t power_stage);
void execute_power_on_sequence();
void execute_power_off_sequence();
void cxl_ready_handler();
bool get_cxl_ready_status();
bool cxl_ready_access(uint8_t sensor_num);

#endif
