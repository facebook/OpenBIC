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

#define POWER_SEQ_HANDLER_STACK_SIZE 1000
#define CHECKK_POWER_DELAY_MS 100

enum POWER_ON_STAGE {
	ASIC_POWER_ON_STAGE1 = 0x00,
	ASIC_POWER_ON_STAGE2,
	DIMM_POWER_ON_STAGE1,
	DIMM_POWER_ON_STAGE2,
	DIMM_POWER_ON_STAGE3,
};

enum POWER_OFF_STAGE {
	DIMM_POWER_OFF_STAGE1 = 0x00,
	DIMM_POWER_OFF_STAGE2,
	DIMM_POWER_OFF_STAGE3,
	ASIC_POWER_OFF_STAGE1,
	ASIC_POWER_OFF_STAGE2,
	BOARD_POWER_OFF_STAGE,
};

enum CONTROL_POWER_SEQ_PIN_MAPPING {
	CONTROL_POWER_SEQ_01 = P0V75_ASIC_EN_BIC,
	CONTROL_POWER_SEQ_02 = P0V8_BIC_ASIC_EN,
	CONTROL_POWER_SEQ_03 = P0V85_BIC_ASIC_EN,
	CONTROL_POWER_SEQ_04 = P1V2_BIC_ASIC_EN,
	CONTROL_POWER_SEQ_05 = P1V8_BIC_ASIC_EN,
	CONTROL_POWER_SEQ_06 = PVPP_AB_BIC_DIMM_EN,
	CONTROL_POWER_SEQ_07 = PVPP_CD_BIC_DIMM_EN,
	CONTROL_POWER_SEQ_08 = PVDDQ_AB_EN_BIC,
	CONTROL_POWER_SEQ_09 = PVDDQ_CD_EN_BIC,
	CONTROL_POWER_SEQ_10 = PVTT_AB_BIC_DIMM_EN,
	CONTROL_POWER_SEQ_11 = PVTT_CD_BIC_DIMM_EN,
};

enum CHECK_POWER_SEQ_PIN_MAPPING {
	CHECK_POWER_SEQ_01 = PWRGD_P0V75_ASIC_BIC,
	CHECK_POWER_SEQ_02 = PWRGD_P0V8_ASIC,
	CHECK_POWER_SEQ_03 = PWRGD_P0V85_ASIC,
	CHECK_POWER_SEQ_04 = PWRGD_P1V2_ASIC,
	CHECK_POWER_SEQ_05 = PWRGD_P1V8_ASIC,
	CHECK_POWER_SEQ_06 = PWRGD_PVPP_AB,
	CHECK_POWER_SEQ_07 = PWRGD_PVPP_CD,
	CHECK_POWER_SEQ_08 = PWRGD_PVDDQ_AB_BIC,
	CHECK_POWER_SEQ_09 = PWRGD_PVDDQ_CD_BIC,
	CHECK_POWER_SEQ_10 = PWRGD_PVTT_AB,
	CHECK_POWER_SEQ_11 = PWRGD_PVTT_CD,
};

void set_MB_DC_status(uint8_t gpio_num);
void control_power_sequence();
void init_power_thread(uint8_t power_status);
void abort_power_thread();
void execute_power_on_sequence();
void execute_power_off_sequence();
int power_on_handler(uint8_t control_stage);
int power_off_handler(uint8_t control_stage);
int enable_power(uint8_t control_stage);
int disable_power(uint8_t control_stage);
int check_power_enable(uint8_t control_stage);
int check_power_disable(uint8_t control_stage);
int check_power_status(uint8_t power_status, uint8_t power_seq);

#endif
