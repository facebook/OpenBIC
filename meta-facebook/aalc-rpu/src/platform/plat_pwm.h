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

#define PWM_PORT0 0

#define REDUNDANT_STEP1_RETRY 6 /*Current interval in zone_table is 5s. STEP0-> STEP1 spent 5*6 s*/
#define REDUNDANT_STEP2A_RETRY                                                                     \
	12 /*Current interval in zone_table is 5s. STEP1-> STEP2A spent 5*12 s*/
#define REDUNDANT_STEP2B_RETRY                                                                     \
	12 /*Current interval in zone_table is 5s. STEP2A-> STEP2B spent 5*12 s*/

enum PWM_DEVICE_E {
	PWM_DEVICE_E_FB_FAN_1 = 0,
	PWM_DEVICE_E_FB_FAN_2,
	PWM_DEVICE_E_FB_FAN_3,
	PWM_DEVICE_E_FB_FAN_4,
	PWM_DEVICE_E_FB_FAN_5,
	PWM_DEVICE_E_FB_FAN_6,
	PWM_DEVICE_E_FB_FAN_7,
	PWM_DEVICE_E_FB_FAN_8,
	PWM_DEVICE_E_FB_FAN_9,
	PWM_DEVICE_E_FB_FAN_10,
	PWM_DEVICE_E_FB_FAN_11,
	PWM_DEVICE_E_FB_FAN_12,
	PWM_DEVICE_E_FB_FAN_13,
	PWM_DEVICE_E_FB_FAN_14,
	PWM_DEVICE_E_PB_PUMB_1,
	PWM_DEVICE_E_PB_PUMB_2,
	PWM_DEVICE_E_PB_PUMB_3,
	PWM_DEVICE_E_PB_PUMB_FAN_1,
	PWM_DEVICE_E_PB_PUMB_FAN_2,
	PWM_DEVICE_E_PB_PUMB_FAN_3,
	PWM_DEVICE_E_BB_FAN,
	PWM_DEVICE_E_MAX,
};

enum PWM_GROUP_E {
	PWM_GROUP_E_HEX_FAN = 0,
	PWM_GROUP_E_PUMP,
	PWM_GROUP_E_RPU_FAN,
	PWM_GROUP_E_MAX,
};

enum MANUAL_PWM_E {
	// group pwm dev
	MANUAL_PWM_E_HEX_FAN = 0,
	MANUAL_PWM_E_PUMP,
	MANUAL_PWM_E_RPU_FAN,
	// single pwm dev
	MANUAL_PWM_E_PUMP_1,
	MANUAL_PWM_E_PUMP_2,
	MANUAL_PWM_E_PUMP_3,
	MANUAL_PWM_E_PUMP_FAN_1,
	MANUAL_PWM_E_PUMP_FAN_2,
	MANUAL_PWM_E_PUMP_FAN_3,
	MANUAL_PWM_E_RPU_PCB_FAN,
	MANUAL_PWM_E_MAX,
};

enum REDUNDANCY_TRANSFORM_E {
	REDUNDANCY_TRANSFORM_DISABLE = 0,
	REDUNDANCY_TRANSFORM_STEP_1,
	REDUNDANCY_TRANSFORM_STEP_2A,
	REDUNDANCY_TRANSFORM_STEP_2B,
};

void init_pwm_dev(void);
int ast_pwm_set(int duty);
uint8_t plat_pwm_ctrl(enum PWM_DEVICE_E dev, uint8_t duty);
void abnormal_pump_redundant_transform();
uint8_t ctl_pwm_pump(uint8_t pump1_duty, uint8_t pump2_duty, uint8_t pump3_duty);
void reset_redundant_transform_status();
uint8_t ctl_all_pwm_dev(uint8_t duty);
uint8_t set_pwm_group(uint8_t group, uint8_t duty);
uint8_t get_pwm_group_cache(uint8_t group);
void set_pwm_group_cache(uint8_t group, uint8_t duty);
uint8_t get_pwm_cache(uint8_t idx);
uint8_t manual_pwm_idx_to_pwm_idx(uint8_t idx);
uint8_t get_manual_pwm_flag(uint8_t idx);
void set_manual_pwm_flag(uint8_t idx, uint8_t flag);
uint8_t get_manual_pwm_cache(uint8_t idx);
void set_manual_pwm_cache(uint8_t idx, uint8_t duty);
void set_manual_pwm_cache_to_default(void);
void set_manual_pwm_cache_to_zero(void);
uint8_t nct7363_wdt_all_disable();
uint8_t nct7363_wdt_all_enable();
