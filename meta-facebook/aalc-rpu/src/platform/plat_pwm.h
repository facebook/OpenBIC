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

void init_pwm_dev(void);
int ast_pwm_set(int duty);
uint8_t plat_pwm_ctrl(enum PWM_DEVICE_E dev, uint8_t duty);
uint8_t ctl_all_pwm_dev(uint8_t duty);
uint8_t set_pwm_group(uint8_t group, uint8_t duty);
uint8_t get_pwm_group_cache(uint8_t group);
uint8_t get_pwm_cache(uint8_t idx);
uint8_t nct7363_wdt_all_disable();
uint8_t nct7363_wdt_all_enable();
