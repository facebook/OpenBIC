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
#ifndef _PLAT_LED_h
#define _PLAT_LED_h

#define LED_IO_EXPANDER_BUS I2C_BUS5
#define LED_IO_EXPANDER_ADDR (0xEE >> 1)
#define LED_IO_EXPANDER_OUTPUT_PORT0_REG 0x02
#define LED_IO_EXPANDER_OUTPUT_PORT1_REG 0x03
#define LED_IO_EXPANDER_CONFIG_PORT0_REG 0x06
#define LED_IO_EXPANDER_CONFIG_PORT1_REG 0x07

#define LED_IO_EXPANDER_FAULT_BIT 6
#define LED_IO_EXPANDER_PWR_BIT 7

#define E1S_LED_IO_EXPANDER_BUS I2C_BUS6
#define E1S_LED_IO_EXPANDER_ADDR (0xEC >> 1)

enum sys_led_ctrl_status {
	LED_CTRL_OFF = 0,
	LED_CTRL_ON = 1,
	LED_CTRL_MAX,
	LED_STATUS_ERROR = 0xFF,
};

enum sys_led_ctrl_source {
	LED_CTRL_SRC_BIC = 0,
	LED_CTRL_SRC_BMC = 1,
	LED_CTRL_SRC_MAX,
};

enum sys_led_type {
	SYS_POWER_LED = 0,
	SYS_FAULT_LED = 1,
	SYS_LED_TYPE_MAX,
};

bool pwr_led_control(uint8_t src, uint8_t status);
bool fault_led_control(uint8_t src, uint8_t status);
void light_fault_led_check();
void pwr_led_check();
uint8_t get_sys_led_status(uint8_t type);
bool e1s_led_control(uint8_t effector_id, uint8_t status);
uint8_t get_ssd_led_status(uint8_t effector_id);
void sys_led_init_and_check();
#endif /* _PLAT_LED_h */
