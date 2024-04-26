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
#ifndef NCT7363_H
#define NCT7363_H
#define NCT7363_REG_DID 0xFD
#define NCT7363_ID_MASK 0xFFFFFF
#define NCT7363_ID (0x491988 & NCT7363_ID_MASK) /* Chip ID */

#define NCT7363_REG_I2C_ADDR 0x46
#define NCT7363_REG_GLOBAL_CONTROL 0x00

#define GPIO0X_IO_CONF_REG 0x03
#define GPIO1X_IO_CONF_REG 0x13

#define GPIO_00_TO_03_PIN_CONFIGURATION_REG 0x20
#define GPIO_04_TO_07_PIN_CONFIGURATION_REG 0x21
#define GPIO_10_TO_13_PIN_CONFIGURATION_REG 0x22
#define GPIO_14_TO_17_PIN_CONFIGURATION_REG 0x23

#define NCT7363_PWM_CTRL_OUTPUT_0_TO_7_REG 0x38
#define NCT7363_PWM_CTRL_OUTPUT_8_TO_15_REG 0x39

#define NCT7363_FANIN_ENABLE_0_TO_7_REG 0x41
#define NCT7363_FANIN_ENABLE_8_TO_15_REG 0x42

#define NCT7363_REG_FAN_COUNT_VALUE_HIGH_BYTE_BASE_OFFSET 0x48
#define NCT7363_REG_FAN_COUNT_VALUE_LOW_BYTE_BASE_OFFSET 0x49

#define FANIN_MONOTORING_GLOBAL_CONTROLLER_REG 0x40
#define NCT7363_REG_PWM_BASE_OFFSET 0x90

#define SPEED_CONTROL_PORT_CONF_REG_BASE_OFFSET 0xB0
#define SPEED_CONTROL_PORT_DIVISOR_BASE_OFFSET 0x91

/* nct7363 port*/
enum nct7363_port {
	NCT7363_1_PORT,
	NCT7363_2_PORT,
	NCT7363_3_PORT,
	NCT7363_4_PORT,
	NCT7363_5_PORT,
	NCT7363_6_PORT,
	NCT7363_7_PORT,
	NCT7363_8_PORT,
	NCT7363_10_PORT,
	NCT7363_11_PORT,
	NCT7363_12_PORT,
	NCT7363_13_PORT,
	NCT7363_14_PORT,
	NCT7363_15_PORT,
	NCT7363_16_PORT,
	NCT7363_17_PORT,
};

#define FAN_STATUS_0_TO_7_REG 0x34
#define FAN_STATUS_8_TO_15_REG 0x35

#define NCT7363_FAN_COUNT_THRESHOLD_REG_HIGH_BYTE_BASE_OFFSET 0x6C
#define NCT7363_FAN_COUNT_THRESHOLD_REG_LOW_BYTE_BASE_OFFSET 0x6D

#define NCT7363_GPIO0x_OUTPUT_PORT_REG_OFFSET 0x01
#define NCT7363_GPIO1x_OUTPUT_PORT_REG_OFFSET 0x10

#define NCT7363_WDT_REG_OFFSET 0x2A

struct nct7363_data {
	uint16_t has_fan; /* Enable fan 0-15 */
	uint16_t fan_speed[16]; /* Register value combine */

	uint16_t has_pwm; /* Enable pwm 0-15 */
	uint16_t pwm[16]; /* Register value combine */

	char valid;
};

enum nct7363_sensor_offset {
	NCT7363_FAN_SPEED_OFFSET,
	NCT7363_FAN_STATUS_OFFSET,
	NCT7363_GPIO_READ_OFFSET,
};

enum nct7363_pin_type {
	NCT7363_PIN_TPYE_GPIO,
	NCT7363_PIN_TPYE_PWM,
	NCT7363_PIN_TPYE_FANIN,
	NCT7363_PIN_TPYE_RESERVED,
	NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT, // user defined
	NCT7363_PIN_TPYE_ERROR
};

enum nct7363_gpio_type { NCT7363_GPIO_OUTPUT, NCT7363_GPIO_INPUT };

enum nct7363_wdt_sec { WDT_15_SEC, WDT_3_75_SEC, WDT_7_5_SEC, WDT_30_SEC, WDT_DISABLE, WDT_ERROR };

#endif
