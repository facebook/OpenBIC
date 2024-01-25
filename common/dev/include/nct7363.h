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

#define GPIO0x_Input_Output_Configuration_REG 0x03
#define GPIO1x_Input_Output_Configuration_REG 0x13

#define GPIO_00_to_03_Pin_Configuration_REG 0x20
#define GPIO_04_to_07_Pin_Configuration_REG 0x21
#define GPIO_10_to_13_Pin_Configuration_REG 0x22
#define GPIO_14_to_17_Pin_Configuration_REG 0x23
#define NCT7363_REG_PWM_CTRL_OUTPUT_0_to_7 0x38
#define NCT7363_REG_PWM_CTRL_OUTPUT_8_to_15 0x39
#define NCT7363_REG_FANIN_CTRL1 0x41
#define NCT7363_REG_FANIN_CTRL2 0x42

#define NCT7363_REG_FAN_COUNT_VALUE_HIGH_BYTE_BASE_OFFSET 0x48
#define NCT7363_REG_FAN_COUNT_VALUE_LOW_BYTE_BASE_OFFSET 0x49

#define NCT7363_FAN_LSB_MASK 0x1F
#define FANIN_Monitoring_Global_Controller_Register 0x40
#define NCT7363_REG_PWM_BASE_OFFSET 0x90

#define Speed_Control_Portx_Configuration_Register_BASE_OFFSET 0xB0
/* nct7363 port*/
#define NCT7363_1_FAN 0x00
#define NCT7363_2_FAN 0x01
#define NCT7363_3_FAN 0x02
#define NCT7363_4_FAN 0x03
#define NCT7363_5_FAN 0x04
#define NCT7363_6_FAN 0x05
#define NCT7363_7_FAN 0x06
#define NCT7363_8_FAN 0x07
#define NCT7363_9_FAN 0x08
#define NCT7363_10_FAN 0x09
#define NCT7363_11_FAN 0x0A
#define NCT7363_12_FAN 0x0B
#define NCT7363_13_FAN 0x0C
#define NCT7363_14_FAN 0x0D
#define NCT7363_15_FAN 0x0E
#define NCT7363_16_FAN 0x0F
struct nct7363_data {
	uint16_t has_fan; /* Enable fan 0-15 */
	uint16_t fan_speed[16]; /* Register value combine */

	uint16_t has_pwm; /* Enable pwm 0-15 */
	uint16_t pwm[16]; /* Register value combine */

	char valid;
};
enum nct7363_sensor_offset {
	NCT7363_FAN_SPEED_OFFSET,
};

#endif
