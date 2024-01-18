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
#define nct7363_REG_DID   0xFD
#define nct7363_ID_MASK	0xFFFFFF
#define nct7363_ID		(0x491988 & nct7363_ID_MASK)	/* Chip ID */

#define nct7363_REG_I2C_ADDR    0x46
#define nct7363_REG_GLOBAL_CONTROL  0x00

#define GPIO_00_to_03_Pin__Configuration_REG 0x20
#define GPIO_04_to_07_Pin__Configuration_REG 0x21
#define GPIO_10_to_13_Pin__Configuration_REG 0x22
#define GPIO_14_to_17_Pin__Configuration_REG 0x23
#define nct7363_REG_PWM_CTRL_OUTPUT_0_to_7 0x38	
#define nct7363_REG_PWM_CTRL_OUTPUT_8_to_15 0x39	
#define nct7363_REG_FANIN_CTRL1 0x41
#define nct7363_REG_FANIN_CTRL2 0x42

#define nct7363_REG_FAN_COUNT_VALUE_HIGH_BYTE_BASE_OFFSET 0x48 
#define nct7363_REG_FAN_COUNT_VALUE_LOW_BYTE_BASE_OFFSET 0x49 



#define nct7363_FAN_LSB_MASK	0x1F

#define FANIN_Monitoring_Global_Controller_Register 0x40



#define nct7363_REG_PWM_BASE_OFFSET 0x90

#define nct7363_REG_FSCPxDIV(index) (0x91 + (index)*2 )//Fan Speed Control Port x Divisor Register for writting speed to fan in unit:Hz (FSCPxDIV)

enum nct7363_sensor_offset {
	NCT7363_FAN_SPEED,
	NCT7363_FAN_CTRL_SET_DUTY,
};

#endif




