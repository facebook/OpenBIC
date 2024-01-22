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

#include <stdio.h>
#include "sensor.h"
#include "hal_i2c.h"
#include "libutil.h"
#include <logging/log.h>
#include <nct7363.h>
struct nct7363_data {
	uint16_t has_fan;	/* Enable fan 0-15 */
	uint16_t fan_speed[16];		/* Register value combine */

	uint16_t has_pwm;	/* Enable pwm 0-15 */
	uint16_t pwm[16];		/* Register value combine */

	char valid;
};
uint8_t combine_gpio_setting_data(uint8_t bit_7_6, uint8_t bit_5_4, uint8_t bit_3_2, uint8_t bit_1_0){
	uint8_t data = 0;
	data |= bit_7_6 << 6;
	data |= bit_5_4 << 4;
	data |= bit_3_2 << 2;
	data |= bit_1_0 << 0;
	return data;
}

uint8_t nct363_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	uint8_t retry = 5;
	uint8_t FAN_COUNT_VALUE_HIGHT_BYTE = 0, FAN_COUNT_VALUE_LOW_BYTE = 0;
	I2C_MSG msg = { 0 };
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	unsigned long rpm = 0;


	//nct7363_init_arg *init_arg = (nct7363_init_arg *)cfg->init_args;
	//uint8_t port = init_arg->port;
	
	
	uint8_t offset = cfg->offset;
	uint8_t duty = cfg->init_args.duty;
	uint8_t port = cfg->init_args.port;
	uint8_t fan_roles = cfg->init_args.fan_roles ;
	struct nct7363_data *data;
	switch (offset) {
	case NCT7363_FAN_SPEED:
		uint8_t fan_count_high_byte_offset = nct7363_REG_FAN_COUNT_VALUE_HIGH_BYTE_BASE_OFFSET + port*2;
		uint8_t fan_count_low_byte_offset = nct7363_REG_FAN_COUNT_VALUE_LOW_BYTE_BASE_OFFSET + port*2;
		msg.rx_len = 1 ;
		msg.tx_len = 1;
		//todo: 
		//read FAN_COUNT_VALUE_HIGH_BYTE
		//read FAN_COUNT_VALUE_LOW_BYTE
		msg.data[0] = fan_count_high_byte_offset;
		if (i2c_master_read(&msg, retry)) {
			return SENSOR_FAIL_TO_ACCESS;
		}
		uint8_t fan_count_high_byte = msg.data[0];
		
		msg.data[0] = fan_count_low_byte_offset;
		if (i2c_master_read(&msg, retry)) {
			return SENSOR_FAIL_TO_ACCESS;
		}
		uint8_t fan_count_low_byte = msg.data[0];
		uint16_t fan_count_value = (fan_count_high_byte << 5) | (fan_count_low_byte & nct7363_FAN_LSB_MASK);
		
		//count result
		rpm = 1350000/(fan_count_value*(fan_roles/4));
		break;
	case NCT7363_FAN_CTRL_SET_DUTY:
		uint8_t duty_offset = nct7363_REG_PWM_BASE_OFFSET + port*2;
		msg.tx_len = 2;
		msg.data[0]=  duty_offset;
		msg.data[1]=  duty;

		if (i2c_master_write(&msg, retry)) {
			LOG_ERR("set NCT7363_FAN_CTRL_SET_DUTY fail, status: %d", NCT7363_FAN_CTRL_SET_DUTY_status);
			return 	0xFF; ////???????? set duty fail 

		}
		break;
	default:
		LOG_ERR("Unknown register offset(%d)", offset);
		return SENSOR_UNSPECIFIED_ERROR;
	}


	//uint8_t test_value = 0x04; //only for test, remove it later
	//msg.data[0] = test_value;

	sensor_val *sval = (sensor_val *)reading;
	sval->integer = (int32_t)rpm;
	sval->fraction = (int32_t)(rpm * 1000) % 1000;
	return SENSOR_READ_SUCCESS;
}

uint8_t nct363_init(sensor_cfg *cfg)
{
	// please add in plateform_sensortable.c
	/*	{ FANBD_1_Fan_speed, sensor_dev_nct7363test, I2C_BUS1, NONE,NCT7363_FAN_SPEED, stby_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &nct7363_init_args[0] },
	*/
	// please add in plat_hook.c
	/*
	nct7363_init_arg nct7363_init_args[] = {
		// mode 0 
		//GPIO setting: Reserved=11, FANINx=10, PWMx=01, GPIOXX=00
		[0] = { 
			//16 ports
			.is_init = false,
			.init__pin_config{
				.GPIO_00_to_03_Pin_Function_Configuration = 0b00000000,
				.GPIO_04_to_07_Pin_Function_Configuration = 0b00000000,
				.GPIO_10_to_13_Pin_Function_Configuration = 0b00000000,
				.GPIO_14_to_17_Pin_Function_Configuration = 0b00000000,
				.PWM_0_to_7_Enable = 0b00000000,
				.PWM_8_to_15_Enable = 0b00000000,
				.FANIN_0_to_7_Monitoring_Enable = 0b00000000,
				.FANIN_8_to_15_Monitoring_Enable = 0b00000000,
			},
			.init_16_pin_config{
				.GPIO00 = 0b00,
				.GPIO01 = 0b00,
				.GPIO02 = 0b00,
				.GPIO03 = 0b00,
				.GPIO04 = 0b00,
				.GPIO05 = 0b00,
				.GPIO06 = 0b00,
				.GPIO07 = 0b00,
				.GPIO10 = 0b00,
				.GPIO11 = 0b00,
				.GPIO12 = 0b00,
				.GPIO13 = 0b00,
				.GPIO14 = 0b00,
				.GPIO15 = 0b00,
				.GPIO16 = 0b00,
				.GPIO17 = 0b00,
				.PWM_0_to_7_Enable = 0b00000000,
				.PWM_8_to_15_Enable = 0b00000000,
				.FANIN_0_to_7_Monitoring_Enable = 0b00000000,
				.FANIN_8_to_15_Monitoring_Enable = 0b00000000,
			},
			//which port to monitor or set
			.port = 0,
			.fan_poles = 0,
			.duty = 0,
			
	},
	*/
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}
	I2C_MSG init_msg = { 0 };
	uint8_t retry = 5;
	init_msg.bus = cfg->port;
	init_msg.target_addr = cfg->target_addr;
	nct7363_init_arg *nct7363_init_arg_data = (nct7363_init_arg *)cfg->init_args;
	//// init_pin_config ////
	uint8_t GPIO_00_to_03_Pin__Configuration_REG_msg = cfg->init_args.init_pin_config.GPIO_00_to_03_Pin_Function_Configuration;
	uint8_t GPIO_04_to_07_Pin__Configuration_REG_msg = cfg->init_args.init_pin_config.GPIO_04_to_07_Pin_Function_Configuration;
	uint8_t GPIO_10_to_13_Pin__Configuration_REG_msg = cfg->init_args.init_pin_config.GPIO_10_to_13_Pin_Function_Configuration;
	uint8_t GPIO_14_to_17_Pin__Configuration_REG_msg = cfg->init_args.init_pin_config.GPIO_14_to_17_Pin_Function_Configuration;
	//// init_16_pin_config ////
	/*
	uint8_t GPIO_00_to_03_Pin__Configuration_REG_msg = combine_gpio_setting_data(cfg->init_args.init_16_pin_config.GPIO03,cfg->init_args.init_16_pin_config.GPIO02,cfg->init_args.init_16_pin_config.GPIO01,cfg->init_args.init_16_pin_config.GPIO00);
	uint8_t GPIO_04_to_07_Pin__Configuration_REG_msg = combine_gpio_setting_data(cfg->init_args.init_16_pin_config.GPIO07,cfg->init_args.init_16_pin_config.GPIO06,cfg->init_args.init_16_pin_config.GPIO05,cfg->init_args.init_16_pin_config.GPIO04);
	uint8_t GPIO_10_to_13_Pin__Configuration_REG_msg = combine_gpio_setting_data(cfg->init_args.init_16_pin_config.GPIO13,cfg->init_args.init_16_pin_config.GPIO12,cfg->init_args.init_16_pin_config.GPIO11,cfg->init_args.init_16_pin_config.GPIO10);
	uint8_t GPIO_14_to_17_Pin__Configuration_REG_msg = combine_gpio_setting_data(cfg->init_args.init_16_pin_config.GPIO17,cfg->init_args.init_16_pin_config.GPIO16,cfg->init_args.init_16_pin_config.GPIO15,cfg->init_args.init_16_pin_config.GPIO14);
	*/
	uint8_t nct7363_REG_PWM_CTRL_OUTPUT_0_to_7_msg = cfg->init_args.init_pin_config.PWM_0_to_7_Enable;
	uint8_t nct7363_REG_PWM_CTRL_OUTPUT_8_to_15_msg = cfg->init_args.init_pin_config.PWM_8_to_15_Enable;
	uint8_t nct7363_REG_FANIN_CTRL1_msg = cfg->init_args.init_pin_config.FANIN_0_to_7_Monitoring_Enable;
	uint8_t nct7363_REG_FANIN_CTRL2_msg = cfg->init_args.init_pin_config.FANIN_8_to_15_Monitoring_Enable;

	///////////   set GPIO/PWM/FAN configuration   ///////////////////////
	init_msg.tx_len = 2;
	init_msg.data[0]=  GPIO_00_to_03_Pin__Configuration_REG;
	init_msg.data[1]=  GPIO_00_to_03_Pin__Configuration_REG_msg;
	if (i2c_master_write(&init_msg, retry)) {
		LOG_ERR("set nct363 set GPIO_00_to_03_Pin__Configuration_REG fail");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}
	init_msg.data[0]=  GPIO_04_to_07_Pin__Configuration_REG;
	init_msg.data[1]=  GPIO_04_to_07_Pin__Configuration_REG_msg;
	if (i2c_master_write(&init_msg, retry)) {
		LOG_ERR("set nct363 set GPIO_04_to_07_Pin__Configuration_REG fail");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}
	init_msg.data[0]=  GPIO_10_to_13_Pin__Configuration_REG;
	init_msg.data[1]=  GPIO_10_to_13_Pin__Configuration_REG_msg;
	if (i2c_master_write(&init_msg, retry)) {
		LOG_ERR("set nct363 set GPIO_10_to_13_Pin__Configuration_REG fail");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}
	init_msg.data[0]=  GPIO_14_to_17_Pin__Configuration_REG;
	init_msg.data[1]=  GPIO_14_to_17_Pin__Configuration_REG_msg;
	if (i2c_master_write(&init_msg, retry)) {
		LOG_ERR("set nct363 set GPIO_14_to_17_Pin__Configuration_REG fail");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	///////////   set PWM  output   ///////////////////////
	init_msg.data[0]=  nct7363_REG_PWM_CTRL_OUTPUT_0_to_7;
	init_msg.data[1]=  nct7363_REG_PWM_CTRL_OUTPUT_0_to_7_msg;
	if (i2c_master_write(&init_msg, retry)) {
		LOG_ERR("set nct363 set nct7363_REG_PWM_CTRL_OUTPUT_0_to_7 fail");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}
	init_msg.data[0]=  nct7363_REG_PWM_CTRL_OUTPUT_8_to_15;
	init_msg.data[1]=  nct7363_REG_PWM_CTRL_OUTPUT_8_to_15_msg;
	if (i2c_master_write(&init_msg, retry)) {
		LOG_ERR("set nct363 set nct7363_REG_PWM_CTRL_OUTPUT_8_to_15 fail");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	///////////   set FANIN control    ///////////////////////
	init_msg.data[0]=  nct7363_REG_FANIN_CTRL1;
	init_msg.data[1]=  nct7363_REG_FANIN_CTRL1_msg;
	if (i2c_master_write(&init_msg, retry)) {
		LOG_ERR("set nct7363_REG_FANIN_CTRL1 fail");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}
	init_msg.data[0]=  nct7363_REG_FANIN_CTRL2;
	init_msg.data[1]=  nct7363_REG_FANIN_CTRL2_msg;
	if (i2c_master_write(&init_msg, retry)) {
		LOG_ERR("set nct7363_REG_FANIN_CTRL2 fail");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	////// set init fan duty ////
	  for (int i = 0; i < 15; i++)
	{
		uint8_t duty_offset = nct7363_REG_PWM_BASE_OFFSET + i*2;
		init_msg.tx_len = 2;
		init_msg.data[0]=  duty_offset;
		init_msg.data[1]=  0x66;  // 40% duty for init

		if (i2c_master_write(&init_msg, retry)) {
			LOG_ERR("set NCT7363_FAN_CTRL_SET_DUTY fail, status: %d", NCT7363_FAN_CTRL_SET_DUTY_status);
			return 	SENSOR_INIT_UNSPECIFIED_ERROR; 
		}
	}
	

	cfg->read = nct363_read;

	init_arg->is_init = true;
	return SENSOR_INIT_SUCCESS;
}