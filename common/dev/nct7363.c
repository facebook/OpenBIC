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
#include <string.h>
LOG_MODULE_REGISTER(dev_nct7363);
uint8_t combine_gpio_setting_data(uint8_t bit_7_6, uint8_t bit_5_4, uint8_t bit_3_2,
				  uint8_t bit_1_0)
{
	uint8_t data = 0;
	data |= bit_7_6 << 6;
	data |= bit_5_4 << 4;
	data |= bit_3_2 << 2;
	data |= bit_1_0 << 0;
	return data;
}
uint8_t nct7363_set_duty(uint8_t bus, uint8_t address, uint8_t port, uint8_t duty)
{
	I2C_MSG msg = { 0 };
	uint8_t retry = 5;
	uint8_t port_offset = port;
	uint8_t duty_offset = nct7363_REG_PWM_BASE_OFFSET + port_offset * 2;
	msg.bus = bus;
	msg.target_addr = address;
	msg.tx_len = 2;
	msg.data[0] = duty_offset;
	msg.data[1] = duty;

	if (i2c_master_write(&msg, retry)) {
		LOG_ERR("set NCT7363_FAN_CTRL_SET_DUTY fail");
		return 0xFF; // set duty fail
	}
	return 0;
}
uint8_t nct7363_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);
	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}
	uint8_t retry = 5;
	I2C_MSG msg = { 0 };
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	uint32_t rpm = 0;
	uint8_t offset = cfg->offset;
	uint8_t port_offset = cfg->arg0;
	uint8_t fan_roles = cfg->arg1;
	uint8_t fan_count_high_byte_offset =
			nct7363_REG_FAN_COUNT_VALUE_HIGH_BYTE_BASE_OFFSET + port_offset * 2;
	uint8_t fan_count_low_byte_offset =
			nct7363_REG_FAN_COUNT_VALUE_LOW_BYTE_BASE_OFFSET + port_offset * 2;
	switch (offset) {
	case NCT7363_FAN_SPEED_OFFSET:
		
		msg.rx_len = 1;
		msg.tx_len = 1;
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
		uint16_t fan_count_value =
			(fan_count_high_byte << 5) | (fan_count_low_byte & nct7363_FAN_LSB_MASK);
		/* count result */
		rpm = 1350000 / (fan_count_value * (fan_roles / 4));
		break;
	default:
		LOG_ERR("Unknown register offset(%d)", offset);
		return SENSOR_UNSPECIFIED_ERROR;
	}
	sensor_val *sval = (sensor_val *)reading;
	sval->integer = (int32_t)rpm;
	sval->fraction = (int32_t)(rpm * 1000) % 1000;
	return SENSOR_READ_SUCCESS;
}

uint8_t nct7363_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}
	nct7363_init_arg *nct7363_init_arg_data = (nct7363_init_arg *)cfg->init_args;
	I2C_MSG init_msg = { 0 };
	uint8_t retry = 5;
	init_msg.bus = cfg->port;
	init_msg.target_addr = cfg->target_addr;
	
	/* init_pin_config */
	uint8_t gpio_00_to_03_pin_configuration_reg_msg =
		nct7363_init_arg_data->init_pin_config.GPIO_00_to_03_Pin_Function_Configuration;
	uint8_t gpio_04_to_07_pin_configuration_reg_msg =
		nct7363_init_arg_data->init_pin_config.GPIO_04_to_07_Pin_Function_Configuration;
	uint8_t gpio_10_to_13_pin_configuration_reg_msg =
		nct7363_init_arg_data->init_pin_config.GPIO_10_to_13_Pin_Function_Configuration;
	uint8_t gpio_14_to_17_pin_configuration_reg_msg =
		nct7363_init_arg_data->init_pin_config.GPIO_14_to_17_Pin_Function_Configuration;
	/* init_16_pin_config */
	uint8_t nct7363_reg_pwm_ctrl_output_0_to_7_msg =
		nct7363_init_arg_data->init_pin_config.PWM_0_to_7_Enable;
	uint8_t nct7363_reg_pwml_ctr_outpu_8_to_15_msg =
		nct7363_init_arg_data->init_pin_config.PWM_8_to_15_Enable;
	uint8_t nct7363_reg_fanin_ctrl1_msg =
		nct7363_init_arg_data->init_pin_config.FANIN_0_to_7_Monitoring_Enable;
	uint8_t nct7363_reg_fanin_ctrl2_msg =
		nct7363_init_arg_data->init_pin_config.FANIN_8_to_15_Monitoring_Enable;
	/* init_gpio_input/output config */
	uint8_t gpio0x_input_output_configuration_msg =
		nct7363_init_arg_data->init_pin_config.GPIO0x_Input_Output_Configuration;
	uint8_t gpio1x_input_output_configuration_msg =
		nct7363_init_arg_data->init_pin_config.GPIO1x_Input_Output_Configuration;
	/* set GPIO/PWM/FAN configuration */
	init_msg.tx_len = 2;
	init_msg.data[0] = GPIO_00_to_03_Pin_Configuration_REG;
	init_msg.data[1] = gpio_00_to_03_pin_configuration_reg_msg;
	if (i2c_master_write(&init_msg, retry)) {
		LOG_ERR("set nct363 set GPIO_00_to_03_Pin_Configuration_REG fail");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}
	init_msg.data[0] = GPIO_04_to_07_Pin_Configuration_REG;
	init_msg.data[1] = gpio_04_to_07_pin_configuration_reg_msg;
	if (i2c_master_write(&init_msg, retry)) {
		LOG_ERR("set nct363 set GPIO_04_to_07_Pin_Configuration_REG fail");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}
	init_msg.data[0] = GPIO_10_to_13_Pin_Configuration_REG;
	init_msg.data[1] = gpio_10_to_13_pin_configuration_reg_msg;
	if (i2c_master_write(&init_msg, retry)) {
		LOG_ERR("set nct363 set GPIO_10_to_13_Pin_Configuration_REG fail");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}
	init_msg.data[0] = GPIO_14_to_17_Pin_Configuration_REG;
	init_msg.data[1] = gpio_14_to_17_pin_configuration_reg_msg;
	if (i2c_master_write(&init_msg, retry)) {
		LOG_ERR("set nct363 set GPIO_14_to_17_Pin_Configuration_REG fail");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}
	/* set GPIO input/output */
	init_msg.data[0] = GPIO0x_Input_Output_Configuration_REG;
	init_msg.data[1] = gpio0x_input_output_configuration_msg;
	if (i2c_master_write(&init_msg, retry)) {
		LOG_ERR("set nct363 set GPIO0x_Input_Output_Configuration fail");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}
	init_msg.data[0] = GPIO1x_Input_Output_Configuration_REG;
	init_msg.data[1] = gpio1x_input_output_configuration_msg;
	if (i2c_master_write(&init_msg, retry)) {
		LOG_ERR("set nct363 set GPIO1x_Input_Output_Configuration fail");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}
	/* set PWM output */
	init_msg.data[0] = nct7363_REG_PWM_CTRL_OUTPUT_0_to_7;
	init_msg.data[1] = nct7363_reg_pwm_ctrl_output_0_to_7_msg;
	if (i2c_master_write(&init_msg, retry)) {
		LOG_ERR("set nct363 set nct7363_REG_PWM_CTRL_OUTPUT_0_to_7 fail");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}
	init_msg.data[0] = nct7363_REG_PWM_CTRL_OUTPUT_8_to_15;
	init_msg.data[1] = nct7363_reg_pwml_ctr_outpu_8_to_15_msg;
	if (i2c_master_write(&init_msg, retry)) {
		LOG_ERR("set nct363 set nct7363_REG_PWM_CTRL_OUTPUT_8_to_15 fail");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}
	/*  set FANIN control */
	init_msg.data[0] = nct7363_REG_FANIN_CTRL1;
	init_msg.data[1] = nct7363_reg_fanin_ctrl1_msg;
	if (i2c_master_write(&init_msg, retry)) {
		LOG_ERR("set nct7363_REG_FANIN_CTRL1 fail");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}
	init_msg.data[0] = nct7363_REG_FANIN_CTRL2;
	init_msg.data[1] = nct7363_reg_fanin_ctrl2_msg;
	if (i2c_master_write(&init_msg, retry)) {
		LOG_ERR("set nct7363_REG_FANIN_CTRL2 fail");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}
	/*  set init fan duty */
	for (int i = 0; i < 15; i++) {
		uint8_t duty_offset = nct7363_REG_PWM_BASE_OFFSET + i * 2;
		init_msg.tx_len = 2;
		init_msg.data[0] = duty_offset;
		init_msg.data[1] = 0x66; // 40% duty for init

		if (i2c_master_write(&init_msg, retry)) {
			LOG_ERR("set NCT7363_FAN_CTRL_SET_DUTY fail");
			return SENSOR_INIT_UNSPECIFIED_ERROR;
		}
	}
	nct7363_init_arg_data->is_init = true;
	cfg->arg1 = nct7363_init_arg_data->fan_poles;
	cfg->read = nct7363_read;
	return SENSOR_INIT_SUCCESS;
}

