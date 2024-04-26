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
#include <math.h>
#include <sys/util.h>

LOG_MODULE_REGISTER(dev_nct7363);

#define NCT7363_PIN_NUMBER 16
#define NCT7363_PIN_FUNC_REG_SIZE 4
#define NCT7363_REG_SIZE 8
#define FREQUENCY_DIVEL_0_RANGE_MAX 62500
#define FREQUENCY_DIVEL_0_RANGE_MIN 488.28125
#define FREQUENCY_DIVEL_1_RANGE_MAX 244.14
#define FREQUENCY_DIVEL_1_RANGE_MIN 1.907
#define NCT7363_FAN_LSB_MASK BIT_MASK(5)

bool nct7363_set_threshold(sensor_cfg *cfg, uint16_t threshold)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);

	I2C_MSG msg = { 0 };
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	uint8_t retry = 5;
	uint8_t port_offset = cfg->arg0;
	uint8_t threshold_offset_high_byte = 0;
	uint8_t threshold_offset_low_byte = 0;
	uint8_t threshold_low_byte_value = threshold & NCT7363_FAN_LSB_MASK;
	uint8_t threshold_high_byte_value = threshold >> 8;

	if (port_offset > NCT7363_8_PORT) {
		// fanin8~15 = pwm/gpio0~7
		threshold_offset_high_byte = NCT7363_FAN_COUNT_THRESHOLD_REG_HIGH_BYTE_BASE_OFFSET +
					     (port_offset - 8) * 2;
		threshold_offset_low_byte = NCT7363_FAN_COUNT_THRESHOLD_REG_LOW_BYTE_BASE_OFFSET +
					    (port_offset - 8) * 2;
	} else {
		// fan0~7 = pwm/gpio8~15
		threshold_offset_high_byte = NCT7363_FAN_COUNT_THRESHOLD_REG_HIGH_BYTE_BASE_OFFSET +
					     (port_offset + 8) * 2;
		threshold_offset_low_byte = NCT7363_FAN_COUNT_THRESHOLD_REG_LOW_BYTE_BASE_OFFSET +
					    (port_offset + 8) * 2;
	}
	// write high byte value
	msg.tx_len = 2;
	msg.data[0] = threshold_offset_high_byte;
	msg.data[1] = threshold_high_byte_value;
	if (i2c_master_write(&msg, retry) != 0) {
		LOG_ERR("set NCT7363_threshold_high_byte_value fail");
		return false;
	}

	// write low byte value
	msg.tx_len = 2;
	msg.data[0] = threshold_offset_low_byte;
	msg.data[1] = threshold_low_byte_value;
	if (i2c_master_write(&msg, retry) != 0) {
		LOG_ERR("set NCT7363_threshold_low_byte_value fail");
		return false;
	}

	return true;
}

bool nct7363_set_duty(sensor_cfg *cfg, uint8_t duty)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);

	if(duty > 100 || duty < 0) {
		LOG_ERR("Set invalid duty: %d", duty);
		return false;
	}

	I2C_MSG msg = { 0 };
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	uint8_t retry = 5;
	uint8_t port_offset = cfg->arg0; // nct7363 pin
	uint8_t duty_offset = NCT7363_REG_PWM_BASE_OFFSET + port_offset * 2;
	float duty_in_255 = 0;
	// set duty
	duty_in_255 = 255 * duty / 100; // 0xFF
	msg.tx_len = 2;
	msg.data[0] = duty_offset;
	msg.data[1] = (uint8_t)duty_in_255;

	if (i2c_master_write(&msg, retry) != 0) {
		LOG_ERR("set NCT7363_FAN_CTRL_SET_DUTY fail");
		return false;
	}

	return true;
}
static bool nct7363_write(sensor_cfg *cfg, uint8_t offset, uint8_t val)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	I2C_MSG msg = { 0 };
	uint8_t retry = 5;
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 2;

	msg.data[0] = offset;
	msg.data[1] = val;

	if (i2c_master_write(&msg, retry) != 0) {
		LOG_ERR("nct7363 write offset 0x%02x, val 0x%02x fail", offset, val);
		return false;
	}

	return true;
}
static bool fan_frequency_convert(float frequency, uint8_t *output_freqency)
{
	int val_reg = 0;
	uint8_t step_value = 1;

	// divel = 0
	if (frequency >= FREQUENCY_DIVEL_0_RANGE_MIN && frequency <= FREQUENCY_DIVEL_0_RANGE_MAX) {
		val_reg = (int)(((16000000 / (frequency * (step_value + 1) * 128)) - 1) + 0.5);
		WRITE_BIT(val_reg, 7, 0);
		*output_freqency = (uint8_t)val_reg;
		return true;
	}
	// devel = 1
	else if (frequency > FREQUENCY_DIVEL_1_RANGE_MIN &&
		 frequency < FREQUENCY_DIVEL_1_RANGE_MAX) {
		val_reg =
			(int)(((16000000 / (frequency * (step_value + 1) * 128 * 256)) - 1) + 0.5);
		WRITE_BIT(val_reg, 7, 1);
		*output_freqency = (uint8_t)val_reg;
		return true;
	} else {
		LOG_ERR("Wrong when frequency convert, value: 0x%f", frequency);
		return false;
	}
}

bool nct7363_set_frequency(sensor_cfg *cfg, float frequency)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	I2C_MSG msg = { 0 };
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	uint8_t port_offset = cfg->arg0; // nct7363 pin
	uint8_t freq_offset = 0;
	uint8_t output_freq = 0;
	freq_offset = SPEED_CONTROL_PORT_DIVISOR_BASE_OFFSET + port_offset * 2;
	bool convert_result = fan_frequency_convert(frequency, &output_freq);

	if (!convert_result)
		return false;

	if (!nct7363_write(cfg, freq_offset, output_freq))
		LOG_ERR("Set frequency error");
		return false;
		
	return true;
}

static uint8_t nct7363_read(sensor_cfg *cfg, int *reading)
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
	float rpm = 0;
	int gpio_result = 0;
	uint8_t offset = cfg->offset;
	uint8_t port_offset = cfg->arg0;
	uint8_t fan_poles = cfg->arg1;
	uint8_t fan_count_high_byte_offset = 0;
	uint8_t fan_count_low_byte_offset = 0;
	if (port_offset > NCT7363_8_PORT) {
		// fanin8~15 = pwm/gpio0~7
		fan_count_high_byte_offset =
			NCT7363_REG_FAN_COUNT_VALUE_HIGH_BYTE_BASE_OFFSET + (port_offset - 8) * 2;
		fan_count_low_byte_offset =
			NCT7363_REG_FAN_COUNT_VALUE_LOW_BYTE_BASE_OFFSET + (port_offset - 8) * 2;
	} else {
		// fan0~7 = pwm/gpio8~15
		fan_count_high_byte_offset =
			NCT7363_REG_FAN_COUNT_VALUE_HIGH_BYTE_BASE_OFFSET + (port_offset + 8) * 2;
		fan_count_low_byte_offset =
			NCT7363_REG_FAN_COUNT_VALUE_LOW_BYTE_BASE_OFFSET + (port_offset + 8) * 2;
	}

	switch (offset) {
	case NCT7363_FAN_SPEED_OFFSET:
		msg.rx_len = 1;
		msg.tx_len = 1;
		msg.data[0] = fan_count_high_byte_offset;
		if (i2c_master_read(&msg, retry) != 0) {
			LOG_ERR("Read fan_count_high_byte_offset fail");
			return SENSOR_FAIL_TO_ACCESS;
		}

		uint8_t fan_count_high_byte = msg.data[0];

		msg.data[0] = fan_count_low_byte_offset;
		if (i2c_master_read(&msg, retry) != 0) {
			LOG_ERR("Read fan_count_low_byte_offset fail");
			return SENSOR_FAIL_TO_ACCESS;
		}

		uint8_t fan_count_low_byte = msg.data[0];
		uint16_t fan_count_value =
			(fan_count_high_byte << 5) | (fan_count_low_byte & NCT7363_FAN_LSB_MASK);
		/* count result */
		rpm = 1350000 / ((float)fan_count_value * ((float)fan_poles / 4)); // RPM
		/* return result */
		sensor_val *sval = (sensor_val *)reading;
		sval->integer = (int16_t)rpm;
		sval->fraction = 0;

		return SENSOR_READ_SUCCESS;
	case NCT7363_FAN_STATUS_OFFSET:
		msg.rx_len = 1;
		msg.tx_len = 1;
		int error_flag = 0;
		msg.data[0] = FAN_STATUS_0_TO_7_REG;
		if (i2c_master_read(&msg, retry) != 0) {
			LOG_ERR("Read FAN_STATUS_0_TO_7_REG fail");
			return SENSOR_FAIL_TO_ACCESS;
		}

		uint8_t fan_status_0_to_7 = msg.data[0];
		msg.data[0] = FAN_STATUS_8_TO_15_REG;
		if (i2c_master_read(&msg, retry) != 0) {
			LOG_ERR("Read FAN_STATUS_8_TO_15_REG fail");
			return SENSOR_FAIL_TO_ACCESS;
		}

		uint8_t fan_status_8_to_15 = msg.data[0];
		uint16_t fan_status = (fan_status_8_to_15 << 8) | (fan_status_0_to_7);
		for (int i = 0; i < NCT7363_PIN_NUMBER; i++) {
			if ((fan_status & 1) == 1) {
				LOG_ERR("FAN%d is not working", i);
				error_flag += 1;
			}
			fan_status = fan_status >> 1;
		}

		return (error_flag) ? SENSOR_UNSPECIFIED_ERROR : SENSOR_READ_SUCCESS;
	case NCT7363_GPIO_READ_OFFSET:
		msg.rx_len = 1;
		msg.tx_len = 1;
		if (port_offset >= 0 && port_offset <= NCT7363_8_PORT) {
			msg.data[0] = NCT7363_GPIO0x_OUTPUT_PORT_REG_OFFSET;
			if (i2c_master_read(&msg, retry) != 0) {
				LOG_ERR("Read NCT7363_GPIO0x_OUTPUT_PORT_REG_OFFSET fail");
				return SENSOR_FAIL_TO_ACCESS;
			}

			/* get port offset gpio data*/
			gpio_result = (msg.data[0] >> port_offset) & 1;
			/* return result */
			sensor_val *sval = (sensor_val *)reading;
			sval->integer = (int16_t)gpio_result;

			return SENSOR_READ_SUCCESS;

		} else if (port_offset >= NCT7363_10_PORT && port_offset <= NCT7363_17_PORT) {
			msg.data[0] = NCT7363_GPIO1x_OUTPUT_PORT_REG_OFFSET;
			if (i2c_master_read(&msg, retry) != 0) {
				LOG_ERR("Read NCT7363_GPIO1x_OUTPUT_PORT_REG_OFFSET fail");
				return SENSOR_FAIL_TO_ACCESS;
			}

			/* get port offset gpio data*/
			gpio_result = (msg.data[0] >> (port_offset - 8)) & 1;
			/* return result */
			sensor_val *sval = (sensor_val *)reading;
			sval->integer = (int16_t)gpio_result;

			return SENSOR_READ_SUCCESS;
		} else {
			LOG_ERR("Read GPIO port %d error!", port_offset);
			return SENSOR_UNSPECIFIED_ERROR;
		}
	default:
		LOG_ERR("Unknown register offset(%d)", offset);
		return SENSOR_UNSPECIFIED_ERROR;
	}
}

uint8_t nct7363_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	nct7363_init_arg *nct7363_init_arg_data = (nct7363_init_arg *)cfg->init_args;

	/* check pin_type is correct */
	for (uint8_t i = 0; i < NCT7363_PIN_NUMBER; i++) {
		if (nct7363_init_arg_data->pin_type[i] >= NCT7363_PIN_TPYE_ERROR) {
			LOG_ERR("Unknown pin_type, pin number(%d)", i);
			return SENSOR_UNSPECIFIED_ERROR;
		}
	}

	/* init_pin_config */
	uint8_t offset, val;
	for (uint8_t i = 0; i < NCT7363_PIN_FUNC_REG_SIZE; i++) {
		offset = GPIO_00_TO_03_PIN_CONFIGURATION_REG + i; // base reg
		// get last 2 bits of pin_type
		val = ((nct7363_init_arg_data->pin_type[3 + NCT7363_PIN_FUNC_REG_SIZE * i] &
			BIT_MASK(2))
		       << 6) | // 03 07 13 17
		      ((nct7363_init_arg_data->pin_type[2 + NCT7363_PIN_FUNC_REG_SIZE * i] &
			BIT_MASK(2))
		       << 4) | // 02 06 12 16
		      ((nct7363_init_arg_data->pin_type[1 + NCT7363_PIN_FUNC_REG_SIZE * i] &
			BIT_MASK(2))
		       << 2) | // 01 05 11 15
		      (nct7363_init_arg_data->pin_type[NCT7363_PIN_FUNC_REG_SIZE * i] &
		       BIT_MASK(2)); // 00 04 10 14
		if (!nct7363_write(cfg, offset, val)) {
			LOG_ERR("Error when setting PIN_CONFIGURATION_REG.");
			return SENSOR_INIT_UNSPECIFIED_ERROR;
		}
	}

	/* init gpio input/output */
	uint8_t val_gpio0x = 0xff, val_gpio1x = 0xff;

	for (uint8_t i = 0; i < NCT7363_PIN_NUMBER; i++) {
		if (nct7363_init_arg_data->pin_type[i] == NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT) {
			if (i < NCT7363_REG_SIZE)
				WRITE_BIT(val_gpio0x, i, 0);
			else
				WRITE_BIT(val_gpio1x, i - NCT7363_REG_SIZE, 0);
		}
	}

	if (!nct7363_write(cfg, GPIO0X_IO_CONF_REG, val_gpio0x))
		return SENSOR_INIT_UNSPECIFIED_ERROR;

	if (!nct7363_write(cfg, GPIO1X_IO_CONF_REG, val_gpio1x))
		return SENSOR_INIT_UNSPECIFIED_ERROR;

	/* set PWM frequency */
	uint8_t val_pwm_ctrl_0_7 = 0, val_pwm_ctrl_8_15 = 0, offset_pwm_freq = 0,
		output_init_freq = 0;

	for (int i = 0; i < NCT7363_PIN_NUMBER; i++) {
		if (nct7363_init_arg_data->pin_type[i] == NCT7363_PIN_TPYE_PWM) {
			if (i < NCT7363_REG_SIZE)
				WRITE_BIT(val_pwm_ctrl_0_7, i, 1);
			else
				WRITE_BIT(val_pwm_ctrl_8_15, i - NCT7363_REG_SIZE, 1);

			offset_pwm_freq = SPEED_CONTROL_PORT_DIVISOR_BASE_OFFSET + i * 2;
			bool convert_result = fan_frequency_convert(
				nct7363_init_arg_data->fan_frequency[i], &output_init_freq);

			if (!convert_result) {
				LOG_ERR("Invalid Frequncy.");
				return SENSOR_INIT_UNSPECIFIED_ERROR;
			}

			if (!nct7363_write(cfg, offset_pwm_freq, convert_result))
				return SENSOR_INIT_UNSPECIFIED_ERROR;
		}
	}

	/* enable PWM */
	if (!nct7363_write(cfg, NCT7363_PWM_CTRL_OUTPUT_0_TO_7_REG, val_pwm_ctrl_0_7))
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	if (!nct7363_write(cfg, NCT7363_PWM_CTRL_OUTPUT_8_TO_15_REG, val_pwm_ctrl_8_15))
		return SENSOR_INIT_UNSPECIFIED_ERROR;

	/* set FANIN */
	/* set threshold*/
	for (int i = 0; i < NCT7363_PIN_NUMBER; i++) {
		/* set init threshold  */
		bool set_threshold = nct7363_set_threshold(cfg, nct7363_init_arg_data->threshold);
		if (!set_threshold)
			return SENSOR_INIT_UNSPECIFIED_ERROR;

		/*  set init fan duty */
		bool set_duty = nct7363_set_duty(cfg, 40); // 40% duty for init
		if (!set_duty)
			return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	/* enable FANIN */
	uint8_t val_fanin_0_7 = 0, val_fanin_8_15 = 0;
	for (int i = 0; i < NCT7363_REG_SIZE; i++) {
		if (nct7363_init_arg_data->pin_type[i] == NCT7363_PIN_TPYE_FANIN)
			WRITE_BIT(val_fanin_8_15, i, 1);
		if (nct7363_init_arg_data->pin_type[i + NCT7363_REG_SIZE] == NCT7363_PIN_TPYE_FANIN)
			WRITE_BIT(val_fanin_0_7, i, 1);
	}

	/* enable fanin read tach*/
	if (!nct7363_write(cfg, NCT7363_FANIN_ENABLE_0_TO_7_REG, val_fanin_0_7))
		return SENSOR_INIT_UNSPECIFIED_ERROR;

	if (!nct7363_write(cfg, NCT7363_FANIN_ENABLE_8_TO_15_REG, val_fanin_8_15))
		return SENSOR_INIT_UNSPECIFIED_ERROR;

	/* set wdt  */
	offset = NCT7363_WDT_REG_OFFSET;
	uint8_t val_wdt = 0;
	uint8_t wdt_setting = nct7363_init_arg_data->wdt_cfg;
	if (wdt_setting < WDT_ERROR) {
		val_wdt = BIT(7) | (wdt_setting << 2);
	} else {
		LOG_ERR("WDT init fail !");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	if (!nct7363_write(cfg, offset, val_wdt))
		return SENSOR_INIT_UNSPECIFIED_ERROR;

	nct7363_init_arg_data->is_init = true;
	cfg->arg1 = nct7363_init_arg_data->fan_poles;
	cfg->read = nct7363_read;

	return SENSOR_INIT_SUCCESS;
}
