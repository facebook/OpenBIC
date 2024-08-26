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

#ifndef PLAT_UTIL_H
#define PLAT_UTIL_H

#include <stdlib.h>
#include "plat_modbus.h"
#include "plat_util.h"
#include "modbus_server.h"
#include <logging/log.h>
#include "util_spi.h"
#include "libutil.h"
#include "sensor.h"
#include "plat_gpio.h"
#include "plat_pwm.h"

#define I2C_MASTER_READ_BACK_MAX_SIZE 16 // 16 registers

static uint16_t temp_read_length;
static uint16_t temp_read_data[I2C_MASTER_READ_BACK_MAX_SIZE];

LOG_MODULE_REGISTER(plat_util);

bool modbus_i2c_master_write_read(const uint16_t *modbus_data, uint8_t data_len)
{
	// write data: bus(2Bytes), addr(2Bytes), read length(2Bytes), data(26Bytes)

	if (data_len <= 3) // check bus,addr,read length is not null
		return false;

	const uint8_t target_bus = modbus_data[0] & BIT_MASK(8); // get 7:0 bit data
	const uint8_t target_addr = (modbus_data[1] & BIT_MASK(8)) >> 1; //8-bit to 7-bit
	const uint8_t target_read_length = modbus_data[2] & BIT_MASK(8);
	I2C_MSG msg = { 0 };
	uint8_t retry = 5;
	msg.bus = target_bus;
	msg.target_addr = target_addr;
	msg.tx_len = data_len - 3; // write length need to -3 (bus,addr,read length)
	for (int i = 0; i < (data_len - 3); i++)
		msg.data[i] = modbus_data[i + 3] & BIT_MASK(8);

	if (target_read_length == 0) { // only write
		int result = i2c_master_write(&msg, retry);
		if (result != 0) {
			LOG_ERR("I2C write fail \n");
			return true;
		}
		return false;
	}

	temp_read_length = target_read_length;
	msg.rx_len = (int)temp_read_length;
	int result = i2c_master_read(&msg, retry);
	if (result != 0) {
		LOG_ERR("I2C read fail \n");
		return true;
	}

	memset(temp_read_data, 0xff, sizeof(temp_read_data));
	for (int i = 0; i < temp_read_length; i++)
		temp_read_data[i] = msg.data[i];

	return false;
}

void modbus_i2c_master_write_read_response(uint16_t *modbus_data)
{
	CHECK_NULL_ARG(modbus_data);
	// write data: bus(2Bytes), addr(2Bytes), read length(2Bytes), data(reg:2Bytes+data:24Bytes)
	memcpy(modbus_data, temp_read_data, sizeof(uint16_t) * temp_read_length);
}

void regs_reverse(uint16_t reg_len, uint16_t *data)
{
	CHECK_NULL_ARG(data);
	for (uint16_t i = 0; i < reg_len; i++)
		data[i] = sys_be16_to_cpu(data[i]);
}

void plat_enable_sensor_poll(void)
{
	enable_sensor_poll();
	nct7363_wdt_all_enable();
}

void plat_disable_sensor_poll(void)
{
	nct7363_wdt_all_disable();
	disable_sensor_poll();
}

void set_rpu_ready()
{
	gpio_set(BIC_RPU_READY0, 1);
	gpio_set(BIC_RPU_READY1, 1);
	gpio_set(BIC_RPU_READY2, 1);
	gpio_set(BIC_RPU_READY3, 1);
}

uint8_t get_rpu_ready_pin_status()
{
	if (gpio_get(BIC_RPU_READY0) && gpio_get(BIC_RPU_READY1) && gpio_get(BIC_RPU_READY2) &&
	    gpio_get(BIC_RPU_READY3))
		return 0;
	else
		return 1;
}

float pow_of_10(int8_t exp)
{
	float ret = 1.0;
	int i;

	if (exp < 0) {
		for (i = 0; i > exp; i--) {
			ret /= 10.0;
		}
	} else if (exp > 0) {
		for (i = 0; i < exp; i++) {
			ret *= 10.0;
		}
	}

	return ret;
}

#endif // PLAT_UTIL_H