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
 *
 * I2C target-mode test device for the MCX-N9XX-EVK port. Registers a
 * tiny byte-addressable register file (like a minimal I2C EEPROM) on
 * flexcomm2_lpi2c2 (I2C_BUS_LCD_HEADER) using Zephyr's generic
 * i2c_target_* API, which the MCUX LPI2C driver (i2c_mcux_lpi2c.c)
 * implements natively for this SoC - this is real target-mode
 * hardware operation, not a software stand-in.
 *
 * Both flexcomm2_lpi2c2 and flexcomm3_lpi2c3's SDA/SCL lines land on
 * the same physical header - the Arduino-compatible header J2 (see
 * README.md for the full, verified pin table) - so exercising this
 * against real silicon just needs two short jumpers on that header,
 * then from the shell use flexcomm3_lpi2c3 (controller mode) to
 * write/read this device at I2C_TARGET_TEST_ADDR:
 *
 *   i2c write flexcomm3_lpi2c3 0x50 0x00 0xde 0xad 0xbe 0xef
 *   i2c read  flexcomm3_lpi2c3 0x50 0x00 4
 *
 * A write's first byte selects the register offset (like a real
 * EEPROM); subsequent bytes write sequentially from there. A read
 * (with or without a preceding register-offset write) returns bytes
 * sequentially from the last-selected offset.
 *
 * This wire-level test has been attempted but is currently untested/
 * inconclusive, not confirmed working or broken - see README.md's
 * "I2C target mode" section for the full writeup. The registration
 * and callbacks here are verified correct in isolation on real
 * hardware.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

#include "plat_i2c.h"

LOG_MODULE_REGISTER(plat_i2c_target, LOG_LEVEL_INF);

#define I2C_TARGET_TEST_ADDR 0x50
#define I2C_TARGET_REGFILE_SIZE 256

static uint8_t regfile[I2C_TARGET_REGFILE_SIZE];
static uint8_t cur_offset;
static bool have_offset;

static int target_write_requested(struct i2c_target_config *config)
{
	ARG_UNUSED(config);
	have_offset = false;
	return 0;
}

static int target_write_received(struct i2c_target_config *config, uint8_t val)
{
	ARG_UNUSED(config);

	if (!have_offset) {
		cur_offset = val;
		have_offset = true;
		return 0;
	}

	regfile[cur_offset] = val;
	cur_offset++;
	return 0;
}

static int target_read_requested(struct i2c_target_config *config, uint8_t *val)
{
	ARG_UNUSED(config);

	*val = regfile[cur_offset];
	cur_offset++;
	return 0;
}

static int target_read_processed(struct i2c_target_config *config, uint8_t *val)
{
	ARG_UNUSED(config);

	*val = regfile[cur_offset];
	cur_offset++;
	return 0;
}

static int target_stop(struct i2c_target_config *config)
{
	ARG_UNUSED(config);
	have_offset = false;
	return 0;
}

static const struct i2c_target_callbacks target_callbacks = {
	.write_requested = target_write_requested,
	.write_received = target_write_received,
	.read_requested = target_read_requested,
	.read_processed = target_read_processed,
	.stop = target_stop,
};

static struct i2c_target_config target_config = {
	.address = I2C_TARGET_TEST_ADDR,
	.callbacks = &target_callbacks,
};

void plat_i2c_target_init(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(flexcomm2_lpi2c2));
	int ret;

	if (!device_is_ready(dev)) {
		LOG_ERR("flexcomm2_lpi2c2 not ready, skipping I2C target registration");
		return;
	}

	ret = i2c_target_register(dev, &target_config);
	if (ret) {
		LOG_ERR("i2c_target_register failed: %d", ret);
		return;
	}

	LOG_INF("I2C target registered on flexcomm2_lpi2c2, addr 0x%02x, %d-byte regfile",
		I2C_TARGET_TEST_ADDR, I2C_TARGET_REGFILE_SIZE);
}
