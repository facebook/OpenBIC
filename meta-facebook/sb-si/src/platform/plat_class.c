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
#include <stdlib.h>
#include <string.h>
#include <logging/log.h>

#include "libutil.h"
#include "plat_i2c.h"
#include "plat_class.h"
#include "plat_pldm_sensor.h"
#include "hal_i3c.h"
#include "hal_i2c.h"
#include "plat_i3c.h"
#include <drivers/flash.h>

LOG_MODULE_REGISTER(plat_class);

#define I2C_BUS_TMP I2C_BUS1
#define TMP_EMC1413_SMSC_ID_DEFAULT 0x5D
#define INVALID_SLOT_ID 0xFF

uint16_t BIC_PID = PLAT_DEFAULT_PID;

static uint8_t vr_type = VR_UNKNOWN;
static uint8_t tmp_type = TMP_TYPE_UNKNOWN;

const mmc_info_t mmc_info_table[MAX_SLOT] = {
	{ .slot = 0, .eid = MCTP_DEFAULT_ENDPOINT, .pid = PLAT_DEFAULT_PID },
	{ .slot = 1, .eid = 0x14, .pid = 0x520 },
	{ .slot = 2, .eid = 0x1E, .pid = 0x530 },
	{ .slot = 3, .eid = 0x28, .pid = 0x540 },
};

void init_vr_vendor_type(void)
{
	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 3;
	i2c_msg.bus = I2C_BUS1;
	i2c_msg.target_addr = VR_ASIC_P0V895_PEX_MP2971_ADDR;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 1;
	i2c_msg.data[0] = 0x00;

	int ret = i2c_master_read(&i2c_msg, retry);

	if (0 == ret) {
		vr_type = VR_MPS_MP2971_MP2891;
	} else {
		vr_type = VR_RNS_ISL69260_RAA228249;
	}
	LOG_INF("VR_TYPE(0x%02X) ", vr_type);
}

void init_tmp_type()
{
	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;
	i2c_msg.bus = I2C_BUS_TMP;
	i2c_msg.target_addr = THERMAL_SENSOR_1_ADDR;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 1;
	i2c_msg.data[0] = 0xFE; //MFG ID REG

	int ret = i2c_master_read(&i2c_msg, retry);

	if (0 == ret) {
		LOG_INF("Assume TMP is EMC1413 by address check");
		tmp_type = TMP_EMC1413;
		return;
	} else {
		LOG_INF("Assume TMP is TMP432 by register check");
		tmp_type = TMP_TMP432;
		return;
	}
}

uint8_t get_vr_type()
{
	return vr_type;
}

uint8_t get_tmp_type()
{
	return tmp_type;
}

void init_platform_config()
{
	init_vr_vendor_type();
	init_tmp_type();
}

void plat_i3c_set_pid()
{
	uint8_t slot_id = get_slot_id();

	if (slot_id < MAX_SLOT) {
		BIC_PID = mmc_info_table[slot_id].pid;
		LOG_INF("Slot ID %d valid, set BIC_PID = 0x%02x", slot_id, BIC_PID);
	} else {
		LOG_WRN("Invalid slot ID (%d), keep default BIC_PID = 0x%02x", slot_id, BIC_PID);
	}

	I3C_MSG i3c_msg = { 0 };
	i3c_msg.bus = I3C_BUS6;
	i3c_set_pid(&i3c_msg, BIC_PID);
}

uint8_t get_slot_id()
{
	const struct device *flash_dev = device_get_binding("spi_spim0_cs0");
	if (!flash_dev) {
		LOG_ERR("Failed to get flash device for slot ID.");
		return INVALID_SLOT_ID;
	}

	uint8_t slot_id = INVALID_SLOT_ID;
	uint32_t op_addr = FLASH_SLOT_ADDRESS;

	if (flash_read(flash_dev, op_addr, &slot_id, 1) != 0) {
		LOG_ERR("Failed to read slot ID from flash at 0x%x", op_addr);
		return INVALID_SLOT_ID;
	}

	if (slot_id >= MAX_SLOT) {
		LOG_ERR("Slot ID read from flash is invalid: %d", slot_id);
		return INVALID_SLOT_ID;
	}

	LOG_INF("Read slot ID %d from flash", slot_id);
	return slot_id;
}