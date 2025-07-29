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
#include <logging/log.h>

#include "plat_class.h"
#include "plat_cpld.h"
#include "plat_pldm_sensor.h"
#include "plat_fru.h"
#include "plat_i2c.h"
#include "plat_util.h"

LOG_MODULE_REGISTER(plat_class);

static uint8_t vr_module = 0;
static uint8_t ubc_module = 0;
static uint8_t mmc_slot = 0;

static bool plat_slot_read(uint8_t *data)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);

	memset(data, 0, 1);

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;
	i2c_msg.bus = I2C_BUS11;
	i2c_msg.target_addr = CPLD_EEPROM_ADDR;
	i2c_msg.tx_len = 2;
	i2c_msg.rx_len = 1;
	i2c_msg.data[0] = 0;
	i2c_msg.data[1] = 0;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Failed to read cpld fru offset 0");
		return false;
	}

	memcpy(data, i2c_msg.data, 1);
	return true;
}

void init_plat_config()
{
	uint8_t module = 0;
	plat_read_cpld(CPLD_OFFSET_VR_VENDER_TYPE, &module, 1);

	vr_module = (module & 0x01);
	ubc_module = (module >> 1) & 0x03;

	change_sensor_cfg(vr_module);

	// cpld fru offset 0: slot
	plat_slot_read(&mmc_slot);
}

uint8_t get_vr_module()
{
	return vr_module;
}

uint8_t get_ubc_module()
{
	return ubc_module;
}

uint8_t get_mmc_slot()
{
	return mmc_slot;
}