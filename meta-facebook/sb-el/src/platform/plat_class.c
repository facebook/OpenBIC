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
#include "plat_cpld.h"
#include "plat_mctp.h"

LOG_MODULE_REGISTER(plat_class);

static uint8_t vr_module = VR_MODULE_UNKNOWN;
static uint8_t ubc_module = UBC_MODULE_UNKNOWN;
// static uint8_t tmp_module = TMP_TMP432;
static uint8_t vr_vendor_module = VENDOR_TYPE_UNKNOWN;
static uint8_t mmc_slot = 0;
static uint8_t asic_board_id = 0;
static uint8_t tray_location = 0;
static uint8_t board_rev_id = 0;

bool plat_cpld_eerprom_read(uint8_t *data, uint16_t offset, uint8_t len)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);

	memset(data, 0, 1);

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;
	i2c_msg.bus = I2C_BUS11;
	i2c_msg.target_addr = CPLD_EEPROM_ADDR;
	i2c_msg.tx_len = 2;
	i2c_msg.rx_len = len;
	i2c_msg.data[0] = offset >> 8;
	i2c_msg.data[1] = offset & 0xFF;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Failed to read cpld fru offset 0");
		return false;
	}

	memcpy(data, i2c_msg.data, len);
	return true;
}

void init_vr_vendor_module(void)
{
	vr_vendor_module = VENDOR_TYPE_UNKNOWN;

	switch (ubc_module) {
	case UBC_MODULE_MPS:
		if (vr_module == VR_MODULE_MPS) {
			vr_vendor_module = MPS_UBC_AND_MPS_VR;
		} else if (vr_module == VR_MODULE_RNS) {
			vr_vendor_module = MPS_UBC_AND_RNS_VR;
		}
		break;

	case UBC_MODULE_DELTA:
		if (vr_module == VR_MODULE_MPS) {
			vr_vendor_module = DELTA_UBC_AND_MPS_VR;
		} else if (vr_module == VR_MODULE_RNS) {
			vr_vendor_module = DELTA_UBC_AND_RNS_VR;
		}
		break;

	case UBC_MODULE_LUXSHARE:
		if (vr_module == VR_MODULE_MPS) {
			vr_vendor_module = LUXSHURE_UBC_AND_MPS_VR;
		} else if (vr_module == VR_MODULE_RNS) {
			vr_vendor_module = LUXSHURE_UBC_AND_RNS_VR;
		}
		break;

	default:
		vr_vendor_module = VENDOR_TYPE_UNKNOWN;
		break;
	}

	LOG_INF("vr_vendor_module=%d (ubc=%d, vr=%d)", vr_vendor_module, ubc_module, vr_module);
}

void init_plat_config()
{
	uint8_t module = 0;
	plat_read_cpld(CPLD_OFFSET_VR_VENDER_TYPE, &module, 1);
	plat_read_cpld(CPLD_OFFSET_BOARD_REV_ID, &board_rev_id, 1);
	// rev id only support 0, 1, 2 bit
	board_rev_id = board_rev_id & 0x07;
	vr_module = (module & 0x01);
	ubc_module = (module >> 1) & 0x03;
	uint8_t board_id = 0;
	plat_read_cpld(CPLD_OFFSET_ASIC_BOARD_ID, &board_id, 1);
	asic_board_id = board_id & 0x03;
	init_vr_vendor_module();
	change_sensor_cfg(asic_board_id, vr_module, ubc_module, board_rev_id);
	// cpld fru offset 0: slot
	plat_cpld_eerprom_read(&mmc_slot, 0, 1);
	// mmc slot 1-4 * 0x0A
	uint8_t init_plat_eid = ((get_mmc_slot() + 1) * MCTP_DEFAULT_ENDPOINT);
	plat_set_eid(init_plat_eid);
	// cpld fru offset 0x3FF: tray location
	plat_cpld_eerprom_read(&tray_location, 1023, 1);
	LOG_INF("init_plat_eid: 0x%x", init_plat_eid);
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

uint8_t get_asic_board_id()
{
	return asic_board_id;
}

uint8_t get_board_rev_id()
{
	return board_rev_id;
}

uint8_t get_tray_location()
{
	return tray_location;
}

// clang-format off
