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

LOG_MODULE_REGISTER(plat_class);

#define AEGIS_CPLD_ADDR (0x4C >> 1)
#define I2C_BUS_CPLD I2C_BUS5
#define I2C_BUS_TMP I2C_BUS1
#define AEGIS_CPLD_VR_VENDOR_TYPE_REG 0x1C
#define AEGIS_CPLD_BOARD_REV_ID_REG 0x1B
#define AEGIS_CPLD_BOARD_TYPE_REG 0x1A

#define TMP_EMC1413_SMSC_ID_DEFAULT 0x5D

static uint8_t vr_vender_type = VR_VENDOR_UNKNOWN;
static uint8_t vr_type = VR_UNKNOWN;
static uint8_t ubc_type = UBC_UNKNOWN;
static uint8_t tmp_type = TMP_TYPE_UNKNOWN;
static uint8_t board_stage = BOARD_STAGE_UNKNOWN;
static uint8_t board_type = BOARD_TYPE_UNKNOWN;

bool plat_read_cpld(uint8_t offset, uint8_t *data)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;
	i2c_msg.bus = I2C_BUS_CPLD;
	i2c_msg.target_addr = AEGIS_CPLD_ADDR;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 1;
	i2c_msg.data[0] = offset;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Failed to read CPLD register 0x%02X", offset);
		return false;
	}

	*data = i2c_msg.data[0];
	return true;
}

void init_board_type(void)
{
	uint8_t board_type_data = BOARD_TYPE_UNKNOWN;
	//get CPLD BOARD_TYPE
	if (!plat_read_cpld(AEGIS_CPLD_BOARD_TYPE_REG, &board_type_data)) {
		LOG_ERR("Failed to get CPLD BOARD_TYPE 0x%02X", AEGIS_CPLD_BOARD_TYPE_REG);
	}

	//print board type word
	switch (board_type_data) {
	case MINERVA_AEGIS_BD:
		board_type = MINERVA_AEGIS_BD;
		LOG_INF("BOARD_TYPE(0x%02X) = AEGIS_BD", board_type);
		break;
	case MINERVA_EVB_BD:
		board_type = MINERVA_EVB_BD;
		LOG_INF("BOARD_TYPE(0x%02X) = EVB_BD", board_type);
		break;
	default:
		LOG_INF("BOARD_TYPE = 0x%02X", board_type_data);
		break;
	}
}

void init_board_stage(void)
{
	uint8_t board_stage_data = BOARD_STAGE_UNKNOWN;
	//get CPLD BOARD_STAGE
	if (!plat_read_cpld(AEGIS_CPLD_BOARD_REV_ID_REG, &board_stage_data)) {
		LOG_ERR("Failed to get CPLD BOARD_STAGE 0x%02X", AEGIS_CPLD_BOARD_REV_ID_REG);
	}

	//print board stage word
	switch (board_stage_data) {
	case FAB1_EVT:
		board_stage = FAB1_EVT;
		LOG_INF("BOARD_STAGE(0x%02X) = EVT", board_stage);
		break;
	case FAB2_DVT:
		board_stage = FAB2_DVT;
		LOG_INF("BOARD_STAGE(0x%02X) = DVT", board_stage);
		break;
	case FAB3_PVT:
		board_stage = FAB3_PVT;
		LOG_INF("BOARD_STAGE(0x%02X) = PVT", board_stage);
		break;
	case FAB4_MP:
		board_stage = FAB4_MP;
		LOG_INF("BOARD_STAGE(0x%02X) = MP", board_stage);
		break;
	default:
		LOG_INF("BOARD_STAGE = 0x%02X", board_stage_data);
		break;
	}
}

void init_vr_vendor_type(void)
{
	//get CPLD VR_VENDOR_TYPE
	if (!plat_read_cpld(AEGIS_CPLD_VR_VENDOR_TYPE_REG, &vr_vender_type)) {
		LOG_ERR("Failed to get CPLD VR_VENDOR_TYPE 0x%02X", AEGIS_CPLD_VR_VENDOR_TYPE_REG);
	}

	LOG_INF("VR_VENDOR_TYPE = 0x%02X", vr_vender_type);

	switch (board_stage) {
	case FAB1_EVT:
		switch (vr_vender_type) {
		case DELTA_UBC_AND_MPS_VR:
			ubc_type = UBC_DELTA_U50SU4P180PMDAFC;
			vr_type = VR_MPS_MP2971_MP2891;
			break;
		case DELTA_UBC_AND_RNS_VR:
			ubc_type = UBC_DELTA_U50SU4P180PMDAFC;
			vr_type = VR_RNS_ISL69260_RAA228238;
			break;
		case MPS_UBC_AND_MPS_VR:
			ubc_type = UBC_MPS_MPC12109;
			vr_type = VR_MPS_MP2971_MP2891;
			break;
		case MPS_UBC_AND_RNS_VR:
			ubc_type = UBC_MPS_MPC12109;
			vr_type = VR_RNS_ISL69260_RAA228238;
			break;
		case FLEX_UBC_AND_MPS_VR:
			ubc_type = UBC_FLEX_BMR313;
			vr_type = VR_MPS_MP2971_MP2891;
			break;
		case FLEX_UBC_AND_RNS_VR:
			ubc_type = UBC_FLEX_BMR313;
			vr_type = VR_RNS_ISL69260_RAA228238;
			break;
		default:
			LOG_WRN("vr vendor type not supported: 0x%x", vr_vender_type);
			break;
		}
		break; //case FAB1_EVT
	case FAB2_DVT:
	case FAB3_PVT:
	case FAB4_MP:
		switch (vr_vender_type) {
		case DELTA_UBC_AND_MPS_VR:
			ubc_type = UBC_DELTA_U50SU4P180PMDAFC;
			vr_type = VR_MPS_MP2971_MP29816A;
			break;
		case DELTA_UBC_AND_RNS_VR:
			ubc_type = UBC_DELTA_U50SU4P180PMDAFC;
			vr_type = VR_RNS_ISL69260_RAA228249;
			break;
		case MPS_UBC_AND_MPS_VR:
			ubc_type = UBC_MPS_MPC12109;
			vr_type = VR_MPS_MP2971_MP29816A;
			break;
		case MPS_UBC_AND_RNS_VR:
			ubc_type = UBC_MPS_MPC12109;
			vr_type = VR_RNS_ISL69260_RAA228249;
			break;
		case FLEX_UBC_AND_MPS_VR:
			ubc_type = UBC_FLEX_BMR313;
			vr_type = VR_MPS_MP2971_MP29816A;
			break;
		case FLEX_UBC_AND_RNS_VR:
			ubc_type = UBC_FLEX_BMR313;
			vr_type = VR_RNS_ISL69260_RAA228249;
			break;
		default:
			LOG_WRN("vr vendor type not supported: 0x%x", vr_vender_type);
			break;
		}
		break; //case FAB2_DVT, FAB3_PVT, FAB4_MP
	default:
		LOG_WRN("board stage not supported: 0x%x", board_stage);
		break;
	}

	//print ubc and vr type
	switch (ubc_type) {
	case UBC_DELTA_U50SU4P180PMDAFC:
		LOG_INF("UBC_TYPE(0x%02X) = DELTA_U50SU4P180PMDAFC", ubc_type);
		break;
	case UBC_MPS_MPC12109:
		LOG_INF("UBC_TYPE(0x%02X) = MPS_MPC12109", ubc_type);
		break;
	case UBC_FLEX_BMR313:
		LOG_INF("UBC_TYPE(0x%02X) = FLEX_BMR313", ubc_type);
		break;
	default:
		LOG_WRN("ubc type not supported: 0x%x", ubc_type);
		break;
	}

	switch (vr_type) {
	case VR_MPS_MP2971_MP2891:
		LOG_INF("VR_TYPE(0x%02X) = MPS_MP2971_MP2891", vr_type);
		break;
	case VR_MPS_MP2971_MP29816A:
		LOG_INF("VR_TYPE(0x%02X) = MPS_MP2971_MP29816A", vr_type);
		break;
	case VR_RNS_ISL69260_RAA228238:
		LOG_INF("VR_TYPE(0x%02X) = RNS_ISL69260_RAA228238", vr_type);
		break;
	case VR_RNS_ISL69260_RAA228249:
		LOG_INF("VR_TYPE(0x%02X) = RNS_ISL69260_RAA228249", vr_type);
		break;
	default:
		LOG_WRN("vr type not supported: 0x%x", vr_type);
		break;
	}
}

void init_tmp_type()
{
	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;
	i2c_msg.bus = I2C_BUS_TMP;
	i2c_msg.target_addr = ASIC_DIE_ATH_SENSOR_0_TEMP_TMP432_ADDR;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 1;
	i2c_msg.data[0] = 0xFE; //MFG ID REG

	if (i2c_master_read(&i2c_msg, retry)) {
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

uint8_t get_ubc_type()
{
	return ubc_type;
}

uint8_t get_board_stage()
{
	return board_stage;
}

uint8_t get_board_type()
{
	return board_type;
}

uint8_t get_tmp_type()
{
	return tmp_type;
}

void init_platform_config()
{
	init_board_type();
	init_board_stage();
	init_vr_vendor_type();
	init_tmp_type();
}
