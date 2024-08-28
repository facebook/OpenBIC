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

LOG_MODULE_REGISTER(plat_class);

#define AEGIS_CPLD_ADDR (0x4C >> 1)
#define I2C_BUS_CPLD I2C_BUS5
#define AEGIS_CPLD_VR_VENDOR_TYPE_REG 0x1C

static uint8_t vr_vender_type = VR_VENDOR_UNKNOWN;
static uint8_t vr_type = VR_UNKNOWN;
static uint8_t ubc_type = UBC_UNKNOWN;

bool plat_read_cpld(uint8_t offset, uint8_t *data)
{
	CHECK_NULL_ARG_WITH_RETURN(data, 1);

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

void get_vr_vendor_type(void)
{
	//get CPLD VR_VENDOR_TYPE
	if (!plat_read_cpld(AEGIS_CPLD_VR_VENDOR_TYPE_REG, &vr_vender_type)) {
		LOG_ERR("Failed to get CPLD VR_VENDOR_TYPE 0x%02X", AEGIS_CPLD_VR_VENDOR_TYPE_REG);
	}

	LOG_INF("VR_VENDOR_TYPE = 0x%02X", vr_vender_type);

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
}

uint8_t get_vr_type()
{
	return vr_type;
}

uint8_t get_ubc_type()
{
	return ubc_type;
}

void init_platform_config()
{
	get_vr_vendor_type();
}
