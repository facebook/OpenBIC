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
#include "plat_user_setting.h"
#include "plat_adc.h"

#define BOARD_TYPE_MASK (BIT(0) | BIT(1))
#define REV_ID_MASK (BIT(0) | BIT(1) | BIT(2))

LOG_MODULE_REGISTER(plat_class);

static uint8_t vr_module = VR_MODULE_UNKNOWN;
static uint8_t ubc_module = UBC_MODULE_UNKNOWN;
static uint8_t tmp_module = TMP_TMP432;
static uint8_t vr_vendor_module = VENDOR_TYPE_UNKNOWN;
static uint8_t asic_board_id = ASIC_BOARD_ID_UNKNOWN;
static uint8_t board_rev_id = REV_ID_UNKNOWN;
static uint8_t mmc_slot = 0;
static uint8_t tray_location = 0;

// clang-format off
const char *vr_vendor_module_name[] = {
	"DELTA_UBC_AND_MPS_VR",
	"DELTA_UBC_AND_RNS_VR",
	"MPS_UBC_AND_MPS_VR",
	"MPS_UBC_AND_RNS_VR",
	"LUXSHURE_UBC_AND_MPS_VR",
	"LUXSHURE_UBC_AND_RNS_VR",
	"VENDOR_TYPE_UNKNOWN",
};

const char *vr_module_name[] = {
	"VR_MODULE_MPS",
	"VR_MODULE_RNS",
	"VR_MODULE_UNKNOWN",
};

const char *ubc_module_name[] = {
	"UBC_MODULE_DELTA",
	"UBC_MODULE_MPS",
	"UBC_MODULE_FLEX",
	"UBC_MODULE_LUXSHARE",
	"UBC_MODULE_UNKNOWN",
};
// clang-format on

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

void init_board_type(void)
{
	uint8_t board_type_data = ASIC_BOARD_ID_UNKNOWN;
	//get CPLD BOARD_TYPE
	if (!plat_read_cpld(CPLD_OFFSET_ASIC_BOARD_ID, &board_type_data, 1)) {
		LOG_ERR("Failed to get CPLD BOARD_TYPE 0x%02X", CPLD_OFFSET_ASIC_BOARD_ID);
	}
	board_type_data = board_type_data & BOARD_TYPE_MASK;

	//print board type word
	switch (board_type_data) {
	case ASIC_BOARD_ID_ELECTRA:
		asic_board_id = ASIC_BOARD_ID_ELECTRA;
		LOG_INF("BOARD_TYPE(0x%02X) = ELECTRA", asic_board_id);
		break;
	case ASIC_BOARD_ID_EVB:
		asic_board_id = ASIC_BOARD_ID_EVB;
		LOG_INF("BOARD_TYPE(0x%02X) = EVB_BD", asic_board_id);
		break;
	default:
		LOG_ERR("BOARD_TYPE = 0x%02X", board_type_data);
		break;
	}
}

void init_board_stage(void)
{
	uint8_t board_stage_data = REV_ID_UNKNOWN;
	//get CPLD BOARD_STAGE
	if (!plat_read_cpld(CPLD_OFFSET_BOARD_REV_ID, &board_stage_data, 1)) {
		LOG_ERR("Failed to get CPLD BOARD_STAGE 0x%02X", CPLD_OFFSET_BOARD_REV_ID);
	}
	board_stage_data = board_stage_data & REV_ID_MASK;

	//print board stage word
	switch (board_stage_data) {
	case REV_ID_EVT1A:
		board_rev_id = REV_ID_EVT1A;
		LOG_INF("BOARD_STAGE(0x%02X) = EVT1A", board_rev_id);
		break;
	case REV_ID_EVT1B:
		board_rev_id = REV_ID_EVT1B;
		LOG_INF("BOARD_STAGE(0x%02X) = EVT1B", board_rev_id);
		break;
	case REV_ID_EVT2:
		board_rev_id = REV_ID_EVT2;
		LOG_INF("BOARD_STAGE(0x%02X) = EVT2", board_rev_id);
		break;
	case REV_ID_DVT:
		board_rev_id = REV_ID_DVT;
		LOG_INF("BOARD_STAGE(0x%02X) = DVT", board_rev_id);
		break;
	case REV_ID_PVT:
		board_rev_id = REV_ID_PVT;
		LOG_INF("BOARD_STAGE(0x%02X) = PVT", board_rev_id);
		break;
	case REV_ID_MP:
		board_rev_id = REV_ID_MP;
		LOG_INF("BOARD_STAGE(0x%02X) = MP", board_rev_id);
		break;
	default:
		LOG_INF("BOARD_STAGE = 0x%02X", board_stage_data);
		break;
	}
}

void init_vr_vendor_type(void)
{
	//get CPLD VR_VENDOR_TYPE
	if (!plat_read_cpld(CPLD_OFFSET_VR_VENDER_TYPE, &vr_vendor_module, 1)) {
		LOG_ERR("Failed to get CPLD VR_VENDOR_TYPE 0x%02X", CPLD_OFFSET_VR_VENDER_TYPE);
	}

	vr_module = (vr_vendor_module & 0x01);
	ubc_module = (vr_vendor_module >> 1) & 0x03;

	LOG_INF("vr_vendor_module=%s (ubc=%s, vr=%s)", vr_vendor_module_name[vr_vendor_module],
		ubc_module_name[ubc_module], vr_module_name[vr_module]);
}

void init_plat_config()
{
	init_board_type();
	init_board_stage();
	init_vr_vendor_type();
	change_sensor_cfg(asic_board_id, vr_module, ubc_module, board_rev_id);
	// cpld fru offset 0: slot
	plat_cpld_eerprom_read(&mmc_slot, MMC_SLOT_USER_SETTING_OFFSET, 1);
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
void pal_show_board_types(const struct shell *shell)
{
	uint8_t adc_type = 0;
	adc_type = get_adc_type();

	shell_print(shell, "* BOARD_TYPE:    (0x%02X)%s", asic_board_id,
		    (asic_board_id == ASIC_BOARD_ID_ELECTRA) ? "ELECTRA" :
		    (asic_board_id == ASIC_BOARD_ID_EVB)     ? "EVB" :
							       "not supported");

	if (asic_board_id == ASIC_BOARD_ID_EVB) {
		shell_print(shell, "* BOARD_STAGE:   (0x%02X)%s", board_rev_id,
			    (board_rev_id == REV_ID_EVT1A) ? "REV_ID_EVT1A" :
			    (board_rev_id == REV_ID_EVT1B) ? "REV_ID_EVT1B" :
			    (board_rev_id == REV_ID_EVT2)  ? "REV_ID_EVT2" :
			    (board_rev_id == REV_ID_DVT)   ? "REV_ID_DVT" :
			    (board_rev_id == REV_ID_PVT)   ? "REV_ID_PVT" :
			    (board_rev_id == REV_ID_MP)	   ? "REV_ID_MP" :
							     "not supported");
	} else if (asic_board_id == ASIC_BOARD_ID_ELECTRA) {
		shell_print(shell, "* BOARD_STAGE:   (0x%02X)%s", board_rev_id,
			    (board_rev_id == REV_ID_EVT1A) ? "REV_ID_EVT1A" :
			    (board_rev_id == REV_ID_EVT1B) ? "REV_ID_EVT1B" :
			    (board_rev_id == REV_ID_EVT2)  ? "REV_ID_EVT2" :
			    (board_rev_id == REV_ID_DVT)   ? "REV_ID_DVT" :
			    (board_rev_id == REV_ID_PVT)   ? "REV_ID_PVT" :
			    (board_rev_id == REV_ID_MP)	   ? "REV_ID_MP" :
							     "not supported");
	}

	shell_print(shell, "* VR_VENDOR_TYPE:(0x%02X)%s", vr_vendor_module,
		    (vr_vendor_module == DELTA_UBC_AND_MPS_VR)	  ? "DELTA_UBC_AND_MPS_VR" :
		    (vr_vendor_module == DELTA_UBC_AND_RNS_VR)	  ? "DELTA_UBC_AND_RNS_VR" :
		    (vr_vendor_module == MPS_UBC_AND_MPS_VR)	  ? "MPS_UBC_AND_MPS_VR" :
		    (vr_vendor_module == MPS_UBC_AND_RNS_VR)	  ? "MPS_UBC_AND_RNS_VR" :
		    (vr_vendor_module == LUXSHURE_UBC_AND_MPS_VR) ? "LUXSHURE_UBC_AND_MPS_VR" :
		    (vr_vendor_module == LUXSHURE_UBC_AND_RNS_VR) ? "LUXSHURE_UBC_AND_RNS_VR" :
								    "not supported");
	shell_print(shell, "* UBC_TYPE:      (0x%02X)%s", ubc_module,
		    (ubc_module == UBC_MODULE_DELTA)	? "UBC_DELTA_S54SS4P1A2" :
		    (ubc_module == UBC_MODULE_MPS)	? "UBC_MPS_MPC12109" :
		    (ubc_module == UBC_MODULE_LUXSHARE) ? "UBC_LUXSHURE_LX6310" :
							  "not supported");

	shell_print(shell, "* VR_TYPE:       (0x%02X)%s", vr_module,
		    (vr_module == VR_MODULE_MPS) ? "VR_MPS_MP2971_MP29816C" :
		    (vr_module == VR_MODULE_RNS) ? "VR_RNS_RAA229140_RAA228249" :
						   "not supported");

	shell_print(shell, "* TMP_TYPE:      (0x%02X)%s", tmp_module,
		    (tmp_module == TMP_TMP432) ? "TMP_TMP75_TMP432" : "not supported");

	shell_print(shell, "* ADC_TYPE:      (0x%02X)%s", adc_type,
		    (adc_type == ADI_AD4058) ? "ADI_AD4058" :
		    (adc_type == TIC_ADS7066) ? "TI_ADS7066" : "not supported");
	
	shell_print(shell, "* I2C connection for MEDHA0/1 to MMC: Disable");
	return;
}

// clang-format on

void pal_show_extra_info(const struct shell *shell)
{
	pal_show_board_types(shell);

	return;
}