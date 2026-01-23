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

#ifndef PLAT_LOG_H
#define PLAT_LOG_H

#include "plat_pldm_sensor.h"

#define CPLD_REGISTER_MAX_NUM 72
#define CPLD_REGISTER_1ST_PART_START_OFFSET                                                        \
	0x00 // first part of cpld register offset from 0x00 to 0x47
#define CPLD_REGISTER_1ST_PART_NUM 72
#define FRU_LOG_SIZE sizeof(plat_err_log_mapping)

#define LOG_ASSERT 1
#define LOG_DEASSERT 0

#define VR_POWER_FAULT_1_REG 0x0D
#define VR_POWER_FAULT_2_REG 0x0E
#define VR_POWER_FAULT_3_REG 0x0F
#define VR_POWER_FAULT_4_REG 0x10
#define VR_POWER_FAULT_5_REG 0x11

enum VR_ERR_LOG_DEVICE_INDEX_E {
	//pwr fault reg 1
	PWRGD_OWL_E_TRVDD0P9_R_FAULT = 1,
	PWRGD_OWL_W_TRVDD0P9_R_FAULT,
	PWRGD_OWL_E_TRVDD0P75_R_FAULT,
	PWRGD_OWL_W_TRVDD0P75_R_FAULT,
	PWRGD_HAMSA_AVDD_PCIE_R_FAULT,
	PWRGD_HAMSA_VDDHRXTX_PCIE_R_FAULT,
	PWRGD_P4V2_R_FAULT,
	PWRGD_P0V75_AVDD_HCSL_R_FAULT,
	//pwr fault reg 2
	PWRGD_MEDHA1_VDD_FAULT,
	PWRGD_MEDHA0_VDD_FAULT,
	PWRGD_OWL_E_VDD_R_FAULT,
	PWRGD_OWL_W_VDD_R_FAULT,
	PWRGD_HAMSA_VDD_R_FAULT,
	PWRGD_MAX_S_VDD_R_FAULT,
	PWRGD_MAX_M_VDD_R_FAULT,
	PWRGD_MAX_N_VDD_R_FAULT,
	//pwr fault reg 3
	PWRGD_VDDQL_HBM0_HBM2_HBM4_HBM6_R_FAULT,
	PWRGD_VDDQC_HBM0_HBM2_HBM4_HBM6_R_FAULT,
	PWRGD_VPP_HBM0_HBM2_HBM4_HBM6_R_FAULT,
	PWRGD_VDDPHY_HBM0_HBM2_HBM4_HBM6_R_FAULT,
	PWRGD_VDDQL_HBM1_HBM3_HBM5_HBM7_R_FAULT,
	PWRGD_VDDQC_HBM1_HBM3_HBM5_HBM7_R_FAULT,
	PWRGD_VPP_HBM1_HBM3_HBM5_HBM7_R_FAULT,
	PWRGD_VDDPHY_HBM1_HBM3_HBM5_HBM7_R_FAULT,
	//pwr fault reg 4
	PWRGD_PLL_VDDA15_HBM0_HBM2_FAULT,
	PWRGD_PLL_VDDA15_HBM4_HBM6_FAULT,
	PWRGD_PLL_VDDA15_HBM1_HBM3_FAULT,
	PWRGD_PLL_VDDA15_HBM5_HBM7_FAULT,
	PWRGD_P0V9_OWL_E_PVDD_FAULT,
	PWRGD_P0V9_OWL_W_PVDD_FAULT,
	PWRGD_P1V5_E_RVDD_FAULT,
	PWRGD_P1V5_W_RVDD_FAULT,
	//pwr fault reg 5
	P12V_UBC_PWRGD_FAULT,
	PWRGD_P5V_R_FAULT,
	PWRGD_P3V3_R_FAULT,
	PWRGD_P1V8_R_FAULT,
	PWRGD_LDO_IN_1V2_R_FAULT,
	PWRGD_P1V5_PLL_VDDA_OWL_FAULT,
	PWRGD_P1V5_PLL_VDDA_SOC_FAULT,
	PWRGD_PVDD1P5_FAULT,
	VR_ERR_DEVICE_DONT_CARE,
};

uint16_t error_log_count(void);
void init_load_eeprom_log(void);

void plat_log_read(uint8_t *log_data, uint8_t cmd_size, uint16_t order);
void error_log_event(uint16_t error_code, bool log_status);
uint8_t plat_log_get_num(void);
void plat_clear_log();
void reset_error_log_event(uint8_t err_type);

typedef struct __attribute__((packed)) _plat_err_log_mapping {
	uint16_t index;
	uint16_t err_code;
	uint64_t sys_time;
	uint8_t error_data[20];
	uint8_t cpld_dump[CPLD_REGISTER_MAX_NUM];
	uint8_t reserved[24];
} plat_err_log_mapping;

enum LOG_ERROR_TRIGGER_CAUSE {
	CPLD_UNEXPECTED_VAL_TRIGGER_CAUSE = 0b100,
	POWER_ON_SEQUENCE_TRIGGER_CAUSE = 0b001,
	AC_ON_TRIGGER_CAUSE = 0b010,
	DC_ON_TRIGGER_CAUSE = 0b011,
	TEMPERATURE_TRIGGER_CAUSE = 0b101,
	MAX_TRIGGER_CAUSE = 0b1000, //trigger cause maxium 3 bit
};

bool check_temp_status_bit(uint8_t bit_num);
#endif
