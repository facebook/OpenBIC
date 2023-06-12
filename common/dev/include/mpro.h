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

#ifndef MPRO_H
#define MPRO_H

#include "plat_def.h"
#ifdef ENABLE_MPRO

#include <stdint.h>
#include "pldm_monitor.h"

#define MAX_MPRO_DIMM_NUM 16

enum mpro_sensor_num_table {
	/* DIMM sensor 1 ~ 70 */
	MPRO_SENSOR_NUM_PWR_DRAM = 1, // total power consumed by all DRAMs

	MPRO_SENSOR_NUM_TMP_DIMM0 = 3,
	MPRO_SENSOR_NUM_STA_DIMM0,
	MPRO_SENSOR_NUM_TMP_DIMM1,
	MPRO_SENSOR_NUM_STA_DIMM1,
	MPRO_SENSOR_NUM_TMP_DIMM2,
	MPRO_SENSOR_NUM_STA_DIMM2,
	MPRO_SENSOR_NUM_TMP_DIMM3,
	MPRO_SENSOR_NUM_STA_DIMM3,
	MPRO_SENSOR_NUM_TMP_DIMM4,
	MPRO_SENSOR_NUM_STA_DIMM4,
	MPRO_SENSOR_NUM_TMP_DIMM5,
	MPRO_SENSOR_NUM_STA_DIMM5,
	MPRO_SENSOR_NUM_TMP_DIMM6,
	MPRO_SENSOR_NUM_STA_DIMM6,
	MPRO_SENSOR_NUM_TMP_DIMM7,
	MPRO_SENSOR_NUM_STA_DIMM7,
	MPRO_SENSOR_NUM_TMP_DIMM8,
	MPRO_SENSOR_NUM_STA_DIMM8,
	MPRO_SENSOR_NUM_TMP_DIMM9,
	MPRO_SENSOR_NUM_STA_DIMM9,
	MPRO_SENSOR_NUM_TMP_DIMM10,
	MPRO_SENSOR_NUM_STA_DIMM10,
	MPRO_SENSOR_NUM_TMP_DIMM11,
	MPRO_SENSOR_NUM_STA_DIMM11,
	MPRO_SENSOR_NUM_TMP_DIMM12,
	MPRO_SENSOR_NUM_STA_DIMM12,
	MPRO_SENSOR_NUM_TMP_DIMM13,
	MPRO_SENSOR_NUM_STA_DIMM13,
	MPRO_SENSOR_NUM_TMP_DIMM14,
	MPRO_SENSOR_NUM_STA_DIMM14,
	MPRO_SENSOR_NUM_TMP_DIMM15,
	MPRO_SENSOR_NUM_STA_DIMM15,

	MPRO_SENSOR_NUM_STA_DDR = 51, // DDR initialization status at the system level

	/* VRD sensor 71 ~ 140 */
	MPRO_SENSOR_NUM_TMP_VRD0 = 71,
	MPRO_SENSOR_NUM_PWR_VRD0,
	MPRO_SENSOR_NUM_VOL_VRD0,
	MPRO_SENSOR_NUM_CUR_VRD0,
	MPRO_SENSOR_NUM_STA_VRD0,

	MPRO_SENSOR_NUM_TMP_VRD1,
	MPRO_SENSOR_NUM_PWR_VRD1,
	MPRO_SENSOR_NUM_VOL_VRD1,
	MPRO_SENSOR_NUM_CUR_VRD1,
	MPRO_SENSOR_NUM_STA_VRD1,

	MPRO_SENSOR_NUM_TMP_VRD2,
	MPRO_SENSOR_NUM_PWR_VRD2,
	MPRO_SENSOR_NUM_VOL_VRD2,
	MPRO_SENSOR_NUM_CUR_VRD2,
	MPRO_SENSOR_NUM_STA_VRD2,

	MPRO_SENSOR_NUM_TMP_VRD3,
	MPRO_SENSOR_NUM_PWR_VRD3,
	MPRO_SENSOR_NUM_VOL_VRD3,
	MPRO_SENSOR_NUM_CUR_VRD3,
	MPRO_SENSOR_NUM_STA_VRD3,

	MPRO_SENSOR_NUM_TMP_VRD4,
	MPRO_SENSOR_NUM_PWR_VRD4,
	MPRO_SENSOR_NUM_VOL_VRD4,
	MPRO_SENSOR_NUM_CUR_VRD4,
	MPRO_SENSOR_NUM_STA_VRD4,

	MPRO_SENSOR_NUM_TMP_VRD5,
	MPRO_SENSOR_NUM_PWR_VRD5,
	MPRO_SENSOR_NUM_VOL_VRD5,
	MPRO_SENSOR_NUM_CUR_VRD5,
	MPRO_SENSOR_NUM_STA_VRD5,

	MPRO_SENSOR_NUM_TMP_VRD6,
	MPRO_SENSOR_NUM_PWR_VRD6,
	MPRO_SENSOR_NUM_VOL_VRD6,
	MPRO_SENSOR_NUM_CUR_VRD6,
	MPRO_SENSOR_NUM_STA_VRD6,

	MPRO_SENSOR_NUM_TMP_VRD7,
	MPRO_SENSOR_NUM_PWR_VRD7,
	MPRO_SENSOR_NUM_VOL_VRD7,
	MPRO_SENSOR_NUM_CUR_VRD7,
	MPRO_SENSOR_NUM_STA_VRD7,

	MPRO_SENSOR_NUM_TMP_VRD8,
	MPRO_SENSOR_NUM_PWR_VRD8,
	MPRO_SENSOR_NUM_VOL_VRD8,
	MPRO_SENSOR_NUM_CUR_VRD8,
	MPRO_SENSOR_NUM_STA_VRD8,

	/* SOC sensor 151 ~ 170 */
	MPRO_SENSOR_NUM_PWR_SOC_PKG = 151,
	MPRO_SENSOR_NUM_TMP_SOC_PKG = 153,

	/* BOOT sensor 171 ~ 190 */
	MPRO_SENSOR_NUM_STA_OVERALL_BOOT = 175,

	/* RAS sensor 191 ~ 210 */

	/* reserved 211 ~ 249 */

	/* MISC sensor 250 ~ 300 */
	MPRO_EFFECTER_NUM_I2C_PROXY_CTRL = 250, // Support after CPU PV2
	MPRO_EFFECTER_NUM_I2C_CNT = 251,
	MPRO_EFFECTER_NUM_I2C_PROXY_DATA = 252,
};

/* Y = (mX + b) * 10^r */
struct plat_mpro_sensor_mapping {
	uint8_t sensor_num;
	uint16_t mpro_sensor_num;
	pldm_sensor_pdr_parm cal_parm;
};

extern struct plat_mpro_sensor_mapping mpro_sensor_map[];
extern const int MPRO_MAP_TAB_SIZE;

uint16_t copy_mpro_read_buffer(uint16_t start, uint16_t length, uint8_t *buffer,
			       uint16_t buffer_len);
void mpro_postcode_read_init();
void mpro_postcode_insert(uint32_t postcode);
void reset_mpro_postcode_buffer();
bool get_4byte_postcode_ok();
void reset_4byte_postcode_ok();

#endif /* ENABLE_MPRO */

#endif /* MPRO_H */
