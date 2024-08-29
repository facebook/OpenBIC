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

#ifndef NVIDIA_H
#define NVIDIA_H

#include "plat_def.h"
#ifdef ENABLE_NVIDIA

#include <stdint.h>
#include "pldm_monitor.h"

enum nv_satmc_sensor_num_table {
	/* numeric sensor */
	NV_SATMC_SENSOR_NUM_PWR_VDD_CPU = 0x0010,
	NV_SATMC_SENSOR_NUM_VOL_VDD_CPU,
	NV_SATMC_SENSOR_NUM_PWR_VDD_SOC = 0x0020,
	NV_SATMC_SENSOR_NUM_VOL_VDD_SOC,
	NV_SATMC_SENSOR_NUM_PWR_MODULE = 0x0030,
	NV_SATMC_SENSOR_NUM_ENG_MODULE,
	NV_SATMC_SENSOR_NUM_PWR_GRACE = 0x0040,
	NV_SATMC_SENSOR_NUM_ENG_GRACE,
	NV_SATMC_SENSOR_NUM_PWR_TOTAL_MODULE,
	NV_SATMC_SENSOR_NUM_ENG_TOTAL_MODULE,
	NV_SATMC_SENSOR_NUM_CNT_PAGE_RETIRE = 0x0050,
	NV_SATMC_SENSOR_NUM_TMP_GRACE = 0x00A0,
	NV_SATMC_SENSOR_NUM_TMP_GRACE_LIMIT,
	NV_SATMC_SENSOR_NUM_FRQ_MEMORY = 0x00B0,
	NV_SATMC_SENSOR_NUM_FRQ_MAX_CPU = 0x00C0,

	/* state sensor */
	NV_SATMC_SENSOR_NUM_CPU_THROT_STATE = 0x0210,
	NV_SATMC_SENSOR_NUM_POWER_BREAK = 0x0240,
	NV_SATMC_SENSOR_NUM_SPARE_CH_PRESENCE = 0x0250,
};

#ifdef ENABLE_SBMR
#define NV_STATUS_CODE_TYPE_PROG_CODE 0x1
#define NV_STATUS_CODE_TYPE_ERROR_CODE 0x2
#define NV_STATUS_CODE_TYPE_DEBUG_CODE 0x3
#endif

/* Y = (mX + b) * 10^r */
struct nv_satmc_sensor_parm {
	uint16_t nv_satmc_sensor_id;
	pldm_sensor_pdr_parm cal_parm;
};

extern struct nv_satmc_sensor_parm satmc_sensor_cfg_list[];
extern const int SATMC_SENSOR_CFG_LIST_SIZE;

#endif /* ENABLE_NVIDIA */
#endif /* NVIDIA_H */
