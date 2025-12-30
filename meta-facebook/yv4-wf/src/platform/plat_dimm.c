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
#include <zephyr.h>
#include <logging/log.h>
#include <errno.h>

#include "sensor.h"
#include "power_status.h"

#include "plat_dimm.h"

LOG_MODULE_REGISTER(plat_dimm);

#ifdef ENABLE_VISTARA
#include "vistara.h"
#include "plat_power_seq.h"

K_THREAD_STACK_DEFINE(dimm_cxl1_init_stack, 2048);
struct k_thread dimm_cxl1_init_thread;
static k_tid_t dimm_cxl1_init_tid = NULL;

K_THREAD_STACK_DEFINE(dimm_cxl2_init_stack, 2048);
struct k_thread dimm_cxl2_init_thread;
static k_tid_t dimm_cxl2_init_tid = NULL;

void dimm_cxl1_slot_info_init_handler()
{
	LOG_INF("DIMM slot info initialization for CXL1 started");

	int retry = 0;
	while (retry < 5) {
		int result = vistara_init_ddr_slot_info(CXL_ID_1);
		if (result > 0) {
			LOG_INF("CXL1 DIMM slot info initialized successfully");
			return;
		} else {
			LOG_ERR("Failed to initialize CXL1 DIMM slot info, retry: %d", retry);
		}
		retry++;
		k_msleep(1000);
	}

	LOG_ERR("Failed to initialize CXL1 DIMM slot info after all retries");
}

void dimm_cxl2_slot_info_init_handler()
{
	LOG_INF("DIMM slot info initialization for CXL2 started");

	int retry = 0;
	while (retry < 5) {
		int result = vistara_init_ddr_slot_info(CXL_ID_2);
		if (result > 0) {
			LOG_INF("CXL2 DIMM slot info initialized successfully");
			return;
		} else {
			LOG_ERR("Failed to initialize CXL2 DIMM slot info, retry: %d", retry);
		}
		retry++;
		k_msleep(1000);
	}

	LOG_ERR("Failed to initialize CXL2 DIMM slot info after all retries");
}

void create_init_ddr_slot_info_thread(uint8_t cxl_id)
{
	if (cxl_id == CXL_ID_1) {
		if ((dimm_cxl1_init_tid != NULL) &&
		    ((strcmp(k_thread_state_str(dimm_cxl1_init_tid), "dead") != 0) &&
		     (strcmp(k_thread_state_str(dimm_cxl1_init_tid), "unknown") != 0))) {
			k_thread_abort(dimm_cxl1_init_tid);
		}

		dimm_cxl1_init_tid =
			k_thread_create(&dimm_cxl1_init_thread, dimm_cxl1_init_stack,
					K_THREAD_STACK_SIZEOF(dimm_cxl1_init_stack),
					dimm_cxl1_slot_info_init_handler, NULL, NULL, NULL,
					CONFIG_MAIN_THREAD_PRIORITY + 1, 0, K_NO_WAIT);
		k_thread_name_set(dimm_cxl1_init_tid, "dimm_cxl1_init_thread");
		k_thread_start(dimm_cxl1_init_tid);
	} else if (cxl_id == CXL_ID_2) {
		if ((dimm_cxl2_init_tid != NULL) &&
		    ((strcmp(k_thread_state_str(dimm_cxl2_init_tid), "dead") != 0) &&
		     (strcmp(k_thread_state_str(dimm_cxl2_init_tid), "unknown") != 0))) {
			k_thread_abort(dimm_cxl2_init_tid);
		}

		dimm_cxl2_init_tid =
			k_thread_create(&dimm_cxl2_init_thread, dimm_cxl2_init_stack,
					K_THREAD_STACK_SIZEOF(dimm_cxl2_init_stack),
					dimm_cxl2_slot_info_init_handler, NULL, NULL, NULL,
					CONFIG_MAIN_THREAD_PRIORITY + 1, 0, K_NO_WAIT);
		k_thread_name_set(dimm_cxl2_init_tid, "dimm_cxl2_init_thread");
		k_thread_start(dimm_cxl2_init_tid);
	} else {
		LOG_ERR("Invalid CXL ID: %d", cxl_id);
	}
}

uint8_t sensor_num_map_dimm_id(uint8_t sensor_num)
{
	uint8_t dimm_id = DIMM_ID_UNKNOWN;

	switch (sensor_num) {
	case SENSOR_NUM_ASIC1_DIMM_A_TEMP:
		dimm_id = ASIC1_DIMM_ID_A;
		break;
	case SENSOR_NUM_ASIC1_DIMM_B_TEMP:
		dimm_id = ASIC1_DIMM_ID_B;
		break;
	case SENSOR_NUM_ASIC1_DIMM_C_TEMP:
		dimm_id = ASIC1_DIMM_ID_C;
		break;
	case SENSOR_NUM_ASIC1_DIMM_D_TEMP:
		dimm_id = ASIC1_DIMM_ID_D;
		break;
	case SENSOR_NUM_ASIC2_DIMM_A_TEMP:
		dimm_id = ASIC2_DIMM_ID_A;
		break;
	case SENSOR_NUM_ASIC2_DIMM_B_TEMP:
		dimm_id = ASIC2_DIMM_ID_B;
		break;
	case SENSOR_NUM_ASIC2_DIMM_C_TEMP:
		dimm_id = ASIC2_DIMM_ID_C;
		break;
	case SENSOR_NUM_ASIC2_DIMM_D_TEMP:
		dimm_id = ASIC2_DIMM_ID_D;
		break;
	default:
		LOG_ERR("Unknown sensor number 0x%02x for DIMM mapping", sensor_num);
		break;
	}
	return dimm_id;
}

uint8_t get_dimm_present(uint8_t dimm_id)
{
	if (dimm_id == DIMM_ID_UNKNOWN) {
		LOG_ERR("Failed to get DIMM's present. DIMM ID: 0x%02x", dimm_id);
		return DIMM_NOT_PRSNT;
	}

	// get dimm present status from cache
	bool present = vistara_get_dimm_present_from_cache(dimm_id);
	uint8_t present_status = present ? DIMM_PRSNT : DIMM_NOT_PRSNT;

	return present_status;
}

#endif