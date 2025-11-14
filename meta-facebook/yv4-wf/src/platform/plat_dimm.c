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

K_THREAD_STACK_DEFINE(dimm_slot_info_init_stack, 2048);
struct k_thread dimm_slot_info_init_thread;

void dimm_slot_info_init_handler(void *arg1, void *arg2, void *arg3)
{
    LOG_INF("DIMM slot info initialization thread started");

    // Wait for CXL ready
    while (1) {
        bool all_cxl_ready = true;

        for (uint8_t cxl_id = 0; cxl_id < MAX_CXL_ID; cxl_id++) {
            if (!get_cxl_ready_status(cxl_id)) {
                all_cxl_ready = false;
                LOG_INF("CXL devices %d not ready...retry", cxl_id);
                break;
            }
        }

        if (all_cxl_ready) {
            LOG_INF("All CXL devices ready, initializing DIMM slot info");
            break;
        }

        k_msleep(1000);
    }

    // Init ddr slot info
    int retry = 0;
    int result;

    while (retry < 5) {
        result = vistara_init_ddr_slot_info();
        if (result > 0) {
            return;
        } else {
            LOG_ERR("Failed to initialize DIMM slot info, retry: %d", retry);
        }
        retry++;
        k_msleep(1000);
    }

	LOG_ERR("Failed to initialize DIMM slot info after all retries");
}

void create_init_ddr_slot_info_thread(void)
{
	const int MAX_RETRY = 30;
	int retry = 0;
	k_tid_t tid = NULL;

	LOG_INF("Creating DIMM init thread...");
	
	while (retry < MAX_RETRY) {
		retry++;
		LOG_INF("Thread creation attempt %d/%d", retry, MAX_RETRY);
		
		tid = k_thread_create(&dimm_slot_info_init_thread, 
		                      dimm_slot_info_init_stack,
		                      K_THREAD_STACK_SIZEOF(dimm_slot_info_init_stack),
		                      dimm_slot_info_init_handler, 
		                      NULL, NULL, NULL,
		                      K_PRIO_PREEMPT(10), 0, K_NO_WAIT);
		
		if (tid != NULL) {
			k_thread_name_set(&dimm_slot_info_init_thread, "dimm_slot_info_init");
			LOG_INF("DIMM init thread created successfully");
			return;
		} 
		
		if (retry < MAX_RETRY) {
			LOG_INF("init_ddr_slot_info_thread creation failed, retrying in 1 second...");
			k_msleep(1000);
			memset(&dimm_slot_info_init_thread, 0, sizeof(struct k_thread));
		}
	}
	
	LOG_ERR("FAILED to create DIMM init thread after %d attempts", MAX_RETRY);
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