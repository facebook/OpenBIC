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

#ifndef PLAT_PLDM_SENSOR_H
#define PLAT_PLDM_SENSOR_H

#include "pdr.h"

#ifdef ENABLE_PLATFORM_PROVIDES_PLDM_SENSOR_STACKS
#define ADC_SENSOR_STACK_SIZE 640
#define VR_SENSOR_STACK_SIZE 1344
#define MB_TEMP_SENSOR_STACK_SIZE 1152
#define CPU_SENSOR_STACK_SIZE 1024
#define INA233_SENSOR_STACK_SIZE 1024
#define DIMM_SENSOR_STACK_SIZE 448
#endif

#define ADDR_TMP75_INLET (0x92 >> 1)
#define ADDR_TMP75_OUTLET (0x98 >> 1)
#define ADDR_TMP75_FIO (0x90 >> 1)
#define ADDR_VR_CPU0 (0XEC >> 1)
#define ADDR_VR_SOC (0XEC >> 1)
#define ADDR_VR_CPU1 (0XC6 >> 1)
#define ADDR_VR_PVDDIO (0XC6 >> 1)
#define ADDR_VR_PVDD11 (0XE4 >> 1)

#define ADDR_X8_INA233 (0x8A >> 1)
#define ADDR_X16_INA233 (0x82 >> 1)
#define ADDR_E1S_BOOT_INA233 (0x8A >> 1)
#define ADDR_E1S_DATA_INA233 (0x80 >> 1)
#define ADDR_X8_RETIMER (0x46 >> 1)
#define ADDR_X16_RETIMER (0x40 >> 1)
#define ADDR_NVME (0xD4 >> 1)

#define OFFSET_TMP75_TEMP 0x00
#define OFFSET_NVME_TEMP 0x00
#define OFFSET_CARD_PRSNT 0x04

#define NUM_SOC_PACKAGE_PWR 0x0055

#define UPDATE_INTERVAL_1S 1
#define UPDATE_INTERVAL_3S 3
#define UPDATA_INTERNAL_1HR 3600

#define VR_DEVICE_UNKNOWN 0xFF

#define SENSOR_NUM_MB_INA233_E1S_DATA_VOLT_V 0x0033
#define SENSOR_NUM_MB_INA233_X8_RTM_CURR_A 0x0045
#define SENSOR_NUM_MB_INA233_E1S_DATA_CURR_A 0x0048
#define SENSOR_NUM_MB_INA233_X8_RTM_PWR_W 0x0062
#define SENSOR_NUM_MB_INA233_E1S_DATA_PWR_W 0x0065

enum SENSOR_THREAD_LIST {
	ADC_SENSOR_THREAD_ID = 0,
	VR_SENSOR_THREAD_ID,
	MB_TEMP_SENSOR_THREAD_ID,
	CPU_SENSOR_THREAD_ID,
	INA233_SENSOR_THREAD_ID,
	DIMM_SENSOR_THREAD_ID,
	MAX_SENSOR_THREAD_ID,
};

enum GET_VR_DEV_STATUS {
	GET_VR_DEV_SUCCESS = 0,
	GET_VR_DEV_FAILED,
};

int plat_pldm_sensor_get_sensor_count(int thread_id);
uint8_t plat_pldm_sensor_get_vr_dev(uint8_t *vr_dev);
uint8_t plat_pldm_sensor_get_ina_dev();
void plat_pldm_sensor_change_vr_dev();
void plat_pldm_sensor_change_ssd_dev();
void plat_pldm_sensor_change_cpu_bus();
void plat_pldm_sensor_change_retimer_dev();
void plat_pldm_sensor_change_ina_dev();
void plat_init_pldm_sensor_table();
void plat_init_pldm_disabled_sensors();
void plat_pldm_sensor_change_dimm_dev();
void plat_pldm_sensor_clear_vr_fault(uint8_t vr_addr, uint8_t vr_bus, uint8_t page_cnt);
bool bootdrive_access(uint8_t sensor_num);
void set_bootdrive_exist_status();
bool get_bootdrive_exist_status();

#define MONITOR_PROCHOT_SENSOR_STACK_SIZE 1024
void start_monitor_prochot_sensor_thread();

#define PROCHOT_SENSOR_TABLE_LEN 18
typedef struct _prochot_sensor_info {
	uint16_t sensor_id;
	uint16_t event_bit;
} prochot_sensor_info;

#define MONITOR_PROCHOT_SENSOR_TIME_MS (1 * 1000) // 1s
void monitor_prochot_sensor_handler();

#endif
