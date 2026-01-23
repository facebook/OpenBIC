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
#include "plat_event.h"
#include <logging/log.h>
#include "tmp431.h"
#include "sensor.h"
#include "plat_log.h"
#include "plat_user_setting.h"
#include "plat_util.h"
#include "plat_i2c.h"
#include "plat_pldm_sensor.h"
#include "plat_led.h"

LOG_MODULE_REGISTER(plat_thermal);

#define TMP432_HIGH_LIMIT_STATUS_REG 0x35
struct k_thread check_thermal_thread;
K_KERNEL_STACK_MEMBER(check_thermal_thread_stack, 1024);
k_tid_t thermal_tid;
bool handler_flag = true;

enum IRIS_TEMP_INDEX_E {
	TEMP_STATUS_INDEX_ASIC_MEDHA0_SENSOR0,
	TEMP_STATUS_INDEX_ASIC_MEDHA0_SENSOR1,
	TEMP_STATUS_INDEX_ASIC_OWL_W,
	TEMP_STATUS_INDEX_ASIC_OWL_E,
	TEMP_STATUS_INDEX_ASIC_MEDHA1_SENSOR0,
	TEMP_STATUS_INDEX_ASIC_MEDHA1_SENSOR1,
	TEMP_STATUS_INDEX_ASIC_HAMSA_CRM,
	TEMP_STATUS_INDEX_ASIC_HAMSA_LS,
	TEMP_STATUS_INDEX_MAX,
};

const char *temperature_name_table[] = {
	"ASIC_MEDHA0_SENSOR0", "ASIC_MEDHA0_SENSOR1", "ASIC_OWL_W",	"ASIC_OWL_E",
	"ASIC_MEDHA1_SENSOR0", "ASIC_MEDHA1_SENSOR1", "ASIC_HAMSA_CRM", "ASIC_HAMSA_LS",
};

typedef struct temp_mapping_sensor_t {
	uint8_t index;
	uint8_t sensor_id;
	uint8_t *sensor_name;
	uint8_t last_status;
} temp_mapping_sensor_t;

temp_mapping_sensor_t temp_alert_index_table[] = {
	{ TEMP_STATUS_INDEX_ASIC_MEDHA0_SENSOR0, SENSOR_NUM_ASIC_MEDHA0_SENSOR0_TEMP_C,
	  "SB_RB_ASIC_MEDHA0_SENSOR0_TEMP", 0 },
	{ TEMP_STATUS_INDEX_ASIC_MEDHA0_SENSOR1, SENSOR_NUM_ASIC_MEDHA0_SENSOR1_TEMP_C,
	  "SB_RB_ASIC_MEDHA0_SENSOR1_TEMP", 0 },
	{ TEMP_STATUS_INDEX_ASIC_OWL_W, SENSOR_NUM_ASIC_OWL_W_TEMP_C, "SB_RB_ASIC_OWL_W_TEMP", 0 },
	{ TEMP_STATUS_INDEX_ASIC_OWL_E, SENSOR_NUM_ASIC_OWL_E_TEMP_C, "SB_RB_ASIC_OWL_E_TEMP", 0 },
	{ TEMP_STATUS_INDEX_ASIC_MEDHA1_SENSOR0, SENSOR_NUM_ASIC_MEDHA1_SENSOR0_TEMP_C,
	  "SB_RB_ASIC_MEDHA1_SENSOR0_TEMP", 0 },
	{ TEMP_STATUS_INDEX_ASIC_MEDHA1_SENSOR1, SENSOR_NUM_ASIC_MEDHA1_SENSOR1_TEMP_C,
	  "SB_RB_ASIC_MEDHA1_SENSOR1_TEMP", 0 },
	{ TEMP_STATUS_INDEX_ASIC_HAMSA_CRM, SENSOR_NUM_ASIC_HAMSA_CRM_TEMP_C,
	  "SB_RB_ASIC_HAMSA_CRM_TEMP", 0 },
	{ TEMP_STATUS_INDEX_ASIC_HAMSA_LS, SENSOR_NUM_ASIC_HAMSA_LS_TEMP_C,
	  "SB_RB_ASIC_HAMSA_LS_TEMP", 0 },
};

void read_temp_status(uint8_t bus, uint8_t target_addr)
{
	uint8_t clear_status_data[1];
	LOG_DBG("bus is %d, target_addr is 0x%x", bus, target_addr);
	plat_i2c_read(bus, target_addr, TMP432_HIGH_LIMIT_STATUS_REG, clear_status_data, 1);
	LOG_DBG("temp status is 0x%x", clear_status_data[0]);
}
void check_thermal_handler(void *arg1, void *arg2, void *arg3)
{
	LOG_INF("check_thermal_handler start");

	while (1) {
		handler_flag = get_plat_sensor_polling_enable_flag();
		if (!handler_flag) {
			k_sleep(K_MSEC(1000));
			continue;
		}
		k_sleep(K_MSEC(1000));
		//check temp_alert_index_table all temperature status
		for (int i = 0; i < ARRAY_SIZE(temp_alert_index_table); i++) {
			sensor_cfg *temp_cfg =
				get_sensor_cfg_by_sensor_id(temp_alert_index_table[i].sensor_id);
			if (temp_cfg == NULL) {
				LOG_ERR("sensor id %d not found",
					temp_alert_index_table[i].sensor_id);
				continue;
			}
			uint8_t status_data;
			//idx will base on 3
			plat_get_temp_status(temp_alert_index_table[i].index + 3, &status_data);

			// if status 4-bit is high than send error log;
			if ((status_data >> 4) & 0x01) {
				//check if still high, don't send error log again
				if (temp_alert_index_table[i].last_status == 1) {
					LOG_DBG("keep error sensor_num 0x%x, bus 0x%x, address 0x%x ",
						temp_cfg->num, temp_cfg->port,
						temp_cfg->target_addr);
					// clear temperature status
					read_temp_status(temp_cfg->port, temp_cfg->target_addr);
					continue;
				} else {
					plat_set_iris_temp_error_log(
						LOG_ASSERT, temp_alert_index_table[i].sensor_id);
					// if last status is 0, set last status to 1
					if (!temp_alert_index_table[i].last_status) {
						temp_alert_index_table[i].last_status = 1;
					}
				}
			} else {
				// if last status is 1, set last status to 0
				if (temp_alert_index_table[i].last_status) {
					temp_alert_index_table[i].last_status = 0;
					LOG_INF("temperature sensor recovered: 0x%x",
						temp_alert_index_table[i].sensor_id);
				}
			}
			// clear temperature status
			LOG_DBG("end clear sensor_num 0x%x, bus 0x%x, address 0x%x ", temp_cfg->num,
				temp_cfg->port, temp_cfg->target_addr);
			read_temp_status(temp_cfg->port, temp_cfg->target_addr);
		}
	}
}

void init_thermal_polling(void)
{
	thermal_tid = k_thread_create(&check_thermal_thread, check_thermal_thread_stack,
				      K_THREAD_STACK_SIZEOF(check_thermal_thread_stack),
				      check_thermal_handler, NULL, NULL, NULL,
				      CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&check_thermal_thread, "thermal_checking_thread");
}