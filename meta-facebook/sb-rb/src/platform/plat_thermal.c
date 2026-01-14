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
#include "plat_thermal.h"
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

const char *temperature_name_table[] = {
	"ASIC_MEDHA0_SENSOR0", "ASIC_MEDHA0_SENSOR1", "ASIC_OWL_W",	"ASIC_OWL_E",
	"ASIC_MEDHA1_SENSOR0", "ASIC_MEDHA1_SENSOR1", "ASIC_HAMSA_CRM", "ASIC_HAMSA_LS",
};

typedef struct temp_mapping_sensor_t {
	uint8_t index;
	uint8_t sensor_id;
	uint8_t *sensor_name;
	uint8_t last_status;
	bool is_open;
	uint8_t log_status_val; // status val for log
	uint8_t log_limit_status_val; // limit status val for log
} temp_mapping_sensor_t;

temp_mapping_sensor_t temp_alert_index_table[] = {
	{ TEMP_STATUS_INDEX_ASIC_MEDHA0_SENSOR0, SENSOR_NUM_ASIC_MEDHA0_SENSOR0_TEMP_C,
	  "SB_RB_ASIC_MEDHA0_SENSOR0_TEMP", 0, false },
	{ TEMP_STATUS_INDEX_ASIC_MEDHA0_SENSOR1, SENSOR_NUM_ASIC_MEDHA0_SENSOR1_TEMP_C,
	  "SB_RB_ASIC_MEDHA0_SENSOR1_TEMP", 0, false },
	{ TEMP_STATUS_INDEX_ASIC_OWL_W, SENSOR_NUM_ASIC_OWL_W_TEMP_C, "SB_RB_ASIC_OWL_W_TEMP", 0,
	  false },
	{ TEMP_STATUS_INDEX_ASIC_OWL_E, SENSOR_NUM_ASIC_OWL_E_TEMP_C, "SB_RB_ASIC_OWL_E_TEMP", 0,
	  false },
	{ TEMP_STATUS_INDEX_ASIC_MEDHA1_SENSOR0, SENSOR_NUM_ASIC_MEDHA1_SENSOR0_TEMP_C,
	  "SB_RB_ASIC_MEDHA1_SENSOR0_TEMP", 0, false },
	{ TEMP_STATUS_INDEX_ASIC_MEDHA1_SENSOR1, SENSOR_NUM_ASIC_MEDHA1_SENSOR1_TEMP_C,
	  "SB_RB_ASIC_MEDHA1_SENSOR1_TEMP", 0, false },
	{ TEMP_STATUS_INDEX_ASIC_HAMSA_CRM, SENSOR_NUM_ASIC_HAMSA_CRM_TEMP_C,
	  "SB_RB_ASIC_HAMSA_CRM_TEMP", 0, false },
	{ TEMP_STATUS_INDEX_ASIC_HAMSA_LS, SENSOR_NUM_ASIC_HAMSA_LS_TEMP_C,
	  "SB_RB_ASIC_HAMSA_LS_TEMP", 0, false },
};

void read_temp_status(uint8_t bus, uint8_t target_addr)
{
	uint8_t clear_status_data[1];
	LOG_DBG("bus is %d, target_addr is 0x%x", bus, target_addr);
	plat_i2c_read(bus, target_addr, TMP432_HIGH_LIMIT_STATUS_REG, clear_status_data, 1);
	LOG_DBG("temp status is 0x%x", clear_status_data[0]);
}

uint8_t get_thermal_status_val_for_log(uint8_t sensor_num)
{
	for (uint8_t i = 0; i < ARRAY_SIZE(temp_alert_index_table); i++) {
		if (temp_alert_index_table[i].sensor_id == sensor_num)
			return temp_alert_index_table[i].log_status_val;
	}

	return 0;
}

uint8_t get_thermal_limit_status_val_for_log(uint8_t sensor_num)
{
	for (uint8_t i = 0; i < ARRAY_SIZE(temp_alert_index_table); i++) {
		if (temp_alert_index_table[i].sensor_id == sensor_num)
			return temp_alert_index_table[i].log_limit_status_val;
	}

	return 0;
}

void check_thermal_handler(void *arg1, void *arg2, void *arg3)
{
	k_sleep(K_MSEC(2000)); // wait sensor thread ready
	LOG_INF("check_thermal_handler start");

	while (1) {
		handler_flag = get_plat_sensor_polling_enable_flag();
		if (!handler_flag) {
			k_sleep(K_MSEC(1000));
			continue;
		}

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

			// check status open
			if (status_data & TEMP_STATUS_OPEN) {
				// check sensor
				if (temp_cfg->cache_status == PLDM_SENSOR_OPEN_CIRCUIT) {
					if (!temp_alert_index_table[i].is_open) {
						temp_alert_index_table[i].log_status_val =
							status_data;
						plat_set_iris_temp_error_log(
							LOG_ASSERT,
							temp_alert_index_table[i].sensor_id);
						temp_alert_index_table[i].is_open = true;
					}
				}
			}

			// if status BIT(3), BIT(4) is high than send error log;
			if (status_data & TEMP_LIMIT_STATUS) {
				uint8_t remote_bit =
					(temp_cfg->offset == TMP432_REMOTE_TEMPERATRUE_1) ? BIT(1) :
					(temp_cfg->offset == TMP432_REMOTE_TEMPERATRUE_2) ? BIT(2) :
											    0;
				uint8_t limit_status_reg =
					(status_data & TEMP_STATUS_H_LIMIT) ? H_LIMIT_STATUS :
					(status_data & TEMP_STATUS_L_LIMIT) ? L_LIMIT_STATUS :
									      0xFF;
				uint8_t limit_status_val = 0;
				if (!get_raw_data_from_sensor_id(
					    temp_alert_index_table[i].sensor_id, limit_status_reg,
					    &limit_status_val, 1) ||
				    (limit_status_reg == 0xFF))
					LOG_ERR("sensor_num 0x02%x get limit status reg 0x%02x fail",
						temp_alert_index_table[i].sensor_id,
						limit_status_reg);
				// check the corresponding remote port
				if (limit_status_val & remote_bit) {
					//check if still high, don't send error log again
					if (temp_alert_index_table[i].last_status == 1) {
						LOG_DBG("keep error sensor_num 0x%x, bus 0x%x, address 0x%x ",
							temp_cfg->num, temp_cfg->port,
							temp_cfg->target_addr);
						// clear temperature status
						read_temp_status(temp_cfg->port,
								 temp_cfg->target_addr);
						continue;
					} else {
						temp_alert_index_table[i].log_status_val =
							status_data;
						temp_alert_index_table[i].log_limit_status_val =
							limit_status_val;
						plat_set_iris_temp_error_log(
							LOG_ASSERT,
							temp_alert_index_table[i].sensor_id);
						// if last status is 0, set last status to 1
						if (!temp_alert_index_table[i].last_status) {
							temp_alert_index_table[i].last_status = 1;
						}
					}
				}
			} else {
				// if last status is 1, set last status to 0
				if (temp_alert_index_table[i].last_status) {
					temp_alert_index_table[i].last_status = 0;
					temp_alert_index_table[i].log_status_val = 0;
					plat_set_iris_temp_error_log(
						LOG_DEASSERT, temp_alert_index_table[i].sensor_id);
					LOG_INF("temperature sensor recovered: 0x%x",
						temp_alert_index_table[i].sensor_id);
				}
			}
			// clear temperature status
			LOG_DBG("end clear sensor_num 0x%x, bus 0x%x, address 0x%x ", temp_cfg->num,
				temp_cfg->port, temp_cfg->target_addr);
			read_temp_status(temp_cfg->port, temp_cfg->target_addr);
		}
		k_sleep(K_MSEC(1000));
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
