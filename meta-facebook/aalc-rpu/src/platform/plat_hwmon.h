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

enum PUMP_STATE {
	PUMP_REDUNDENT_SWITCHED = 0,
	MANUAL_CONTROL_PUMP = 1,
	MANUAL_CONTROL_FAN = 2,
	AUTOTUNE_FLOW_CONTROL = 3,
	AUTOTUNE_PRESSURE_BALANCE_CONTROL = 4,
	SYSTEM_STOP = 5,
	RPU_REMOTE_POWER_CYCLE = 6,
	MANUAL_CONTROL = 9,
	CLEAR_PUMP_RUNNING_TIME = 10,
	CLEAR_LOG = 11,
	PUMP_1_RESET = 12,
	PUMP_2_RESET = 13,
	PUMP_3_RESET = 14,
};

typedef struct _pump_reset_struct {
	uint8_t function_index;
	bool (*fn)(struct _pump_reset_struct *, uint8_t bit_value);
	uint8_t senser_num;
} pump_reset_struct;

enum PUMP_NUM {
	PUMP_1_UPTIME,
	PUMP_2_UPTIME,
	PUMP_3_UPTIME,
	PUMP_MAX_NUM,
};

typedef struct _pump_running_time_struct {
	uint8_t pump_num;
	uint8_t size;
	uint16_t eeprom_offset;
} pump_running_time;

bool clear_log_for_modbus_pump_setting(pump_reset_struct *data, uint8_t bit_val);
bool pump_setting_pump1_reset(pump_reset_struct *data, uint8_t bit_val);
bool pump_setting_pump2_reset(pump_reset_struct *data, uint8_t bit_val);
bool pump_setting_pump3_reset(pump_reset_struct *data, uint8_t bit_val);
bool close_pump(pump_reset_struct *data, uint8_t bit_val);
bool pump_setting_set_manual_flag(pump_reset_struct *data, uint8_t bit_val);
bool pump_setting_set_auto_tune_flag(pump_reset_struct *data, uint8_t bit_val);
bool pump_setting_set_pump_redundant(pump_reset_struct *data, uint8_t bit_val);
bool modbus_pump_setting_unsupport_function(pump_reset_struct *data, uint8_t bit_val);
bool set_all_pump_power(bool switch_val);
bool rpu_remote_power_cycle_function(pump_reset_struct *data, uint8_t bit_val);
uint16_t get_pump_redundant_switch_time();
uint8_t get_pump_redundant_switch_time_type();
void set_pump_redundant_switch_time(uint16_t time);
void set_pump_redundant_switch_time_type(uint8_t type);
void pump_redundant_enable(uint8_t onoff);
uint8_t pwm_control(uint8_t group, uint8_t duty);
bool get_pump_uptime_secs(uint8_t pump_num, uint32_t *return_uptime);
bool set_pump_uptime_secs(uint8_t pump_1_set, uint8_t pump_2_set, uint8_t pump_3_set);
bool get_pump_last_switch_time(uint8_t pump_num, uint32_t *return_uptime);
bool get_pump_current_boot_unrunning_time(uint8_t pump_num, uint32_t *return_uptime);
bool modbus_clear_pump_running_time_function(pump_reset_struct *data, uint8_t bit_val);
