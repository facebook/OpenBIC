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

bool clear_log_for_modbus_pump_setting(pump_reset_struct *data, uint8_t bit_val);
bool pump_reset(pump_reset_struct *data, uint8_t bit_val);
bool modbus_pump_setting_unsupport_function(pump_reset_struct *data, uint8_t bit_val);
