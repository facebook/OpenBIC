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
enum pump_state {
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

enum check_bit_status {
	BIT_LOW = 0,
	BIT_HIGH = 1
};
bool pump_reset(uint8_t sensor_num);
uint8_t modbus_pump_setting(modbus_command_mapping *cmd);