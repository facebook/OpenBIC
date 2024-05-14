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
	PUMP_REDUNDENT_SWITCHED,
	MANUAL_CONTROL_PUMP,
	MANUAL_CONTROL_FAN,
	AUTOTUNE_FLOW_CONTROL,
	AUTOTUNE_PRESSURE_BALANCE_CONTROL,
	SYSTEM_STOP,
	RPU_REMOTE_POWER_CYCLE,
	REVERSE_STATE_1,
	REVERSE_STATE_2,
	MANUAL_CONTROL,
	CLEAR_PUMP_RUNNING_TIME,
	CLEAR_LOG,
	PUMP_1_RESET,
	PUMP_2_RESET,
	PUMP_3_RESET,
};

bool pump_reset(uint8_t sensor_num);
uint8_t modbus_pump_setting(modbus_command_mapping *cmd);