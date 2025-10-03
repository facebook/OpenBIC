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

enum {
	IT_LEAK_E_0,
	IT_LEAK_E_1,
	IT_LEAK_E_2,
	IT_LEAK_E_3,
};

void it_leak_action_0();
void it_leak_action_1();
void it_leak_action_2();
void it_leak_action_3();

void emergency_button_action();
void fault_leak_action();
void deassert_all_rpu_ready_pin();
void set_all_rpu_ready_pin_normal(void);
void aalc_leak_behavior(uint8_t sensor_num);
void shutdown_save_uptime_action();