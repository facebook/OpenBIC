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

#ifndef PLAT_THRESHOLD_H
#define PLAT_THRESHOLD_H

#define TWO_BYTES_SENSOR_STATUS 0xFF

typedef struct {
	uint8_t sensor_num;
	uint8_t type;
	float lcr;
	float ucr;
	void (*fn)(uint8_t, uint8_t); // para: arg, status
	uint8_t arg0;

	// priv data
	uint8_t last_status; // record the last status
} sensor_threshold;

enum THRESHOLD_STATUS {
	THRESHOLD_STATUS_NORMAL,
	THRESHOLD_STATUS_LCR,
	THRESHOLD_STATUS_UCR,
	THRESHOLD_STATUS_NOT_ACCESS,
	THRESHOLD_STATUS_UNKNOWN,
};

void set_threshold_poll_enable_flag(bool flag);
bool get_threshold_poll_enable_flag();
void threshold_poll_init();
void fan_pump_pwrgd();
uint8_t get_threshold_status(uint8_t sensor_num);

#endif // PLAT_THRESHOLD_H