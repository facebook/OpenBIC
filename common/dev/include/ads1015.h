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

#ifndef ADS1015_H
#define ADS1015_H

enum ADS1015_REG_MAP {
	CONVERSION_REG,
	CONFIG_REG,
	LO_THRESH_REG,
	HI_THRESH_REG,
};

enum DEVICE_OPERATION_MODE {
	CONTINUOUS_MODE,
	SINGLE_SHOT_MODE,
}; // operating mode

enum LATCH_COMPARATOR {
	DISABLE_LATCH,
	ENABLE_LATCH,
}; // matching comparator

typedef struct _ina238_init_arg {
	bool is_init;
	// user defined
	uint8_t device_operation_mode;
	uint8_t alert_latch;
} ina238_init_arg;

#endif
