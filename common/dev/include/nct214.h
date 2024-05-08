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

#ifndef NCT214_H
#define NCT214_H

#define LOCAL_TEMP_REG 0x00
#define EXTERNAL_TEMP_UPPER_BYTE_REG 0x01
#define EXTERNAL_TEMP_LOWER_BYTE_REG 0x10
#define CONFIG_READ_REG 0x03
#define CONFIG_WRITE_REG 0x09

typedef struct _nct214_init_arg {
	bool is_init;
	uint8_t temperature_range;
} nct214_init_arg;

enum NCT214_CHANNELS {
	NCT214_LOCAL_TEMPERATRUE,
	NCT214_REMOTE_TEMPERATRUE,
};

enum NCT_214_TEMPERATURE_RANGE {
	NCT_214_TEMPERATURE_RANGE_0_TO_127,
	NCT_214_TEMPERATURE_RANGE_EXTENDED, // −64°C to +191°C
};
#endif
