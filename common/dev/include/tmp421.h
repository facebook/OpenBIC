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

#ifndef TMP421_H
#define TMP421_H

enum TMP421_TEMP_RANGE {
	TMP421_RANGE_127 = 0x0, // -55~127°C
	TMP421_RANGE_150 = 0x1, // -55~150°C
};

enum TMP421_CHANNELS {
	TMP421_LOCAL_TEMPERATRUE,
	TMP421_REMOTE_TEMPERATRUE_1,
};

enum TMP421_REIGSTER_OFFSET {
	TMP421_OFFSET_LOCAL_TEMPERATURE_HIGH_BYTE = 0x00,
	TMP421_OFFSET_REMOTE_TEMPERATURE_HIGH_BYTE_1 = 0x01,
	TMP421_OFFSET_CONFIGURATION_1 = 0x09,
	TMP421_OFFSET_LOCAL_TEMPERATURE_LOW_BYTE = 0x10,
	TMP421_OFFSET_REMOTE_TEMPERATURE_LOW_BYTE_1 = 0x11,

};

#endif
