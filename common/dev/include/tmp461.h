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

#ifndef TMP461_H
#define TMP461_H

enum TMP461_TEMP_RANGE {
	TEMP_RANGE_M40_127 = 0x0, // -40~127°C
	TEMP_RANGE_M64_191 = 0x1, // -64~191°C
	TEMP_RANGE_NO_INIT = 0xFF,
};

enum TMP461_CHANNELS {
	TMP461_LOCAL_TEMPERATRUE,
	TMP461_REMOTE_TEMPERATRUE,
};

enum TMP461_REIGSTER_OFFSET {
	OFFSET_LOCAL_TEMPERATURE_HIGH_BYTE = 0x00,
	OFFSET_REMOTE_TEMPERATURE_HIGH_BYTE = 0x01,
	OFFSET_CONFIGURATION = 0x03,
	OFFSET_REMOTE_TEMPERATURE_LOW_BYTE = 0x10,
	OFFSET_LOCAL_TEMPERATURE_LOW_BYTE = 0x15,
};

#endif
