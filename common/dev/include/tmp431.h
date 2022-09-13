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

#ifndef TMP431_H
#define TMP431_H

enum TMP431_CHANNELS {
	TMP431_LOCAL_TEMPERATRUE,
	TMP431_REMOTE_TEMPERATRUE,
};

enum TMP431_REIGSTER_MAP {
	LOCAL_TEMPERATURE_HIGH_BYTE = 0x00,
	REMOTE_TEMPERATURE_HIGH_BYTE = 0x01,
	CONFIGURATION_REGISTER_1 = 0x03,
	REMOTE_TEMPERATURE_LOW_BYTE = 0x10,
	LOCAL_TEMPERATURE_LOW_BYTE = 0x15,
};

#endif
