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

#ifndef EMC1413_H
#define EMC1413_H

enum EMC1413_CHANNELS {
	EMC1413_LOCAL_TEMPERATRUE,
	EMC1413_REMOTE_TEMPERATRUE_1,
	EMC1413_REMOTE_TEMPERATRUE_2,
};

enum EMC1413_REIGSTER_MAP {
	INTERNAL_DIODE_HIGH_BYTE = 0x00,
	EXTERNAL_DIODE_1_HIGH_BYTE = 0x01,
	EXTERNAL_DIODE_2_HIGH_BYTE = 0x23,
	INTERNAL_DIODE_LOW_BYTE = 0x29,
	EXTERNAL_DIODE_1_LOW_BYTE = 0x10,
	EXTERNAL_DIODE_2_LOW_BYTE = 0x24,
};

#endif
