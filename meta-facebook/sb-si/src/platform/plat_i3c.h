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

#ifndef PLAT_I3C_h
#define PLAT_I3C_h

#include "hal_i3c.h"

// map i3c bus to peripherial bus
// i3c peripheral 1 based, as used i3c index 0 in firmware.
#define I3C_BUS1 0
#define I3C_BUS2 1
#define I3C_BUS3 2
#define I3C_BUS4 3
#define I3C_BUS5 4
#define I3C_BUS6 5

#define I3C_BUS_MAX_NUM 6

#define PLAT_DEFAULT_PID 0x510

#endif
