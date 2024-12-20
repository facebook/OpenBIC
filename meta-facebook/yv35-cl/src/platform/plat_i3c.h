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

#ifndef PLAT_I3C_H
#define PLAT_I3C_H

#include "hal_i3c.h"

#define I3C_BUS4 3
/* i3c 8-bit addr */
#define I3C_STATIC_ADDR_BIC 0x40
#define I3C_STATIC_ADDR_BMC 0x20

/* i3c dev bus */
#define I3C_BUS_BMC 0

#define I3C_BUS0_PID 0

/* mctp endpoint */
#define MCTP_EID_BMC 0x01

#endif
