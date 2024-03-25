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

#define I3C_BUS1 0
#define I3C_BUS2 1
#define I3C_BUS3 2

#define MCTP_I3C_BIC_ADDR 0x40 //8-bit addr

#define MCTP_I3C_BMC_BUS I3C_BUS1
#define MCTP_I3C_BMC_ADDR 0x20 //8-bit addr

#define PLDM_I3C_1OU I3C_BUS2
#define PLDM_I3C_2OU I3C_BUS3

#endif
