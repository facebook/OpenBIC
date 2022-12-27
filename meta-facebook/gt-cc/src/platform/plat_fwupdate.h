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

#ifndef PLAT_FWUPDATE_H
#define PLAT_FWUPDATE_H

#include <stdbool.h>
#include <stdint.h>
#include "plat_ipmi.h"
#include "pldm_firmware_update.h"

enum { COMP_ID_BIC = GLOBAL_COMP_ID_BIC,
       COMP_ID_VR0,
       COMP_ID_VR1,
       COMP_ID_PEX0,
       COMP_ID_PEX1,
       COMP_ID_PEX2,
       COMP_ID_PEX3,
       COMP_ID_CPLD,
       COMP_ID_MAX,
};

void load_pldmupdate_comp_config(void);

#endif
