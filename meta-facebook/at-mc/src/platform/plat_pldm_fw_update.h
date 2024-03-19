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

#ifndef _PLAT_FWUPDATE_H_
#define _PLAT_FWUPDATE_H_

#include <stdbool.h>
#include <stdint.h>
#include "pldm_firmware_update.h"

#define PLDM_FW_UPDATE_SUCCESS 0
#define PLDM_FW_UPDATE_ERROR 1

void load_pldmupdate_comp_config(void);
void clear_pending_version(uint8_t activate_method);

#endif /* _PLAT_FWUPDATE_H_ */
