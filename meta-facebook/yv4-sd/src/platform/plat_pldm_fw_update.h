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

void load_pldmupdate_comp_config(void);
int load_mctp_support_types(uint8_t *type_len, uint8_t *types);

#endif /* _PLAT_FWUPDATE_H_ */
