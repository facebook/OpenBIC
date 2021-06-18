/*
 * Copyright (c) Facebook, Inc. and its affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include "objects.h"
#include "fru.h"
#include "plat_fru.h"

#define MB_FRU_PORT 0x00
#define MB_FRU_ADDR 0x50

const EEPROM_CFG fru_config[] = {
  {
    NV_ATMEL_24C128,
    MB_FRU_ID,
    MB_FRU_PORT,
    MB_FRU_ADDR,
    FRU_DEV_ACCESS_BYTE,
    FRU_START,
    FRU_SIZE,
  },
};
