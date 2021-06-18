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
#include "sdr.h"
#include "sensor.h"
#include "sensor_def.h"
#include "hal_i2c.h"
#include "plat_i2c.h"
#include "pal.h"

snr_cfg sensor_config[] = {
  /* number,              type,        port,     address,    offset,           access check         index,  cache,  cache_status */
  {SENSOR_NUM_TEMP_TMP75, tmp75,       i2c_bus2, tmp75_addr, tmp75_tmp_offset, pal_stby_access,         0,      0,      SNR_INIT_STATUS},
  {SENSOR_NUM_TEST      , tmp75,       i2c_bus2, tmp75_addr, tmp75_tmp_offset, pal_DC_access,           0,      0,      SNR_INIT_STATUS},
};

bool pal_stby_access(uint8_t snr_num) {
  return 1;
}

bool pal_DC_access(uint8_t snr_num) {
  return 0;
}

