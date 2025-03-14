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

#ifndef PLAT_I2C_SLAVE_H
#define PLAT_I2C_SLAVE_H

#include <drivers/i2c.h>
#include "hal_i2c_target.h"

#define TARGET_ENABLE 1
#define TARGET_DISABLE 0

void update_sensor_data_0_2_table(void);
void sensor_data_table_init(void);

#endif
