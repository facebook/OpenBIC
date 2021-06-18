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
#include <stdbool.h>
#include "pal.h"
#include "plat_i2c.h"
#include "i2c_aspeed.h"
#include "board_device.h"
#include "hal_i2c.h"

const uint8_t i2c_bus_to_index[] = { 0, 0, 1, 2, 3, 4, 5, 6, 7, 8 };

void pal_I2C_init(void) {
  // driver define i2c index should start from 0

  i2c[0].device = &i2c0;
  i2c[1].device = &i2c1;
  i2c[2].device = &i2c2;
  i2c[3].device = &i2c3;
  i2c[4].device = &i2c4;
  i2c[5].device = &i2c5;
  i2c[6].device = &i2c6;
  i2c[7].device = &i2c7;
  i2c[8].device = &i2c8;
  i2c[9].device = &i2c9;

  i2c_init(&i2c[0]);
  i2c_init(&i2c[1]);
  i2c_init(&i2c[2]);
  i2c_init(&i2c[3]);
  i2c_init(&i2c[4]);
  i2c_init(&i2c[5]);
  i2c_init(&i2c[6]);
  i2c_init(&i2c[7]);
  i2c_init(&i2c[8]);
  i2c_init(&i2c[9]);

  return;
}


