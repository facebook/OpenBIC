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

#ifndef HAL_I2C_H
#define HAL_I2C_H

#include "board_device.h"
#include "objects.h"
#include "i2c_aspeed.h"

extern const uint8_t i2c_bus_to_index[];
extern i2c_t i2c[];

#define ASPEED_I2C_DMA_SIZE 40
#define MAX_I2C_BUS_NUM 16
#define DEBUG_I2C 0

typedef struct _I2C_MSG_ {
  uint8_t slave_addr;
  uint8_t bus;
  uint8_t rx_len;
  uint8_t tx_len;
  uint8_t data[ASPEED_I2C_DMA_SIZE];
} I2C_MSG;

bool i2c_master_read(I2C_MSG *msg, uint8_t retry);
bool i2c_master_write(I2C_MSG *msg, uint8_t retry);
void util_init_I2C(void);

#endif
