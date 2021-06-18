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
#include "i2c_aspeed.h"
#include "cmsis_os2.h"
#include "board_device.h"
#include "objects.h"
#include "hal_i2c.h"
#include "timer.h"

#define I2C_mutex_timeout util_get_ms_tick(10000)
#define ASPEED_I2C_DMA_SIZE 40
extern i2c_t i2c[];

static osMutexId_t util_i2c_mutex[MAX_I2C_BUS_NUM]; // mutex for each I2C bus access protection
const osMutexAttr_t util_i2c_Mutex_attr = {
  "utilI2CQueueMutex",                   // human readable mutex name
  osMutexPrioInherit,    // attr_bits, osMutexRobust: unlock mutex while thread terminated, osMutexPrioInherit: adjuct thread priority to avoid dead lock
  NULL,                                  // memory for control block
  0U,                                    // size for control block
};

bool i2c_master_read(I2C_MSG *msg, uint8_t retry) {
  uint8_t i;
  osStatus_t status;

  if (DEBUG_I2C) {
    printf("i2c_master_read: bus %d, addr %x, rxlen %d, txlen %d, txbuf:",msg->bus, msg->slave_addr, msg->rx_len, msg->tx_len);
    for (int i = 0; i < msg->tx_len; i++) {
      printf(" %x",msg->data[i]);
    }
    printf("\n");
  }

  if(msg->rx_len == 0) {
    printf("i2c_master_read with rx_len = 0\n");
    return false;
  }

  do { // break while getting mutex success but tranmission fail
    status = osMutexAcquire(util_i2c_mutex[msg->bus], I2C_mutex_timeout);
    if (status == osOK) {
      for (i = 0; i < retry; i++) {
        if ( i2c_write(&i2c[msg->bus], msg->slave_addr, &msg->data, msg->tx_len, 0) ) {
          continue;
        }
        memset(msg->data, 0xff, msg->rx_len);
        if ( i2c_read(&i2c[msg->bus], msg->slave_addr, &msg->data, msg->rx_len, I2C_M_STOP) ) {
          continue;
        }

        if (DEBUG_I2C) {
          printf("rxbuf:");
          for (int i = 0; i < msg->rx_len; i++) {
            printf(" %x",msg->data[i]);
          }
          printf("\n");
        }

        status = osMutexRelease(util_i2c_mutex[msg->bus]);
        if (status != osOK) {
          printf("I2C %d master read release mutex fail\n",msg->bus);
        }
        return true; // i2c write and read success
      }
      printf("I2C %d master read retry reach max\n",msg->bus);
      status = osMutexRelease(util_i2c_mutex[msg->bus]);
      if (status != osOK) {
        printf("I2C %d master read release mutex fail\n",msg->bus);
      }
      return false;
    } else {
      printf("I2C %d master read get mutex timeout\n",msg->bus);
      return false;
    }
  } while(0);

  
  return false; // should not reach here
}

bool i2c_master_write(I2C_MSG *msg, uint8_t retry) {
  uint8_t i;
  osStatus_t status;

  if (DEBUG_I2C) {
    printf("i2c_master_write: bus %d, addr %x, txlen %d, txbuf:",msg->bus, msg->slave_addr, msg->tx_len);
    for (int i = 0; i < msg->tx_len; i++) {
      printf(" %x",msg->data[i]);
    }
    printf("\n");
  }

  status = osMutexAcquire(util_i2c_mutex[msg->bus], I2C_mutex_timeout);
  if (status == osOK) {
    for (i = 0; i < retry; i++) {
      if ( i2c_write(&i2c[msg->bus], msg->slave_addr, &msg->data, msg->tx_len, I2C_M_STOP) ) {
        continue;        
      } else { // i2c write success
        status = osMutexRelease(util_i2c_mutex[msg->bus]);
        if (status != osOK) {
          printf("I2C %d master write release mutex fail\n",msg->bus);
        }
        return true;
      }
    }
    printf("I2C %d master write retry reach max\n",msg->bus);
    status = osMutexRelease(util_i2c_mutex[msg->bus]);
    if (status != osOK) {
      printf("I2C %d master write release mutex fail\n",msg->bus);
    }
    return false;

  } else {
    printf("I2C %d master write get mutex timeout\n",msg->bus);
    return false;
  }

  return false;
}

void util_init_I2C(void) {
  uint8_t i;

  for (i = 0; i < MAX_I2C_BUS_NUM; i++) {
    util_i2c_mutex[i] = osMutexNew(&util_i2c_Mutex_attr);
    if (util_i2c_mutex[i] == NULL) {
      printf("util I2C mutex %d init fail\n",i);
    }
  }
}
