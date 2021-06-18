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
#include "hal_eeprom.h"
#include "hal_i2c.h"
#include <string.h>

uint8_t eeprom_write(EEPROM_ENTRY *entry) {
  I2C_MSG msg;
  uint8_t retry = 5;

  msg.bus = entry->config.port;
  msg.slave_addr = entry->config.slave_addr;
  msg.tx_len = entry->data_len+2; // write 2 byte offset to EEPROM
  msg.data[0] = ((entry->config.start_offset + entry->offset) >> 8) & 0xFF; // offset msb
  msg.data[1] = (entry->config.start_offset + entry->offset) & 0xFF; // offset lsb
  memcpy(&msg.data[2],&entry->data[0],entry->data_len);

  if ( !i2c_master_write(&msg, retry) ) {
    return false;
  }

  return true;
}


uint8_t eeprom_read(EEPROM_ENTRY *entry) {
  I2C_MSG msg;
  uint8_t retry = 5;
  
  msg.bus = entry->config.port;
  msg.slave_addr = entry->config.slave_addr;
  msg.tx_len = 2; // write 2 byte offset to EEPROM
  msg.rx_len = entry->data_len;
  msg.data[0] = ((entry->config.start_offset + entry->offset) >> 8) & 0xFF; // offset msb
  msg.data[1] = (entry->config.start_offset + entry->offset) & 0xFF; // offset lsb

  if ( !i2c_master_read(&msg, retry) ) {
    return false;
  }

  entry->data_len = msg.rx_len;
  memcpy(&entry->data, &msg.data, msg.rx_len);

  return true;
}
 
