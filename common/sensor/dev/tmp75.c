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
#include "sensor.h"
#include "hal_i2c.h"

bool tmp75_read(uint8_t sensor_num, int *reading) {
  uint8_t retry = 5;
  int val;
  I2C_MSG msg;

  msg.bus = sensor_config[SnrNum_SnrCfg_map[sensor_num]].port;
  msg.slave_addr = sensor_config[SnrNum_SnrCfg_map[sensor_num]].slave_addr;
  msg.tx_len = 1;
  msg.rx_len = 1;
  msg.data[0] = sensor_config[SnrNum_SnrCfg_map[sensor_num]].offset;

  if ( !i2c_master_read(&msg, retry) ) {
    sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache_status = SNR_FAIL_TO_ACCESS;
    printf("Snr num %x read tmp75 fail\n", sensor_num);
    return false;
  }
  
  val = msg.data[0];

  *reading = cal_MBR(sensor_num,val);
  sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache = *reading;
  sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache_status = SNR_READ_SUCCESS;

  return true;
}



