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
#include <string.h>
#include "sdr.h"
#include "sensor.h"
#include "sensor_def.h"
#include "plat_ipmb.h"

SDR_INFO sdr_info;
static uint16_t RSV_ID =0;
uint8_t is_SDR_not_init = 1;

__WEAK SDR_Full_sensor full_sensor_table[] = {

};

int SDR_NUM = sizeof(full_sensor_table)/sizeof(full_sensor_table[0]);

void SDR_clear_ID(void) {
  sdr_info.current_ID = sdr_info.start_ID;
  return;
}

uint16_t SDR_get_record_ID(uint16_t current_ID) {
  uint16_t ret = (++current_ID);

  if (ret < sdr_info.last_ID) {
    return ret;
  } else if (ret == sdr_info.last_ID) {
    return SDR_END_ID;
  }

  return SDR_INVALID_ID;  
}

uint16_t SDR_check_record_ID(uint16_t current_ID) {
  if (current_ID > sdr_info.last_ID) {
    return false;
  }

  return true;
}

uint16_t SDR_get_RSV_ID(void) {
  return (++RSV_ID);
}

bool SDR_RSV_ID_check(uint16_t ID) {
  return (RSV_ID == ID) ? 1 : 0;
}

void SDR_init(void) {
  int i;

  sdr_info.start_ID = 0x0000;
  sdr_info.current_ID = sdr_info.start_ID;

  for (i=0; i< SDR_NUM; i++) {
    full_sensor_table[i].record_id_h = (i >> 8) & 0xFF;
    full_sensor_table[i].record_id_l = (i & 0xFF);
    full_sensor_table[i].ID_len = strlen(full_sensor_table[i].ID_str);
    full_sensor_table[i].record_len += full_sensor_table[i].ID_len;
    
    if (DEBUG_SNR) {
      printf("%s ID: 0x%x%x, size: %d, recordlen: %d\n",full_sensor_table[i].ID_str, full_sensor_table[i].record_id_h, full_sensor_table[i].record_id_l, full_sensor_table[i].ID_len, full_sensor_table[i].record_len);
    }
  }
  i--;
  sdr_info.last_ID =  (full_sensor_table[i].record_id_h << 8) | (full_sensor_table[i].record_id_l);

  is_SDR_not_init = 0;
  return;
}


