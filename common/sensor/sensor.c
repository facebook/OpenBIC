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
#include "i2c_aspeed.h"
#include "sensor_def.h"

#define tmp75_addr 0x4d
#define tmp75_tmp_offset 0x00

uint8_t SnrNum_SnrCfg_map[SENSOR_NUM_MAX];
uint8_t SnrNum_SDR_map[SENSOR_NUM_MAX];

bool enable_sensor_poll = 0;

const int negative_ten_power[16] = {1,1,1,1,1,1,1,1000000000,100000000,10000000,1000000,100000,10000,1000,100,10};

__WEAK snr_cfg sensor_config[] = {
  /* number,              type,        port,     address,    offset,           access check         index,  cache,  cache_status */     
};

static void init_SnrNum(void) {
  for (int i = 0; i < SENSOR_NUM_MAX; i++) {
    SnrNum_SDR_map[i] = 0xFF;
    SnrNum_SnrCfg_map[i] = 0xFF;
  }
}

void map_SnrNum_SDR_CFG(void) {
  uint8_t i,j;

  for (i = 0; i < SENSOR_NUM_MAX; i++) {
    for (j = 0; j < SDR_NUM; j++) {
      if (i == full_sensor_table[j].sensor_num) {
        SnrNum_SDR_map[i] = j;
        break;
      } else if (i == SDR_NUM) {
        SnrNum_SDR_map[i] = sensor_null;
      }
    }
    for (j = 0; j < SDR_NUM; j++) {
      if (i == sensor_config[j].num) {
        SnrNum_SnrCfg_map[i] = j;
        break;
      }
      else if (i == SDR_NUM) {
        SnrNum_SnrCfg_map[i] = sensor_null;
      }
    }
  }

  return ;
}

bool access_check(uint8_t sensor_num) {
  bool (*access_checker)(uint8_t);

  access_checker = sensor_config[SnrNum_SnrCfg_map[sensor_num]].access_checker;
  return (access_checker)(sensor_config[SnrNum_SnrCfg_map[sensor_num]].num);
}

bool sensor_read(uint8_t sensor_num, int *reading) {
  bool status;

  switch(sensor_config[SnrNum_SnrCfg_map[sensor_num]].type){
    case tmp75:
      status = tmp75_read(sensor_num, reading);
      if (status)
        return true;
      break;

    default:
      printf("sensor_read with unexpected sensor type\n");
      return false;
      break;
  }
  return false;
}

uint8_t get_sensor_reading(uint8_t sensor_num, int *reading, uint8_t read_mode) {
  uint8_t status;

  if(SnrNum_SDR_map[sensor_num] == 0xFF) { // look for sensor in SDR table
    return SNR_NOT_FOUND;
  }

  if( !access_check(sensor_num) ) { // sensor not accessable
    return SNR_NOT_ACCESSIBLE;
  }

  if (read_mode == get_from_sensor) {
    status = sensor_read(sensor_num, reading);
    if (status) {
      return sensor_config[sensor_num].cache_status;
    } else {
      printf("sensor[%x] read fail\n",sensor_num);
      return sensor_config[sensor_num].cache_status;
    }
  } else if (read_mode == get_from_cache) {
    if (sensor_config[sensor_num].cache_status == SNR_READ_SUCCESS ) {
      *reading = sensor_config[sensor_num].cache;
      return sensor_config[sensor_num].cache_status;
    } else {
      sensor_config[sensor_num].cache = sensor_fail;
      sensor_config[sensor_num].cache_status = SNR_FAIL_TO_ACCESS;
      printf("sensor[%x] cache read fail\n",sensor_num);
      return sensor_config[sensor_num].cache_status;
    }
  }

  return SNR_UNSPECIFIED_ERROR; // should not reach here
}

void SNR_poll_handler(void* pvParameters) {
  uint8_t poll_num;
  int reading;
  osStatus_t os_status; 

  osDelay(SNR_POLL_INTERVEL_ms);

  while(1) {
    for (poll_num = 0; poll_num < SENSOR_NUM_MAX; poll_num++) {
      if (SnrNum_SnrCfg_map[poll_num] == sensor_null) { // sensor not exist
        continue;
      }
      get_sensor_reading(SnrNum_SnrCfg_map[poll_num], &reading, get_from_sensor);

      os_status = osThreadYield();
      if (os_status != osOK) {
        printf("SNR_poll_handler yield failure\n");
      }
    }
    osDelay(SNR_POLL_INTERVEL_ms);
  }
}

void sensor_poll_init()
{
  osThreadAttr_t SNR_poll_Task_attr;

  memset(&SNR_poll_Task_attr, 0, sizeof(SNR_poll_Task_attr));
  SNR_poll_Task_attr.name = "SNR_poll_task";
  SNR_poll_Task_attr.priority = osPriorityBelowNormal;
  SNR_poll_Task_attr.stack_size = 0x1000;
  osThreadNew(SNR_poll_handler, NULL, &SNR_poll_Task_attr);

  return;
}

bool sensor_init(void)
{
  init_SnrNum();
  map_SnrNum_SDR_CFG();  
  SDR_init();
  
  if (DEBUG_SNR) {
    printf("SNR0: %s, SNR1: %s\n",full_sensor_table[SnrNum_SDR_map[1]].ID_str, full_sensor_table[SnrNum_SDR_map[3]].ID_str);
  }

  if (enable_sensor_poll) {
    sensor_poll_init();
  }

  return true;
}
