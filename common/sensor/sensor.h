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

#ifndef SENSOR_H
#define SENSOR_H

#include "objects.h"
#include "i2c_aspeed.h"
#include "sdr.h"

#define get_from_sensor 0x00
#define get_from_cache 0x01

#define sensor_null 0xFF
#define sensor_fail 0xFF
#define SENSOR_NUM_MAX 0x64

#define SNR_POLL_INTERVEL_ms 10000

#define DEBUG_SNR 0

static inline uint8_t cal_MBR(uint8_t sensor_num, int val){
 return (val* SDR_M(sensor_num)/ SDR_Rexp(sensor_num)) + round_add(sensor_num,val); 
}

enum {
  SNR_READ_SUCCESS,
  SNR_NOT_FOUND,
  SNR_NOT_ACCESSIBLE,
  SNR_FAIL_TO_ACCESS,
  SNR_INIT_STATUS,
  SNR_UNSPECIFIED_ERROR,
};

typedef struct _snr_cfg__ {
  uint8_t num;
  uint8_t type;
  uint8_t port; // port, bus, channel, etc.
  uint8_t slave_addr;
  uint8_t offset;
  bool (*access_checker)(uint8_t);
  uint8_t index;
  uint8_t cache;
  uint8_t cache_status;
} snr_cfg;
  
extern bool enable_sensor_poll;
extern int SDR_NUM;
extern snr_cfg sensor_config[];
extern uint8_t SnrNum_SnrCfg_map[SENSOR_NUM_MAX];
extern i2c_t i2c[6];

uint8_t get_sensor_reading(uint8_t sensor_num, int *reading, uint8_t read_mode);
bool sensor_init(void);
bool tmp75_read(uint8_t sensor_num, int *reading);

#endif

