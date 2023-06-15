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

#ifndef PLAT_SENSOR_TABLE_H
#define PLAT_SENSOR_TABLE_H

/* SENSOR ADDRESS(7-bit)/OFFSET */
#define TMP75_IN_ADDR (0x94 >> 1)
#define TMP75_OUT_ADDR (0x92 >> 1)
#define TMP75_FIO_ADDR (0x90 >> 1)
#define SSD_ADDR (0xD4 >> 1)
#define MPRO_ADDR (0x9E >> 1)

#define ADM1278_ADDR (0x80 >> 1)
#define LTC4282_ADDR (0x82 >> 1)
#define TEMP_HSC_ADDR (0x98 >> 1)
#define MP5990_ADDR (0xA0 >> 1)

/* SENSOR NUMBER - sel */
#define SENSOR_NUM_SYSTEM_STATUS 0x10
#define SENSOR_NUM_POWER_ERROR 0x56
#define SENSOR_NUM_PROC_FAIL 0x65
#define SENSOR_NUM_VR_OCP 0x71
#define SENSOR_NUM_VR_HOT 0x72
#define SENSOR_NUM_VR_FAULT 0x73
#define SENSOR_NUM_PMIC_ERROR 0xB4

#endif
