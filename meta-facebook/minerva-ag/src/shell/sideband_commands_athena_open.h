
//
// Copyright 2020 Broadcom Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

// Script: flatstruct.py v1.0
// Ran on: Thu Oct 23 09:34:01 2025
// Input:  monitor_athena.MOD.h

#pragma once

// Various suffixes:
//     BITPOS    : For bitfields, the starting bit position.
//     BITWID    : For bitfields, the bitfield width.
//     BYTES     : Full size of the field in bytes (ie 128 for "uint32_t[7]").
//     COUNT     : Count of the field type (ie 7 for "uint32_t[7]").
//     OFFSET    : Offset of the field in bytes. Only present for non-bitfields.
//     SIZE      : Width of the field type in bytes (ie 4 for "uint32_t[7]").
//     TYPE      : C type of the field

// ---------- monitoring_details ----------

// monitoring_details
#define  BRCMLIB__MONITORING_DETAILS__BYTES                                                          (12680)
#define  BRCMLIB__MONITORING_DETAILS__OFFSET                                                         (0)
#define  BRCMLIB__MONITORING_DETAILS__SIZE                                                           (12680)
#define  BRCMLIB__MONITORING_DETAILS__FULLPOS                                                        (0)
#define  BRCMLIB__MONITORING_DETAILS__FULLWID                                                        (101440)

// monitoring_details.header
#define  BRCMLIB__MONITORING_DETAILS_HEADER__BYTES                                                   (12)
#define  BRCMLIB__MONITORING_DETAILS_HEADER__COUNT                                                   (1)
#define  BRCMLIB__MONITORING_DETAILS_HEADER__OFFSET                                                  (0)
#define  BRCMLIB__MONITORING_DETAILS_HEADER__SIZE                                                    (12)
#define  BRCMLIB__MONITORING_DETAILS_HEADER__FULLPOS                                                 (0)
#define  BRCMLIB__MONITORING_DETAILS_HEADER__FULLWID                                                 (96)

// monitoring_details.header.size
#define  BRCMLIB__MONITORING_DETAILS_HEADER_SIZE__BYTES                                              (4)
#define  BRCMLIB__MONITORING_DETAILS_HEADER_SIZE__COUNT                                              (1)
#define  BRCMLIB__MONITORING_DETAILS_HEADER_SIZE__OFFSET                                             (0)
#define  BRCMLIB__MONITORING_DETAILS_HEADER_SIZE__SIZE                                               (4)
//#define  BRCMLIB__MONITORING_DETAILS_HEADER_SIZE__TYPE                                               (uint32_t)
#define  BRCMLIB__MONITORING_DETAILS_HEADER_SIZE__FULLPOS                                            (0)
#define  BRCMLIB__MONITORING_DETAILS_HEADER_SIZE__FULLWID                                            (32)

// monitoring_details.header.version
#define  BRCMLIB__MONITORING_DETAILS_HEADER_VERSION__BYTES                                           (4)
#define  BRCMLIB__MONITORING_DETAILS_HEADER_VERSION__COUNT                                           (1)
#define  BRCMLIB__MONITORING_DETAILS_HEADER_VERSION__OFFSET                                          (4)
#define  BRCMLIB__MONITORING_DETAILS_HEADER_VERSION__SIZE                                            (4)
//#define  BRCMLIB__MONITORING_DETAILS_HEADER_VERSION__TYPE                                            (uint32_t)
#define  BRCMLIB__MONITORING_DETAILS_HEADER_VERSION__FULLPOS                                         (32)
#define  BRCMLIB__MONITORING_DETAILS_HEADER_VERSION__FULLWID                                         (32)

// monitoring_details.header.device
#define  BRCMLIB__MONITORING_DETAILS_HEADER_DEVICE__BYTES                                            (4)
#define  BRCMLIB__MONITORING_DETAILS_HEADER_DEVICE__COUNT                                            (1)
#define  BRCMLIB__MONITORING_DETAILS_HEADER_DEVICE__OFFSET                                           (8)
#define  BRCMLIB__MONITORING_DETAILS_HEADER_DEVICE__SIZE                                             (4)
//#define  BRCMLIB__MONITORING_DETAILS_HEADER_DEVICE__TYPE                                             (uint32_t)
#define  BRCMLIB__MONITORING_DETAILS_HEADER_DEVICE__FULLPOS                                          (64)
#define  BRCMLIB__MONITORING_DETAILS_HEADER_DEVICE__FULLWID                                          (32)

// monitoring_details.vtmon
#define  BRCMLIB__MONITORING_DETAILS_VTMON__BYTES                                                    (52)
#define  BRCMLIB__MONITORING_DETAILS_VTMON__COUNT                                                    (1)
#define  BRCMLIB__MONITORING_DETAILS_VTMON__OFFSET                                                   (66)
#define  BRCMLIB__MONITORING_DETAILS_VTMON__SIZE                                                     (52)
#define  BRCMLIB__MONITORING_DETAILS_VTMON__FULLPOS                                                  (528)
#define  BRCMLIB__MONITORING_DETAILS_VTMON__FULLWID                                                  (416)

// monitoring_details.vtmon.north
#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH__BYTES                                              (26)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH__COUNT                                              (1)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH__OFFSET                                             (66)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH__SIZE                                               (26)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH__FULLPOS                                            (528)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH__FULLWID                                            (208)

// monitoring_details.vtmon.north.id
#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH_ID__BYTES                                           (1)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH_ID__COUNT                                           (1)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH_ID__OFFSET                                          (66)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH_ID__SIZE                                            (1)
//#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH_ID__TYPE                                            (uint8_t)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH_ID__FULLPOS                                         (528)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH_ID__FULLWID                                         (8)

// monitoring_details.vtmon.north.temp_local
#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH_TEMP_LOCAL__BYTES                                   (1)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH_TEMP_LOCAL__COUNT                                   (1)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH_TEMP_LOCAL__OFFSET                                  (67)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH_TEMP_LOCAL__SIZE                                    (1)
//#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH_TEMP_LOCAL__TYPE                                    (uint8_t)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH_TEMP_LOCAL__FULLPOS                                 (536)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH_TEMP_LOCAL__FULLWID                                 (8)

// monitoring_details.vtmon.north.temp_remote
#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH_TEMP_REMOTE__BYTES                                  (8)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH_TEMP_REMOTE__COUNT                                  (8)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH_TEMP_REMOTE__OFFSET                                 (68)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH_TEMP_REMOTE__SIZE                                   (1)
//#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH_TEMP_REMOTE__TYPE                                   (uint8_t)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH_TEMP_REMOTE__FULLPOS                                (544)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH_TEMP_REMOTE__FULLWID                                (64)

// monitoring_details.vtmon.north.volt_1v
#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH_VOLT_1V__BYTES                                      (12)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH_VOLT_1V__COUNT                                      (6)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH_VOLT_1V__OFFSET                                     (76)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH_VOLT_1V__SIZE                                       (2)
//#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH_VOLT_1V__TYPE                                       (uint16_t)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH_VOLT_1V__FULLPOS                                    (608)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH_VOLT_1V__FULLWID                                    (96)

// monitoring_details.vtmon.north.volt_1p8v
#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH_VOLT_1P8V__BYTES                                    (2)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH_VOLT_1P8V__COUNT                                    (1)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH_VOLT_1P8V__OFFSET                                   (88)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH_VOLT_1P8V__SIZE                                     (2)
//#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH_VOLT_1P8V__TYPE                                     (uint16_t)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH_VOLT_1P8V__FULLPOS                                  (704)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH_VOLT_1P8V__FULLWID                                  (16)

// monitoring_details.vtmon.north.volt_3p3v
#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH_VOLT_3P3V__BYTES                                    (2)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH_VOLT_3P3V__COUNT                                    (1)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH_VOLT_3P3V__OFFSET                                   (90)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH_VOLT_3P3V__SIZE                                     (2)
//#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH_VOLT_3P3V__TYPE                                     (uint16_t)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH_VOLT_3P3V__FULLPOS                                  (720)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_NORTH_VOLT_3P3V__FULLWID                                  (16)

// monitoring_details.vtmon.south
#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH__BYTES                                              (26)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH__COUNT                                              (1)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH__OFFSET                                             (92)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH__SIZE                                               (26)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH__FULLPOS                                            (736)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH__FULLWID                                            (208)

// monitoring_details.vtmon.south.id
#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH_ID__BYTES                                           (1)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH_ID__COUNT                                           (1)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH_ID__OFFSET                                          (92)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH_ID__SIZE                                            (1)
//#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH_ID__TYPE                                            (uint8_t)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH_ID__FULLPOS                                         (736)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH_ID__FULLWID                                         (8)

// monitoring_details.vtmon.south.temp_local
#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH_TEMP_LOCAL__BYTES                                   (1)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH_TEMP_LOCAL__COUNT                                   (1)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH_TEMP_LOCAL__OFFSET                                  (93)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH_TEMP_LOCAL__SIZE                                    (1)
//#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH_TEMP_LOCAL__TYPE                                    (uint8_t)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH_TEMP_LOCAL__FULLPOS                                 (744)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH_TEMP_LOCAL__FULLWID                                 (8)

// monitoring_details.vtmon.south.temp_remote
#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH_TEMP_REMOTE__BYTES                                  (8)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH_TEMP_REMOTE__COUNT                                  (8)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH_TEMP_REMOTE__OFFSET                                 (94)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH_TEMP_REMOTE__SIZE                                   (1)
//#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH_TEMP_REMOTE__TYPE                                   (uint8_t)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH_TEMP_REMOTE__FULLPOS                                (752)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH_TEMP_REMOTE__FULLWID                                (64)

// monitoring_details.vtmon.south.volt_1v
#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH_VOLT_1V__BYTES                                      (12)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH_VOLT_1V__COUNT                                      (6)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH_VOLT_1V__OFFSET                                     (102)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH_VOLT_1V__SIZE                                       (2)
//#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH_VOLT_1V__TYPE                                       (uint16_t)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH_VOLT_1V__FULLPOS                                    (816)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH_VOLT_1V__FULLWID                                    (96)

// monitoring_details.vtmon.south.volt_1p8v
#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH_VOLT_1P8V__BYTES                                    (2)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH_VOLT_1P8V__COUNT                                    (1)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH_VOLT_1P8V__OFFSET                                   (114)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH_VOLT_1P8V__SIZE                                     (2)
//#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH_VOLT_1P8V__TYPE                                     (uint16_t)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH_VOLT_1P8V__FULLPOS                                  (912)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH_VOLT_1P8V__FULLWID                                  (16)

// monitoring_details.vtmon.south.volt_3p3v
#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH_VOLT_3P3V__BYTES                                    (2)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH_VOLT_3P3V__COUNT                                    (1)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH_VOLT_3P3V__OFFSET                                   (116)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH_VOLT_3P3V__SIZE                                     (2)
//#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH_VOLT_3P3V__TYPE                                     (uint16_t)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH_VOLT_3P3V__FULLPOS                                  (928)
#define  BRCMLIB__MONITORING_DETAILS_VTMON_SOUTH_VOLT_3P3V__FULLWID                                  (16)

// monitoring_details.efuse
#define  BRCMLIB__MONITORING_DETAILS_EFUSE__BYTES                                                    (111)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE__COUNT                                                    (1)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE__OFFSET                                                   (2242)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE__SIZE                                                     (111)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE__FULLPOS                                                  (17936)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE__FULLWID                                                  (888)

// monitoring_details.efuse.athena
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA__BYTES                                             (37)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA__COUNT                                             (1)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA__OFFSET                                            (2242)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA__SIZE                                              (37)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA__FULLPOS                                           (17936)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA__FULLWID                                           (296)

// monitoring_details.efuse.athena.fab_designation
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_FAB_DESIGNATION__BYTES                             (1)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_FAB_DESIGNATION__COUNT                             (1)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_FAB_DESIGNATION__OFFSET                            (2242)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_FAB_DESIGNATION__SIZE                              (1)
//#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_FAB_DESIGNATION__TYPE                              (char)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_FAB_DESIGNATION__FULLPOS                           (17936)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_FAB_DESIGNATION__FULLWID                           (8)

// monitoring_details.efuse.athena.lot_designation
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_LOT_DESIGNATION__BYTES                             (1)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_LOT_DESIGNATION__COUNT                             (1)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_LOT_DESIGNATION__OFFSET                            (2243)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_LOT_DESIGNATION__SIZE                              (1)
//#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_LOT_DESIGNATION__TYPE                              (char)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_LOT_DESIGNATION__FULLPOS                           (17944)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_LOT_DESIGNATION__FULLWID                           (8)

// monitoring_details.efuse.athena.lot_number
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_LOT_NUMBER__BYTES                                  (4)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_LOT_NUMBER__COUNT                                  (4)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_LOT_NUMBER__OFFSET                                 (2244)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_LOT_NUMBER__SIZE                                   (1)
//#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_LOT_NUMBER__TYPE                                   (char)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_LOT_NUMBER__FULLPOS                                (17952)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_LOT_NUMBER__FULLWID                                (32)

// monitoring_details.efuse.athena.wafer_number
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_WAFER_NUMBER__BYTES                                (1)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_WAFER_NUMBER__COUNT                                (1)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_WAFER_NUMBER__OFFSET                               (2248)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_WAFER_NUMBER__SIZE                                 (1)
//#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_WAFER_NUMBER__TYPE                                 (uint8_t)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_WAFER_NUMBER__FULLPOS                              (17984)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_WAFER_NUMBER__FULLWID                              (8)

// monitoring_details.efuse.athena.wafer_region
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_WAFER_REGION__BYTES                                (2)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_WAFER_REGION__COUNT                                (1)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_WAFER_REGION__OFFSET                               (2249)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_WAFER_REGION__SIZE                                 (2)
//#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_WAFER_REGION__TYPE                                 (uint16_t)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_WAFER_REGION__FULLPOS                              (17992)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_WAFER_REGION__FULLWID                              (16)

// monitoring_details.efuse.athena.internal1
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_INTERNAL1__BYTES                                   (1)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_INTERNAL1__COUNT                                   (1)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_INTERNAL1__OFFSET                                  (2251)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_INTERNAL1__SIZE                                    (1)
//#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_INTERNAL1__TYPE                                    (uint8_t)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_INTERNAL1__FULLPOS                                 (18008)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_INTERNAL1__FULLWID                                 (8)

// monitoring_details.efuse.athena.oscillator_count
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_OSCILLATOR_COUNT__BYTES                            (2)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_OSCILLATOR_COUNT__COUNT                            (1)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_OSCILLATOR_COUNT__OFFSET                           (2252)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_OSCILLATOR_COUNT__SIZE                             (2)
//#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_OSCILLATOR_COUNT__TYPE                             (uint16_t)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_OSCILLATOR_COUNT__FULLPOS                          (18016)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_OSCILLATOR_COUNT__FULLWID                          (16)

// monitoring_details.efuse.athena.internal2
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_INTERNAL2__BYTES                                   (8)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_INTERNAL2__COUNT                                   (1)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_INTERNAL2__OFFSET                                  (2254)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_INTERNAL2__SIZE                                    (8)
//#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_INTERNAL2__TYPE                                    (uint64_t)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_INTERNAL2__FULLPOS                                 (18032)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_INTERNAL2__FULLWID                                 (64)

// monitoring_details.efuse.athena.overclocking
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_OVERCLOCKING__BYTES                                (2)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_OVERCLOCKING__COUNT                                (1)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_OVERCLOCKING__OFFSET                               (2262)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_OVERCLOCKING__SIZE                                 (2)
//#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_OVERCLOCKING__TYPE                                 (uint16_t)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_OVERCLOCKING__FULLPOS                              (18096)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_OVERCLOCKING__FULLWID                              (16)

// monitoring_details.efuse.athena.m7serial
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_M7SERIAL__BYTES                                    (15)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_M7SERIAL__COUNT                                    (15)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_M7SERIAL__OFFSET                                   (2264)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_M7SERIAL__SIZE                                     (1)
//#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_M7SERIAL__TYPE                                     (uint8_t)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_M7SERIAL__FULLPOS                                  (18112)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_ATHENA_M7SERIAL__FULLWID                                  (120)

// monitoring_details.efuse.owl
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__BYTES                                                (74)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__COUNT                                                (2)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__OFFSET                                               (2279)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__SIZE                                                 (37)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__FULLPOS                                              (18232)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__FULLWID                                              (592)

// monitoring_details.efuse.owl[0]
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0__BYTES                                             (37)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0__COUNT                                             (1)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0__OFFSET                                            (2279)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0__SIZE                                              (37)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0__FULLPOS                                           (18232)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0__FULLWID                                           (296)

// monitoring_details.efuse.owl[0].fab_designation
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_FAB_DESIGNATION__BYTES                             (1)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_FAB_DESIGNATION__COUNT                             (1)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_FAB_DESIGNATION__OFFSET                            (2279)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_FAB_DESIGNATION__SIZE                              (1)
//#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_FAB_DESIGNATION__TYPE                              (char)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_FAB_DESIGNATION__FULLPOS                           (18232)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_FAB_DESIGNATION__FULLWID                           (8)

// monitoring_details.efuse.owl[0].lot_designation
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_LOT_DESIGNATION__BYTES                             (1)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_LOT_DESIGNATION__COUNT                             (1)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_LOT_DESIGNATION__OFFSET                            (2280)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_LOT_DESIGNATION__SIZE                              (1)
//#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_LOT_DESIGNATION__TYPE                              (char)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_LOT_DESIGNATION__FULLPOS                           (18240)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_LOT_DESIGNATION__FULLWID                           (8)

// monitoring_details.efuse.owl[0].lot_number
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_LOT_NUMBER__BYTES                                  (4)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_LOT_NUMBER__COUNT                                  (4)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_LOT_NUMBER__OFFSET                                 (2281)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_LOT_NUMBER__SIZE                                   (1)
//#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_LOT_NUMBER__TYPE                                   (char)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_LOT_NUMBER__FULLPOS                                (18248)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_LOT_NUMBER__FULLWID                                (32)

// monitoring_details.efuse.owl[0].wafer_number
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_WAFER_NUMBER__BYTES                                (1)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_WAFER_NUMBER__COUNT                                (1)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_WAFER_NUMBER__OFFSET                               (2285)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_WAFER_NUMBER__SIZE                                 (1)
//#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_WAFER_NUMBER__TYPE                                 (uint8_t)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_WAFER_NUMBER__FULLPOS                              (18280)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_WAFER_NUMBER__FULLWID                              (8)

// monitoring_details.efuse.owl[0].wafer_region
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_WAFER_REGION__BYTES                                (2)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_WAFER_REGION__COUNT                                (1)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_WAFER_REGION__OFFSET                               (2286)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_WAFER_REGION__SIZE                                 (2)
//#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_WAFER_REGION__TYPE                                 (uint16_t)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_WAFER_REGION__FULLPOS                              (18288)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_WAFER_REGION__FULLWID                              (16)

// monitoring_details.efuse.owl[0].internal1
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_INTERNAL1__BYTES                                   (1)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_INTERNAL1__COUNT                                   (1)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_INTERNAL1__OFFSET                                  (2288)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_INTERNAL1__SIZE                                    (1)
//#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_INTERNAL1__TYPE                                    (uint8_t)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_INTERNAL1__FULLPOS                                 (18304)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_INTERNAL1__FULLWID                                 (8)

// monitoring_details.efuse.owl[0].oscillator_count
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_OSCILLATOR_COUNT__BYTES                            (2)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_OSCILLATOR_COUNT__COUNT                            (1)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_OSCILLATOR_COUNT__OFFSET                           (2289)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_OSCILLATOR_COUNT__SIZE                             (2)
//#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_OSCILLATOR_COUNT__TYPE                             (uint16_t)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_OSCILLATOR_COUNT__FULLPOS                          (18312)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_OSCILLATOR_COUNT__FULLWID                          (16)

// monitoring_details.efuse.owl[0].internal2
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_INTERNAL2__BYTES                                   (8)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_INTERNAL2__COUNT                                   (1)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_INTERNAL2__OFFSET                                  (2291)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_INTERNAL2__SIZE                                    (8)
//#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_INTERNAL2__TYPE                                    (uint64_t)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_INTERNAL2__FULLPOS                                 (18328)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_INTERNAL2__FULLWID                                 (64)

// monitoring_details.efuse.owl[0].overclocking
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_OVERCLOCKING__BYTES                                (2)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_OVERCLOCKING__COUNT                                (1)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_OVERCLOCKING__OFFSET                               (2299)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_OVERCLOCKING__SIZE                                 (2)
//#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_OVERCLOCKING__TYPE                                 (uint16_t)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_OVERCLOCKING__FULLPOS                              (18392)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_OVERCLOCKING__FULLWID                              (16)

// monitoring_details.efuse.owl[0].m7serial
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_M7SERIAL__BYTES                                    (15)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_M7SERIAL__COUNT                                    (15)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_M7SERIAL__OFFSET                                   (2301)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_M7SERIAL__SIZE                                     (1)
//#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_M7SERIAL__TYPE                                     (uint8_t)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_M7SERIAL__FULLPOS                                  (18408)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__0_M7SERIAL__FULLWID                                  (120)

// monitoring_details.efuse.owl[1]
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1__BYTES                                             (37)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1__COUNT                                             (1)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1__OFFSET                                            (2316)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1__SIZE                                              (37)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1__FULLPOS                                           (18528)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1__FULLWID                                           (296)

// monitoring_details.efuse.owl[1].fab_designation
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_FAB_DESIGNATION__BYTES                             (1)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_FAB_DESIGNATION__COUNT                             (1)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_FAB_DESIGNATION__OFFSET                            (2316)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_FAB_DESIGNATION__SIZE                              (1)
//#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_FAB_DESIGNATION__TYPE                              (char)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_FAB_DESIGNATION__FULLPOS                           (18528)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_FAB_DESIGNATION__FULLWID                           (8)

// monitoring_details.efuse.owl[1].lot_designation
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_LOT_DESIGNATION__BYTES                             (1)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_LOT_DESIGNATION__COUNT                             (1)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_LOT_DESIGNATION__OFFSET                            (2317)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_LOT_DESIGNATION__SIZE                              (1)
//#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_LOT_DESIGNATION__TYPE                              (char)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_LOT_DESIGNATION__FULLPOS                           (18536)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_LOT_DESIGNATION__FULLWID                           (8)

// monitoring_details.efuse.owl[1].lot_number
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_LOT_NUMBER__BYTES                                  (4)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_LOT_NUMBER__COUNT                                  (4)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_LOT_NUMBER__OFFSET                                 (2318)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_LOT_NUMBER__SIZE                                   (1)
//#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_LOT_NUMBER__TYPE                                   (char)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_LOT_NUMBER__FULLPOS                                (18544)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_LOT_NUMBER__FULLWID                                (32)

// monitoring_details.efuse.owl[1].wafer_number
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_WAFER_NUMBER__BYTES                                (1)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_WAFER_NUMBER__COUNT                                (1)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_WAFER_NUMBER__OFFSET                               (2322)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_WAFER_NUMBER__SIZE                                 (1)
//#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_WAFER_NUMBER__TYPE                                 (uint8_t)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_WAFER_NUMBER__FULLPOS                              (18576)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_WAFER_NUMBER__FULLWID                              (8)

// monitoring_details.efuse.owl[1].wafer_region
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_WAFER_REGION__BYTES                                (2)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_WAFER_REGION__COUNT                                (1)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_WAFER_REGION__OFFSET                               (2323)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_WAFER_REGION__SIZE                                 (2)
//#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_WAFER_REGION__TYPE                                 (uint16_t)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_WAFER_REGION__FULLPOS                              (18584)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_WAFER_REGION__FULLWID                              (16)

// monitoring_details.efuse.owl[1].internal1
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_INTERNAL1__BYTES                                   (1)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_INTERNAL1__COUNT                                   (1)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_INTERNAL1__OFFSET                                  (2325)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_INTERNAL1__SIZE                                    (1)
//#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_INTERNAL1__TYPE                                    (uint8_t)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_INTERNAL1__FULLPOS                                 (18600)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_INTERNAL1__FULLWID                                 (8)

// monitoring_details.efuse.owl[1].oscillator_count
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_OSCILLATOR_COUNT__BYTES                            (2)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_OSCILLATOR_COUNT__COUNT                            (1)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_OSCILLATOR_COUNT__OFFSET                           (2326)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_OSCILLATOR_COUNT__SIZE                             (2)
//#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_OSCILLATOR_COUNT__TYPE                             (uint16_t)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_OSCILLATOR_COUNT__FULLPOS                          (18608)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_OSCILLATOR_COUNT__FULLWID                          (16)

// monitoring_details.efuse.owl[1].internal2
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_INTERNAL2__BYTES                                   (8)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_INTERNAL2__COUNT                                   (1)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_INTERNAL2__OFFSET                                  (2328)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_INTERNAL2__SIZE                                    (8)
//#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_INTERNAL2__TYPE                                    (uint64_t)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_INTERNAL2__FULLPOS                                 (18624)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_INTERNAL2__FULLWID                                 (64)

// monitoring_details.efuse.owl[1].overclocking
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_OVERCLOCKING__BYTES                                (2)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_OVERCLOCKING__COUNT                                (1)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_OVERCLOCKING__OFFSET                               (2336)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_OVERCLOCKING__SIZE                                 (2)
//#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_OVERCLOCKING__TYPE                                 (uint16_t)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_OVERCLOCKING__FULLPOS                              (18688)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_OVERCLOCKING__FULLWID                              (16)

// monitoring_details.efuse.owl[1].m7serial
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_M7SERIAL__BYTES                                    (15)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_M7SERIAL__COUNT                                    (15)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_M7SERIAL__OFFSET                                   (2338)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_M7SERIAL__SIZE                                     (1)
//#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_M7SERIAL__TYPE                                     (uint8_t)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_M7SERIAL__FULLPOS                                  (18704)
#define  BRCMLIB__MONITORING_DETAILS_EFUSE_OWL__1_M7SERIAL__FULLWID                                  (120)

// monitoring_details.hbm_all.max_temp_all_chan
#define  BRCMLIB__MONITORING_DETAILS_HBM_ALL_MAX_TEMP_ALL_CHAN__BYTES                                (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM_ALL_MAX_TEMP_ALL_CHAN__COUNT                                (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM_ALL_MAX_TEMP_ALL_CHAN__OFFSET                               (4037)
#define  BRCMLIB__MONITORING_DETAILS_HBM_ALL_MAX_TEMP_ALL_CHAN__SIZE                                 (1)
//#define  BRCMLIB__MONITORING_DETAILS_HBM_ALL_MAX_TEMP_ALL_CHAN__TYPE                                 (int8_t)
#define  BRCMLIB__MONITORING_DETAILS_HBM_ALL_MAX_TEMP_ALL_CHAN__FULLPOS                              (32296)
#define  BRCMLIB__MONITORING_DETAILS_HBM_ALL_MAX_TEMP_ALL_CHAN__FULLWID                              (8)

// monitoring_details.hbm_all.min_temp_all_chan_sid2
#define  BRCMLIB__MONITORING_DETAILS_HBM_ALL_MIN_TEMP_ALL_CHAN_SID2__BYTES                           (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM_ALL_MIN_TEMP_ALL_CHAN_SID2__COUNT                           (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM_ALL_MIN_TEMP_ALL_CHAN_SID2__OFFSET                          (4038)
#define  BRCMLIB__MONITORING_DETAILS_HBM_ALL_MIN_TEMP_ALL_CHAN_SID2__SIZE                            (1)
//#define  BRCMLIB__MONITORING_DETAILS_HBM_ALL_MIN_TEMP_ALL_CHAN_SID2__TYPE                            (int8_t)
#define  BRCMLIB__MONITORING_DETAILS_HBM_ALL_MIN_TEMP_ALL_CHAN_SID2__FULLPOS                         (32304)
#define  BRCMLIB__MONITORING_DETAILS_HBM_ALL_MIN_TEMP_ALL_CHAN_SID2__FULLWID                         (8)

// monitoring_details.hbm[0].thermal.hbm_sel_two_reg
#define  BRCMLIB__MONITORING_DETAILS_HBM__0_THERMAL_HBM_SEL_TWO_REG__BYTES                           (68)
#define  BRCMLIB__MONITORING_DETAILS_HBM__0_THERMAL_HBM_SEL_TWO_REG__COUNT                           (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__0_THERMAL_HBM_SEL_TWO_REG__OFFSET                          (4783)
#define  BRCMLIB__MONITORING_DETAILS_HBM__0_THERMAL_HBM_SEL_TWO_REG__SIZE                            (68)
#define  BRCMLIB__MONITORING_DETAILS_HBM__0_THERMAL_HBM_SEL_TWO_REG__FULLPOS                         (38264)
#define  BRCMLIB__MONITORING_DETAILS_HBM__0_THERMAL_HBM_SEL_TWO_REG__FULLWID                         (544)

// monitoring_details.hbm[0].thermal.hbm_sel_two_reg.max_temp_current
#define  BRCMLIB__MONITORING_DETAILS_HBM__0_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__BYTES          (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__0_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__COUNT          (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__0_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__OFFSET         (4783)
#define  BRCMLIB__MONITORING_DETAILS_HBM__0_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__SIZE           (1)
//#define  BRCMLIB__MONITORING_DETAILS_HBM__0_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__TYPE           (int8_t)
#define  BRCMLIB__MONITORING_DETAILS_HBM__0_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__FULLPOS        (38264)
#define  BRCMLIB__MONITORING_DETAILS_HBM__0_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__FULLWID        (8)

// monitoring_details.hbm[0].thermal.hbm_sel_two_reg.junction_temperature
#define  BRCMLIB__MONITORING_DETAILS_HBM__0_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__BYTES      (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__0_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__COUNT      (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__0_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__OFFSET     (4784)
#define  BRCMLIB__MONITORING_DETAILS_HBM__0_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__SIZE       (1)
//#define  BRCMLIB__MONITORING_DETAILS_HBM__0_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__TYPE       (int8_t)
#define  BRCMLIB__MONITORING_DETAILS_HBM__0_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__FULLPOS    (38272)
#define  BRCMLIB__MONITORING_DETAILS_HBM__0_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__FULLWID    (8)

// monitoring_details.hbm[0].thermal.hbm_sel_two_reg.min_temp_history
#define  BRCMLIB__MONITORING_DETAILS_HBM__0_THERMAL_HBM_SEL_TWO_REG_MIN_TEMP_HISTORY__BYTES          (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__0_THERMAL_HBM_SEL_TWO_REG_MIN_TEMP_HISTORY__COUNT          (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__0_THERMAL_HBM_SEL_TWO_REG_MIN_TEMP_HISTORY__OFFSET         (4785)
#define  BRCMLIB__MONITORING_DETAILS_HBM__0_THERMAL_HBM_SEL_TWO_REG_MIN_TEMP_HISTORY__SIZE           (1)
//#define  BRCMLIB__MONITORING_DETAILS_HBM__0_THERMAL_HBM_SEL_TWO_REG_MIN_TEMP_HISTORY__TYPE           (int8_t)
#define  BRCMLIB__MONITORING_DETAILS_HBM__0_THERMAL_HBM_SEL_TWO_REG_MIN_TEMP_HISTORY__FULLPOS        (38280)
#define  BRCMLIB__MONITORING_DETAILS_HBM__0_THERMAL_HBM_SEL_TWO_REG_MIN_TEMP_HISTORY__FULLWID        (8)

// monitoring_details.hbm[0].thermal.hbm_sel_two_reg.max_temp_history
#define  BRCMLIB__MONITORING_DETAILS_HBM__0_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_HISTORY__BYTES          (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__0_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_HISTORY__COUNT          (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__0_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_HISTORY__OFFSET         (4786)
#define  BRCMLIB__MONITORING_DETAILS_HBM__0_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_HISTORY__SIZE           (1)
//#define  BRCMLIB__MONITORING_DETAILS_HBM__0_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_HISTORY__TYPE           (int8_t)
#define  BRCMLIB__MONITORING_DETAILS_HBM__0_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_HISTORY__FULLPOS        (38288)
#define  BRCMLIB__MONITORING_DETAILS_HBM__0_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_HISTORY__FULLWID        (8)

// monitoring_details.hbm[0].thermal.hbm_sel_two_reg.channel_temps
#define  BRCMLIB__MONITORING_DETAILS_HBM__0_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__BYTES             (64)
#define  BRCMLIB__MONITORING_DETAILS_HBM__0_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__COUNT             (64)
#define  BRCMLIB__MONITORING_DETAILS_HBM__0_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__OFFSET            (4787)
#define  BRCMLIB__MONITORING_DETAILS_HBM__0_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__SIZE              (1)
//#define  BRCMLIB__MONITORING_DETAILS_HBM__0_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__TYPE              (int8_t)
#define  BRCMLIB__MONITORING_DETAILS_HBM__0_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__FULLPOS           (38296)
#define  BRCMLIB__MONITORING_DETAILS_HBM__0_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__FULLWID           (512)

// monitoring_details.hbm[1].thermal.hbm_sel_two_reg
#define  BRCMLIB__MONITORING_DETAILS_HBM__1_THERMAL_HBM_SEL_TWO_REG__BYTES                           (68)
#define  BRCMLIB__MONITORING_DETAILS_HBM__1_THERMAL_HBM_SEL_TWO_REG__COUNT                           (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__1_THERMAL_HBM_SEL_TWO_REG__OFFSET                          (5805)
#define  BRCMLIB__MONITORING_DETAILS_HBM__1_THERMAL_HBM_SEL_TWO_REG__SIZE                            (68)
#define  BRCMLIB__MONITORING_DETAILS_HBM__1_THERMAL_HBM_SEL_TWO_REG__FULLPOS                         (46440)
#define  BRCMLIB__MONITORING_DETAILS_HBM__1_THERMAL_HBM_SEL_TWO_REG__FULLWID                         (544)

// monitoring_details.hbm[1].thermal.hbm_sel_two_reg.max_temp_current
#define  BRCMLIB__MONITORING_DETAILS_HBM__1_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__BYTES          (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__1_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__COUNT          (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__1_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__OFFSET         (5805)
#define  BRCMLIB__MONITORING_DETAILS_HBM__1_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__SIZE           (1)
//#define  BRCMLIB__MONITORING_DETAILS_HBM__1_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__TYPE           (int8_t)
#define  BRCMLIB__MONITORING_DETAILS_HBM__1_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__FULLPOS        (46440)
#define  BRCMLIB__MONITORING_DETAILS_HBM__1_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__FULLWID        (8)

// monitoring_details.hbm[1].thermal.hbm_sel_two_reg.junction_temperature
#define  BRCMLIB__MONITORING_DETAILS_HBM__1_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__BYTES      (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__1_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__COUNT      (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__1_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__OFFSET     (5806)
#define  BRCMLIB__MONITORING_DETAILS_HBM__1_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__SIZE       (1)
//#define  BRCMLIB__MONITORING_DETAILS_HBM__1_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__TYPE       (int8_t)
#define  BRCMLIB__MONITORING_DETAILS_HBM__1_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__FULLPOS    (46448)
#define  BRCMLIB__MONITORING_DETAILS_HBM__1_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__FULLWID    (8)

// monitoring_details.hbm[1].thermal.hbm_sel_two_reg.min_temp_history
#define  BRCMLIB__MONITORING_DETAILS_HBM__1_THERMAL_HBM_SEL_TWO_REG_MIN_TEMP_HISTORY__BYTES          (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__1_THERMAL_HBM_SEL_TWO_REG_MIN_TEMP_HISTORY__COUNT          (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__1_THERMAL_HBM_SEL_TWO_REG_MIN_TEMP_HISTORY__OFFSET         (5807)
#define  BRCMLIB__MONITORING_DETAILS_HBM__1_THERMAL_HBM_SEL_TWO_REG_MIN_TEMP_HISTORY__SIZE           (1)
//#define  BRCMLIB__MONITORING_DETAILS_HBM__1_THERMAL_HBM_SEL_TWO_REG_MIN_TEMP_HISTORY__TYPE           (int8_t)
#define  BRCMLIB__MONITORING_DETAILS_HBM__1_THERMAL_HBM_SEL_TWO_REG_MIN_TEMP_HISTORY__FULLPOS        (46456)
#define  BRCMLIB__MONITORING_DETAILS_HBM__1_THERMAL_HBM_SEL_TWO_REG_MIN_TEMP_HISTORY__FULLWID        (8)

// monitoring_details.hbm[1].thermal.hbm_sel_two_reg.max_temp_history
#define  BRCMLIB__MONITORING_DETAILS_HBM__1_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_HISTORY__BYTES          (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__1_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_HISTORY__COUNT          (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__1_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_HISTORY__OFFSET         (5808)
#define  BRCMLIB__MONITORING_DETAILS_HBM__1_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_HISTORY__SIZE           (1)
//#define  BRCMLIB__MONITORING_DETAILS_HBM__1_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_HISTORY__TYPE           (int8_t)
#define  BRCMLIB__MONITORING_DETAILS_HBM__1_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_HISTORY__FULLPOS        (46464)
#define  BRCMLIB__MONITORING_DETAILS_HBM__1_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_HISTORY__FULLWID        (8)

// monitoring_details.hbm[1].thermal.hbm_sel_two_reg.channel_temps
#define  BRCMLIB__MONITORING_DETAILS_HBM__1_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__BYTES             (64)
#define  BRCMLIB__MONITORING_DETAILS_HBM__1_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__COUNT             (64)
#define  BRCMLIB__MONITORING_DETAILS_HBM__1_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__OFFSET            (5809)
#define  BRCMLIB__MONITORING_DETAILS_HBM__1_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__SIZE              (1)
//#define  BRCMLIB__MONITORING_DETAILS_HBM__1_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__TYPE              (int8_t)
#define  BRCMLIB__MONITORING_DETAILS_HBM__1_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__FULLPOS           (46472)
#define  BRCMLIB__MONITORING_DETAILS_HBM__1_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__FULLWID           (512)

// monitoring_details.hbm[2].thermal.hbm_sel_two_reg
#define  BRCMLIB__MONITORING_DETAILS_HBM__2_THERMAL_HBM_SEL_TWO_REG__BYTES                           (68)
#define  BRCMLIB__MONITORING_DETAILS_HBM__2_THERMAL_HBM_SEL_TWO_REG__COUNT                           (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__2_THERMAL_HBM_SEL_TWO_REG__OFFSET                          (6827)
#define  BRCMLIB__MONITORING_DETAILS_HBM__2_THERMAL_HBM_SEL_TWO_REG__SIZE                            (68)
#define  BRCMLIB__MONITORING_DETAILS_HBM__2_THERMAL_HBM_SEL_TWO_REG__FULLPOS                         (54616)
#define  BRCMLIB__MONITORING_DETAILS_HBM__2_THERMAL_HBM_SEL_TWO_REG__FULLWID                         (544)

// monitoring_details.hbm[2].thermal.hbm_sel_two_reg.max_temp_current
#define  BRCMLIB__MONITORING_DETAILS_HBM__2_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__BYTES          (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__2_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__COUNT          (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__2_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__OFFSET         (6827)
#define  BRCMLIB__MONITORING_DETAILS_HBM__2_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__SIZE           (1)
//#define  BRCMLIB__MONITORING_DETAILS_HBM__2_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__TYPE           (int8_t)
#define  BRCMLIB__MONITORING_DETAILS_HBM__2_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__FULLPOS        (54616)
#define  BRCMLIB__MONITORING_DETAILS_HBM__2_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__FULLWID        (8)

// monitoring_details.hbm[2].thermal.hbm_sel_two_reg.junction_temperature
#define  BRCMLIB__MONITORING_DETAILS_HBM__2_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__BYTES      (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__2_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__COUNT      (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__2_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__OFFSET     (6828)
#define  BRCMLIB__MONITORING_DETAILS_HBM__2_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__SIZE       (1)
//#define  BRCMLIB__MONITORING_DETAILS_HBM__2_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__TYPE       (int8_t)
#define  BRCMLIB__MONITORING_DETAILS_HBM__2_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__FULLPOS    (54624)
#define  BRCMLIB__MONITORING_DETAILS_HBM__2_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__FULLWID    (8)

// monitoring_details.hbm[2].thermal.hbm_sel_two_reg.min_temp_history
#define  BRCMLIB__MONITORING_DETAILS_HBM__2_THERMAL_HBM_SEL_TWO_REG_MIN_TEMP_HISTORY__BYTES          (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__2_THERMAL_HBM_SEL_TWO_REG_MIN_TEMP_HISTORY__COUNT          (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__2_THERMAL_HBM_SEL_TWO_REG_MIN_TEMP_HISTORY__OFFSET         (6829)
#define  BRCMLIB__MONITORING_DETAILS_HBM__2_THERMAL_HBM_SEL_TWO_REG_MIN_TEMP_HISTORY__SIZE           (1)
//#define  BRCMLIB__MONITORING_DETAILS_HBM__2_THERMAL_HBM_SEL_TWO_REG_MIN_TEMP_HISTORY__TYPE           (int8_t)
#define  BRCMLIB__MONITORING_DETAILS_HBM__2_THERMAL_HBM_SEL_TWO_REG_MIN_TEMP_HISTORY__FULLPOS        (54632)
#define  BRCMLIB__MONITORING_DETAILS_HBM__2_THERMAL_HBM_SEL_TWO_REG_MIN_TEMP_HISTORY__FULLWID        (8)

// monitoring_details.hbm[2].thermal.hbm_sel_two_reg.max_temp_history
#define  BRCMLIB__MONITORING_DETAILS_HBM__2_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_HISTORY__BYTES          (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__2_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_HISTORY__COUNT          (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__2_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_HISTORY__OFFSET         (6830)
#define  BRCMLIB__MONITORING_DETAILS_HBM__2_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_HISTORY__SIZE           (1)
//#define  BRCMLIB__MONITORING_DETAILS_HBM__2_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_HISTORY__TYPE           (int8_t)
#define  BRCMLIB__MONITORING_DETAILS_HBM__2_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_HISTORY__FULLPOS        (54640)
#define  BRCMLIB__MONITORING_DETAILS_HBM__2_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_HISTORY__FULLWID        (8)

// monitoring_details.hbm[2].thermal.hbm_sel_two_reg.channel_temps
#define  BRCMLIB__MONITORING_DETAILS_HBM__2_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__BYTES             (64)
#define  BRCMLIB__MONITORING_DETAILS_HBM__2_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__COUNT             (64)
#define  BRCMLIB__MONITORING_DETAILS_HBM__2_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__OFFSET            (6831)
#define  BRCMLIB__MONITORING_DETAILS_HBM__2_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__SIZE              (1)
//#define  BRCMLIB__MONITORING_DETAILS_HBM__2_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__TYPE              (int8_t)
#define  BRCMLIB__MONITORING_DETAILS_HBM__2_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__FULLPOS           (54648)
#define  BRCMLIB__MONITORING_DETAILS_HBM__2_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__FULLWID           (512)

// monitoring_details.hbm[3].thermal.hbm_sel_two_reg
#define  BRCMLIB__MONITORING_DETAILS_HBM__3_THERMAL_HBM_SEL_TWO_REG__BYTES                           (68)
#define  BRCMLIB__MONITORING_DETAILS_HBM__3_THERMAL_HBM_SEL_TWO_REG__COUNT                           (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__3_THERMAL_HBM_SEL_TWO_REG__OFFSET                          (7849)
#define  BRCMLIB__MONITORING_DETAILS_HBM__3_THERMAL_HBM_SEL_TWO_REG__SIZE                            (68)
#define  BRCMLIB__MONITORING_DETAILS_HBM__3_THERMAL_HBM_SEL_TWO_REG__FULLPOS                         (62792)
#define  BRCMLIB__MONITORING_DETAILS_HBM__3_THERMAL_HBM_SEL_TWO_REG__FULLWID                         (544)

// monitoring_details.hbm[3].thermal.hbm_sel_two_reg.max_temp_current
#define  BRCMLIB__MONITORING_DETAILS_HBM__3_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__BYTES          (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__3_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__COUNT          (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__3_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__OFFSET         (7849)
#define  BRCMLIB__MONITORING_DETAILS_HBM__3_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__SIZE           (1)
//#define  BRCMLIB__MONITORING_DETAILS_HBM__3_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__TYPE           (int8_t)
#define  BRCMLIB__MONITORING_DETAILS_HBM__3_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__FULLPOS        (62792)
#define  BRCMLIB__MONITORING_DETAILS_HBM__3_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__FULLWID        (8)

// monitoring_details.hbm[3].thermal.hbm_sel_two_reg.junction_temperature
#define  BRCMLIB__MONITORING_DETAILS_HBM__3_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__BYTES      (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__3_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__COUNT      (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__3_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__OFFSET     (7850)
#define  BRCMLIB__MONITORING_DETAILS_HBM__3_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__SIZE       (1)
//#define  BRCMLIB__MONITORING_DETAILS_HBM__3_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__TYPE       (int8_t)
#define  BRCMLIB__MONITORING_DETAILS_HBM__3_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__FULLPOS    (62800)
#define  BRCMLIB__MONITORING_DETAILS_HBM__3_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__FULLWID    (8)

// monitoring_details.hbm[3].thermal.hbm_sel_two_reg.min_temp_history
#define  BRCMLIB__MONITORING_DETAILS_HBM__3_THERMAL_HBM_SEL_TWO_REG_MIN_TEMP_HISTORY__BYTES          (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__3_THERMAL_HBM_SEL_TWO_REG_MIN_TEMP_HISTORY__COUNT          (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__3_THERMAL_HBM_SEL_TWO_REG_MIN_TEMP_HISTORY__OFFSET         (7851)
#define  BRCMLIB__MONITORING_DETAILS_HBM__3_THERMAL_HBM_SEL_TWO_REG_MIN_TEMP_HISTORY__SIZE           (1)
//#define  BRCMLIB__MONITORING_DETAILS_HBM__3_THERMAL_HBM_SEL_TWO_REG_MIN_TEMP_HISTORY__TYPE           (int8_t)
#define  BRCMLIB__MONITORING_DETAILS_HBM__3_THERMAL_HBM_SEL_TWO_REG_MIN_TEMP_HISTORY__FULLPOS        (62808)
#define  BRCMLIB__MONITORING_DETAILS_HBM__3_THERMAL_HBM_SEL_TWO_REG_MIN_TEMP_HISTORY__FULLWID        (8)

// monitoring_details.hbm[3].thermal.hbm_sel_two_reg.max_temp_history
#define  BRCMLIB__MONITORING_DETAILS_HBM__3_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_HISTORY__BYTES          (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__3_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_HISTORY__COUNT          (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__3_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_HISTORY__OFFSET         (7852)
#define  BRCMLIB__MONITORING_DETAILS_HBM__3_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_HISTORY__SIZE           (1)
//#define  BRCMLIB__MONITORING_DETAILS_HBM__3_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_HISTORY__TYPE           (int8_t)
#define  BRCMLIB__MONITORING_DETAILS_HBM__3_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_HISTORY__FULLPOS        (62816)
#define  BRCMLIB__MONITORING_DETAILS_HBM__3_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_HISTORY__FULLWID        (8)

// monitoring_details.hbm[3].thermal.hbm_sel_two_reg.channel_temps
#define  BRCMLIB__MONITORING_DETAILS_HBM__3_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__BYTES             (64)
#define  BRCMLIB__MONITORING_DETAILS_HBM__3_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__COUNT             (64)
#define  BRCMLIB__MONITORING_DETAILS_HBM__3_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__OFFSET            (7853)
#define  BRCMLIB__MONITORING_DETAILS_HBM__3_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__SIZE              (1)
//#define  BRCMLIB__MONITORING_DETAILS_HBM__3_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__TYPE              (int8_t)
#define  BRCMLIB__MONITORING_DETAILS_HBM__3_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__FULLPOS           (62824)
#define  BRCMLIB__MONITORING_DETAILS_HBM__3_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__FULLWID           (512)

// monitoring_details.hbm[4].thermal.hbm_sel_two_reg
#define  BRCMLIB__MONITORING_DETAILS_HBM__4_THERMAL_HBM_SEL_TWO_REG__BYTES                           (68)
#define  BRCMLIB__MONITORING_DETAILS_HBM__4_THERMAL_HBM_SEL_TWO_REG__COUNT                           (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__4_THERMAL_HBM_SEL_TWO_REG__OFFSET                          (8871)
#define  BRCMLIB__MONITORING_DETAILS_HBM__4_THERMAL_HBM_SEL_TWO_REG__SIZE                            (68)
#define  BRCMLIB__MONITORING_DETAILS_HBM__4_THERMAL_HBM_SEL_TWO_REG__FULLPOS                         (70968)
#define  BRCMLIB__MONITORING_DETAILS_HBM__4_THERMAL_HBM_SEL_TWO_REG__FULLWID                         (544)

// monitoring_details.hbm[4].thermal.hbm_sel_two_reg.max_temp_current
#define  BRCMLIB__MONITORING_DETAILS_HBM__4_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__BYTES          (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__4_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__COUNT          (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__4_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__OFFSET         (8871)
#define  BRCMLIB__MONITORING_DETAILS_HBM__4_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__SIZE           (1)
//#define  BRCMLIB__MONITORING_DETAILS_HBM__4_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__TYPE           (int8_t)
#define  BRCMLIB__MONITORING_DETAILS_HBM__4_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__FULLPOS        (70968)
#define  BRCMLIB__MONITORING_DETAILS_HBM__4_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__FULLWID        (8)

// monitoring_details.hbm[4].thermal.hbm_sel_two_reg.junction_temperature
#define  BRCMLIB__MONITORING_DETAILS_HBM__4_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__BYTES      (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__4_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__COUNT      (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__4_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__OFFSET     (8872)
#define  BRCMLIB__MONITORING_DETAILS_HBM__4_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__SIZE       (1)
//#define  BRCMLIB__MONITORING_DETAILS_HBM__4_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__TYPE       (int8_t)
#define  BRCMLIB__MONITORING_DETAILS_HBM__4_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__FULLPOS    (70976)
#define  BRCMLIB__MONITORING_DETAILS_HBM__4_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__FULLWID    (8)

// monitoring_details.hbm[4].thermal.hbm_sel_two_reg.min_temp_history
#define  BRCMLIB__MONITORING_DETAILS_HBM__4_THERMAL_HBM_SEL_TWO_REG_MIN_TEMP_HISTORY__BYTES          (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__4_THERMAL_HBM_SEL_TWO_REG_MIN_TEMP_HISTORY__COUNT          (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__4_THERMAL_HBM_SEL_TWO_REG_MIN_TEMP_HISTORY__OFFSET         (8873)
#define  BRCMLIB__MONITORING_DETAILS_HBM__4_THERMAL_HBM_SEL_TWO_REG_MIN_TEMP_HISTORY__SIZE           (1)
//#define  BRCMLIB__MONITORING_DETAILS_HBM__4_THERMAL_HBM_SEL_TWO_REG_MIN_TEMP_HISTORY__TYPE           (int8_t)
#define  BRCMLIB__MONITORING_DETAILS_HBM__4_THERMAL_HBM_SEL_TWO_REG_MIN_TEMP_HISTORY__FULLPOS        (70984)
#define  BRCMLIB__MONITORING_DETAILS_HBM__4_THERMAL_HBM_SEL_TWO_REG_MIN_TEMP_HISTORY__FULLWID        (8)

// monitoring_details.hbm[4].thermal.hbm_sel_two_reg.max_temp_history
#define  BRCMLIB__MONITORING_DETAILS_HBM__4_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_HISTORY__BYTES          (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__4_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_HISTORY__COUNT          (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__4_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_HISTORY__OFFSET         (8874)
#define  BRCMLIB__MONITORING_DETAILS_HBM__4_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_HISTORY__SIZE           (1)
//#define  BRCMLIB__MONITORING_DETAILS_HBM__4_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_HISTORY__TYPE           (int8_t)
#define  BRCMLIB__MONITORING_DETAILS_HBM__4_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_HISTORY__FULLPOS        (70992)
#define  BRCMLIB__MONITORING_DETAILS_HBM__4_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_HISTORY__FULLWID        (8)

// monitoring_details.hbm[4].thermal.hbm_sel_two_reg.channel_temps
#define  BRCMLIB__MONITORING_DETAILS_HBM__4_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__BYTES             (64)
#define  BRCMLIB__MONITORING_DETAILS_HBM__4_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__COUNT             (64)
#define  BRCMLIB__MONITORING_DETAILS_HBM__4_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__OFFSET            (8875)
#define  BRCMLIB__MONITORING_DETAILS_HBM__4_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__SIZE              (1)
//#define  BRCMLIB__MONITORING_DETAILS_HBM__4_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__TYPE              (int8_t)
#define  BRCMLIB__MONITORING_DETAILS_HBM__4_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__FULLPOS           (71000)
#define  BRCMLIB__MONITORING_DETAILS_HBM__4_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__FULLWID           (512)

// monitoring_details.hbm[5].thermal.hbm_sel_two_reg
#define  BRCMLIB__MONITORING_DETAILS_HBM__5_THERMAL_HBM_SEL_TWO_REG__BYTES                           (68)
#define  BRCMLIB__MONITORING_DETAILS_HBM__5_THERMAL_HBM_SEL_TWO_REG__COUNT                           (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__5_THERMAL_HBM_SEL_TWO_REG__OFFSET                          (9893)
#define  BRCMLIB__MONITORING_DETAILS_HBM__5_THERMAL_HBM_SEL_TWO_REG__SIZE                            (68)
#define  BRCMLIB__MONITORING_DETAILS_HBM__5_THERMAL_HBM_SEL_TWO_REG__FULLPOS                         (79144)
#define  BRCMLIB__MONITORING_DETAILS_HBM__5_THERMAL_HBM_SEL_TWO_REG__FULLWID                         (544)

// monitoring_details.hbm[5].thermal.hbm_sel_two_reg.max_temp_current
#define  BRCMLIB__MONITORING_DETAILS_HBM__5_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__BYTES          (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__5_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__COUNT          (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__5_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__OFFSET         (9893)
#define  BRCMLIB__MONITORING_DETAILS_HBM__5_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__SIZE           (1)
//#define  BRCMLIB__MONITORING_DETAILS_HBM__5_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__TYPE           (int8_t)
#define  BRCMLIB__MONITORING_DETAILS_HBM__5_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__FULLPOS        (79144)
#define  BRCMLIB__MONITORING_DETAILS_HBM__5_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_CURRENT__FULLWID        (8)

// monitoring_details.hbm[5].thermal.hbm_sel_two_reg.junction_temperature
#define  BRCMLIB__MONITORING_DETAILS_HBM__5_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__BYTES      (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__5_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__COUNT      (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__5_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__OFFSET     (9894)
#define  BRCMLIB__MONITORING_DETAILS_HBM__5_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__SIZE       (1)
//#define  BRCMLIB__MONITORING_DETAILS_HBM__5_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__TYPE       (int8_t)
#define  BRCMLIB__MONITORING_DETAILS_HBM__5_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__FULLPOS    (79152)
#define  BRCMLIB__MONITORING_DETAILS_HBM__5_THERMAL_HBM_SEL_TWO_REG_JUNCTION_TEMPERATURE__FULLWID    (8)

// monitoring_details.hbm[5].thermal.hbm_sel_two_reg.min_temp_history
#define  BRCMLIB__MONITORING_DETAILS_HBM__5_THERMAL_HBM_SEL_TWO_REG_MIN_TEMP_HISTORY__BYTES          (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__5_THERMAL_HBM_SEL_TWO_REG_MIN_TEMP_HISTORY__COUNT          (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__5_THERMAL_HBM_SEL_TWO_REG_MIN_TEMP_HISTORY__OFFSET         (9895)
#define  BRCMLIB__MONITORING_DETAILS_HBM__5_THERMAL_HBM_SEL_TWO_REG_MIN_TEMP_HISTORY__SIZE           (1)
//#define  BRCMLIB__MONITORING_DETAILS_HBM__5_THERMAL_HBM_SEL_TWO_REG_MIN_TEMP_HISTORY__TYPE           (int8_t)
#define  BRCMLIB__MONITORING_DETAILS_HBM__5_THERMAL_HBM_SEL_TWO_REG_MIN_TEMP_HISTORY__FULLPOS        (79160)
#define  BRCMLIB__MONITORING_DETAILS_HBM__5_THERMAL_HBM_SEL_TWO_REG_MIN_TEMP_HISTORY__FULLWID        (8)

// monitoring_details.hbm[5].thermal.hbm_sel_two_reg.max_temp_history
#define  BRCMLIB__MONITORING_DETAILS_HBM__5_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_HISTORY__BYTES          (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__5_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_HISTORY__COUNT          (1)
#define  BRCMLIB__MONITORING_DETAILS_HBM__5_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_HISTORY__OFFSET         (9896)
#define  BRCMLIB__MONITORING_DETAILS_HBM__5_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_HISTORY__SIZE           (1)
//#define  BRCMLIB__MONITORING_DETAILS_HBM__5_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_HISTORY__TYPE           (int8_t)
#define  BRCMLIB__MONITORING_DETAILS_HBM__5_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_HISTORY__FULLPOS        (79168)
#define  BRCMLIB__MONITORING_DETAILS_HBM__5_THERMAL_HBM_SEL_TWO_REG_MAX_TEMP_HISTORY__FULLWID        (8)

// monitoring_details.hbm[5].thermal.hbm_sel_two_reg.channel_temps
#define  BRCMLIB__MONITORING_DETAILS_HBM__5_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__BYTES             (64)
#define  BRCMLIB__MONITORING_DETAILS_HBM__5_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__COUNT             (64)
#define  BRCMLIB__MONITORING_DETAILS_HBM__5_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__OFFSET            (9897)
#define  BRCMLIB__MONITORING_DETAILS_HBM__5_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__SIZE              (1)
//#define  BRCMLIB__MONITORING_DETAILS_HBM__5_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__TYPE              (int8_t)
#define  BRCMLIB__MONITORING_DETAILS_HBM__5_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__FULLPOS           (79176)
#define  BRCMLIB__MONITORING_DETAILS_HBM__5_THERMAL_HBM_SEL_TWO_REG_CHANNEL_TEMPS__FULLWID           (512)

// monitoring_details.owl[0].vtmon
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__BYTES                                             (152)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__COUNT                                             (2)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__OFFSET                                            (12328)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__SIZE                                              (76)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__FULLPOS                                           (98624)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__FULLWID                                           (1216)

// monitoring_details.owl[0].vtmon[0]
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0__BYTES                                          (76)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0__COUNT                                          (1)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0__OFFSET                                         (12328)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0__SIZE                                           (76)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0__FULLPOS                                        (98624)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0__FULLWID                                        (608)

// monitoring_details.owl[0].vtmon[0].v3p3
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_V3P3__BYTES                                     (4)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_V3P3__COUNT                                     (1)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_V3P3__OFFSET                                    (12328)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_V3P3__SIZE                                      (4)
//#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_V3P3__TYPE                                      (int32_t)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_V3P3__FULLPOS                                   (98624)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_V3P3__FULLWID                                   (32)

// monitoring_details.owl[0].vtmon[0].v1p8
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_V1P8__BYTES                                     (4)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_V1P8__COUNT                                     (1)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_V1P8__OFFSET                                    (12332)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_V1P8__SIZE                                      (4)
//#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_V1P8__TYPE                                      (int32_t)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_V1P8__FULLPOS                                   (98656)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_V1P8__FULLWID                                   (32)

// monitoring_details.owl[0].vtmon[0].v1p0
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_V1P0__BYTES                                     (24)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_V1P0__COUNT                                     (6)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_V1P0__OFFSET                                    (12336)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_V1P0__SIZE                                      (4)
//#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_V1P0__TYPE                                      (int32_t)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_V1P0__FULLPOS                                   (98688)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_V1P0__FULLWID                                   (192)

// monitoring_details.owl[0].vtmon[0].temp
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_TEMP__BYTES                                     (32)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_TEMP__COUNT                                     (8)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_TEMP__OFFSET                                    (12360)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_TEMP__SIZE                                      (4)
//#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_TEMP__TYPE                                      (int32_t)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_TEMP__FULLPOS                                   (98880)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_TEMP__FULLWID                                   (256)

// monitoring_details.owl[0].vtmon[0].diode
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_DIODE__BYTES                                    (4)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_DIODE__COUNT                                    (1)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_DIODE__OFFSET                                   (12392)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_DIODE__SIZE                                     (4)
//#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_DIODE__TYPE                                     (int32_t)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_DIODE__FULLPOS                                  (99136)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_DIODE__FULLWID                                  (32)

// monitoring_details.owl[0].vtmon[0].vctat
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_VCTAT__BYTES                                    (4)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_VCTAT__COUNT                                    (1)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_VCTAT__OFFSET                                   (12396)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_VCTAT__SIZE                                     (4)
//#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_VCTAT__TYPE                                     (int32_t)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_VCTAT__FULLPOS                                  (99168)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_VCTAT__FULLWID                                  (32)

// monitoring_details.owl[0].vtmon[0].error
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_ERROR__BYTES                                    (4)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_ERROR__COUNT                                    (1)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_ERROR__OFFSET                                   (12400)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_ERROR__SIZE                                     (4)
//#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_ERROR__TYPE                                     (int32_t)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_ERROR__FULLPOS                                  (99200)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__0_ERROR__FULLWID                                  (32)

// monitoring_details.owl[0].vtmon[1]
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1__BYTES                                          (76)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1__COUNT                                          (1)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1__OFFSET                                         (12404)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1__SIZE                                           (76)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1__FULLPOS                                        (99232)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1__FULLWID                                        (608)

// monitoring_details.owl[0].vtmon[1].v3p3
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_V3P3__BYTES                                     (4)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_V3P3__COUNT                                     (1)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_V3P3__OFFSET                                    (12404)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_V3P3__SIZE                                      (4)
//#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_V3P3__TYPE                                      (int32_t)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_V3P3__FULLPOS                                   (99232)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_V3P3__FULLWID                                   (32)

// monitoring_details.owl[0].vtmon[1].v1p8
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_V1P8__BYTES                                     (4)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_V1P8__COUNT                                     (1)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_V1P8__OFFSET                                    (12408)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_V1P8__SIZE                                      (4)
//#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_V1P8__TYPE                                      (int32_t)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_V1P8__FULLPOS                                   (99264)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_V1P8__FULLWID                                   (32)

// monitoring_details.owl[0].vtmon[1].v1p0
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_V1P0__BYTES                                     (24)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_V1P0__COUNT                                     (6)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_V1P0__OFFSET                                    (12412)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_V1P0__SIZE                                      (4)
//#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_V1P0__TYPE                                      (int32_t)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_V1P0__FULLPOS                                   (99296)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_V1P0__FULLWID                                   (192)

// monitoring_details.owl[0].vtmon[1].temp
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_TEMP__BYTES                                     (32)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_TEMP__COUNT                                     (8)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_TEMP__OFFSET                                    (12436)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_TEMP__SIZE                                      (4)
//#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_TEMP__TYPE                                      (int32_t)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_TEMP__FULLPOS                                   (99488)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_TEMP__FULLWID                                   (256)

// monitoring_details.owl[0].vtmon[1].diode
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_DIODE__BYTES                                    (4)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_DIODE__COUNT                                    (1)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_DIODE__OFFSET                                   (12468)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_DIODE__SIZE                                     (4)
//#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_DIODE__TYPE                                     (int32_t)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_DIODE__FULLPOS                                  (99744)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_DIODE__FULLWID                                  (32)

// monitoring_details.owl[0].vtmon[1].vctat
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_VCTAT__BYTES                                    (4)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_VCTAT__COUNT                                    (1)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_VCTAT__OFFSET                                   (12472)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_VCTAT__SIZE                                     (4)
//#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_VCTAT__TYPE                                     (int32_t)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_VCTAT__FULLPOS                                  (99776)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_VCTAT__FULLWID                                  (32)

// monitoring_details.owl[0].vtmon[1].error
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_ERROR__BYTES                                    (4)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_ERROR__COUNT                                    (1)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_ERROR__OFFSET                                   (12476)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_ERROR__SIZE                                     (4)
//#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_ERROR__TYPE                                     (int32_t)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_ERROR__FULLPOS                                  (99808)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_VTMON__1_ERROR__FULLWID                                  (32)

// monitoring_details.owl[0].max_die_temp
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_MAX_DIE_TEMP__BYTES                                      (4)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_MAX_DIE_TEMP__COUNT                                      (1)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_MAX_DIE_TEMP__OFFSET                                     (12480)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_MAX_DIE_TEMP__SIZE                                       (4)
//#define  BRCMLIB__MONITORING_DETAILS_OWL__0_MAX_DIE_TEMP__TYPE                                       (int32_t)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_MAX_DIE_TEMP__FULLPOS                                    (99840)
#define  BRCMLIB__MONITORING_DETAILS_OWL__0_MAX_DIE_TEMP__FULLWID                                    (32)

// monitoring_details.owl[1].vtmon
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__BYTES                                             (152)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__COUNT                                             (2)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__OFFSET                                            (12516)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__SIZE                                              (76)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__FULLPOS                                           (100128)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__FULLWID                                           (1216)

// monitoring_details.owl[1].vtmon[0]
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0__BYTES                                          (76)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0__COUNT                                          (1)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0__OFFSET                                         (12516)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0__SIZE                                           (76)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0__FULLPOS                                        (100128)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0__FULLWID                                        (608)

// monitoring_details.owl[1].vtmon[0].v3p3
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_V3P3__BYTES                                     (4)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_V3P3__COUNT                                     (1)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_V3P3__OFFSET                                    (12516)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_V3P3__SIZE                                      (4)
//#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_V3P3__TYPE                                      (int32_t)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_V3P3__FULLPOS                                   (100128)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_V3P3__FULLWID                                   (32)

// monitoring_details.owl[1].vtmon[0].v1p8
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_V1P8__BYTES                                     (4)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_V1P8__COUNT                                     (1)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_V1P8__OFFSET                                    (12520)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_V1P8__SIZE                                      (4)
//#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_V1P8__TYPE                                      (int32_t)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_V1P8__FULLPOS                                   (100160)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_V1P8__FULLWID                                   (32)

// monitoring_details.owl[1].vtmon[0].v1p0
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_V1P0__BYTES                                     (24)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_V1P0__COUNT                                     (6)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_V1P0__OFFSET                                    (12524)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_V1P0__SIZE                                      (4)
//#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_V1P0__TYPE                                      (int32_t)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_V1P0__FULLPOS                                   (100192)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_V1P0__FULLWID                                   (192)

// monitoring_details.owl[1].vtmon[0].temp
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_TEMP__BYTES                                     (32)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_TEMP__COUNT                                     (8)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_TEMP__OFFSET                                    (12548)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_TEMP__SIZE                                      (4)
//#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_TEMP__TYPE                                      (int32_t)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_TEMP__FULLPOS                                   (100384)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_TEMP__FULLWID                                   (256)

// monitoring_details.owl[1].vtmon[0].diode
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_DIODE__BYTES                                    (4)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_DIODE__COUNT                                    (1)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_DIODE__OFFSET                                   (12580)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_DIODE__SIZE                                     (4)
//#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_DIODE__TYPE                                     (int32_t)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_DIODE__FULLPOS                                  (100640)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_DIODE__FULLWID                                  (32)

// monitoring_details.owl[1].vtmon[0].vctat
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_VCTAT__BYTES                                    (4)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_VCTAT__COUNT                                    (1)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_VCTAT__OFFSET                                   (12584)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_VCTAT__SIZE                                     (4)
//#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_VCTAT__TYPE                                     (int32_t)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_VCTAT__FULLPOS                                  (100672)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_VCTAT__FULLWID                                  (32)

// monitoring_details.owl[1].vtmon[0].error
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_ERROR__BYTES                                    (4)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_ERROR__COUNT                                    (1)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_ERROR__OFFSET                                   (12588)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_ERROR__SIZE                                     (4)
//#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_ERROR__TYPE                                     (int32_t)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_ERROR__FULLPOS                                  (100704)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__0_ERROR__FULLWID                                  (32)

// monitoring_details.owl[1].vtmon[1]
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1__BYTES                                          (76)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1__COUNT                                          (1)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1__OFFSET                                         (12592)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1__SIZE                                           (76)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1__FULLPOS                                        (100736)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1__FULLWID                                        (608)

// monitoring_details.owl[1].vtmon[1].v3p3
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_V3P3__BYTES                                     (4)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_V3P3__COUNT                                     (1)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_V3P3__OFFSET                                    (12592)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_V3P3__SIZE                                      (4)
//#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_V3P3__TYPE                                      (int32_t)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_V3P3__FULLPOS                                   (100736)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_V3P3__FULLWID                                   (32)

// monitoring_details.owl[1].vtmon[1].v1p8
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_V1P8__BYTES                                     (4)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_V1P8__COUNT                                     (1)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_V1P8__OFFSET                                    (12596)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_V1P8__SIZE                                      (4)
//#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_V1P8__TYPE                                      (int32_t)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_V1P8__FULLPOS                                   (100768)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_V1P8__FULLWID                                   (32)

// monitoring_details.owl[1].vtmon[1].v1p0
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_V1P0__BYTES                                     (24)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_V1P0__COUNT                                     (6)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_V1P0__OFFSET                                    (12600)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_V1P0__SIZE                                      (4)
//#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_V1P0__TYPE                                      (int32_t)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_V1P0__FULLPOS                                   (100800)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_V1P0__FULLWID                                   (192)

// monitoring_details.owl[1].vtmon[1].temp
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_TEMP__BYTES                                     (32)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_TEMP__COUNT                                     (8)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_TEMP__OFFSET                                    (12624)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_TEMP__SIZE                                      (4)
//#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_TEMP__TYPE                                      (int32_t)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_TEMP__FULLPOS                                   (100992)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_TEMP__FULLWID                                   (256)

// monitoring_details.owl[1].vtmon[1].diode
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_DIODE__BYTES                                    (4)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_DIODE__COUNT                                    (1)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_DIODE__OFFSET                                   (12656)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_DIODE__SIZE                                     (4)
//#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_DIODE__TYPE                                     (int32_t)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_DIODE__FULLPOS                                  (101248)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_DIODE__FULLWID                                  (32)

// monitoring_details.owl[1].vtmon[1].vctat
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_VCTAT__BYTES                                    (4)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_VCTAT__COUNT                                    (1)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_VCTAT__OFFSET                                   (12660)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_VCTAT__SIZE                                     (4)
//#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_VCTAT__TYPE                                     (int32_t)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_VCTAT__FULLPOS                                  (101280)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_VCTAT__FULLWID                                  (32)

// monitoring_details.owl[1].vtmon[1].error
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_ERROR__BYTES                                    (4)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_ERROR__COUNT                                    (1)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_ERROR__OFFSET                                   (12664)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_ERROR__SIZE                                     (4)
//#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_ERROR__TYPE                                     (int32_t)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_ERROR__FULLPOS                                  (101312)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_VTMON__1_ERROR__FULLWID                                  (32)

// monitoring_details.owl[1].max_die_temp
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_MAX_DIE_TEMP__BYTES                                      (4)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_MAX_DIE_TEMP__COUNT                                      (1)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_MAX_DIE_TEMP__OFFSET                                     (12668)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_MAX_DIE_TEMP__SIZE                                       (4)
//#define  BRCMLIB__MONITORING_DETAILS_OWL__1_MAX_DIE_TEMP__TYPE                                       (int32_t)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_MAX_DIE_TEMP__FULLPOS                                    (101344)
#define  BRCMLIB__MONITORING_DETAILS_OWL__1_MAX_DIE_TEMP__FULLWID                                    (32)

#define   BRCMLIB_SB_STATUS_LENGTH_W_OFF                               (   0) // 0
#define   BRCMLIB_SB_STATUS_LENGTH_W_WID                               (   1) // 8
#define   BRCMLIB_SB_STATUS_FLAGS_W_OFF                                (   1) // 8
#define   BRCMLIB_SB_STATUS_FLAGS_W_WID                                (   1) // 8
//#define BRCMLIB_SB_STATUS_FLAGS_WAITING_HOST_POR_W_OFF               (   1) // 8
//#define BRCMLIB_SB_STATUS_FLAGS_WAITING_HOST_POR_W_WID               (   0) // 1
//#define BRCMLIB_SB_STATUS_RESV_W_OFF                                 (   1) // 9
//#define BRCMLIB_SB_STATUS_RESV_W_WID                                 (   0) // 5
//#define BRCMLIB_SB_STATUS_FLAGS_NOT_READY_W_OFF                      (   1) // 14
//#define BRCMLIB_SB_STATUS_FLAGS_NOT_READY_W_WID                      (   0) // 1
//#define BRCMLIB_SB_STATUS_RESV2_W_OFF                                (   1) // 15
//#define BRCMLIB_SB_STATUS_RESV2_W_WID                                (   0) // 1
#define   BRCMLIB_SB_STATUS_NA2_W_OFF                                  (   2) // 16
#define   BRCMLIB_SB_STATUS_NA2_W_WID                                  (   1) // 8
#define   BRCMLIB_SB_STATUS_TEMP_W_OFF                                 (   3) // 24
#define   BRCMLIB_SB_STATUS_TEMP_W_WID                                 (   1) // 8
#define   BRCMLIB_SB_STATUS_NA4_W_OFF                                  (   4) // 32
#define   BRCMLIB_SB_STATUS_NA4_W_WID                                  (   1) // 8
#define   BRCMLIB_SB_STATUS_RESERVED_W_OFF                             (   5) // 40
#define   BRCMLIB_SB_STATUS_RESERVED_W_WID                             (   2) // 16
#define   BRCMLIB_SB_STATUS_PEC_BYTE_W_OFF                             (   7) // 56
#define   BRCMLIB_SB_STATUS_PEC_BYTE_W_WID                             (   1) // 8
#define   BRCMLIB_SB_ID_LENGTH_W_OFF                                   (   8) // 64
#define   BRCMLIB_SB_ID_LENGTH_W_WID                                   (   1) // 8
#define   BRCMLIB_SB_ID_PCIE_VENDOR_ID_W_OFF                           (   9) // 72
#define   BRCMLIB_SB_ID_PCIE_VENDOR_ID_W_WID                           (   2) // 16
#define   BRCMLIB_SB_ID_PEC_BYTE_W_OFF                                 (  11) // 88
#define   BRCMLIB_SB_ID_PEC_BYTE_W_WID                                 (   1) // 8
#define   BRCMLIB_SB_MODULE_ID_LENGTH_W_OFF                            (  12) // 96
#define   BRCMLIB_SB_MODULE_ID_LENGTH_W_WID                            (   1) // 8
#define   BRCMLIB_SB_MODULE_ID_FB_ID_W_OFF                             (  13) // 104
#define   BRCMLIB_SB_MODULE_ID_FB_ID_W_WID                             (   1) // 8
#define   BRCMLIB_SB_MODULE_ID_MEFF_W_OFF                              (  14) // 112
#define   BRCMLIB_SB_MODULE_ID_MEFF_W_WID                              (   1) // 8
#define   BRCMLIB_SB_MODULE_ID_FFI_0_W_OFF                             (  15) // 120
#define   BRCMLIB_SB_MODULE_ID_FFI_0_W_WID                             (   1) // 8
#define   BRCMLIB_SB_MODULE_ID_RESERVED_W_OFF                          (  16) // 128
#define   BRCMLIB_SB_MODULE_ID_RESERVED_W_WID                          (  10) // 80
#define   BRCMLIB_SB_MODULE_ID_PEC_BYTE_W_OFF                          (  26) // 208
#define   BRCMLIB_SB_MODULE_ID_PEC_BYTE_W_WID                          (   1) // 8
#define   BRCMLIB_SB_MODULE_STATUS_LENGTH_W_OFF                        (  27) // 216
#define   BRCMLIB_SB_MODULE_STATUS_LENGTH_W_WID                        (   1) // 8
//#define BRCMLIB_SB_MODULE_STATUS_ASIC_ERR_TYPE_W_OFF                 (  28) // 224
//#define BRCMLIB_SB_MODULE_STATUS_ASIC_ERR_TYPE_W_WID                 (   0) // 1
#define   BRCMLIB_SB_MODULE_STATUS_HEALTH_W_OFF                        (  28) // 224
#define   BRCMLIB_SB_MODULE_STATUS_HEALTH_W_WID                        (   1) // 8
//#define BRCMLIB_SB_MODULE_STATUS_MOD_ERR_TYPE_W_OFF                  (  28) // 225
//#define BRCMLIB_SB_MODULE_STATUS_MOD_ERR_TYPE_W_WID                  (   0) // 1
//#define BRCMLIB_SB_MODULE_STATUS_WARN_FLAG_W_OFF                     (  28) // 226
//#define BRCMLIB_SB_MODULE_STATUS_WARN_FLAG_W_WID                     (   0) // 1
//#define BRCMLIB_SB_MODULE_STATUS_INT_FLAG_W_OFF                      (  28) // 227
//#define BRCMLIB_SB_MODULE_STATUS_INT_FLAG_W_WID                      (   0) // 1
//#define BRCMLIB_SB_MODULE_STATUS_MAX_ASIC_TEMP_W_OFF                 (  28) // 228
//#define BRCMLIB_SB_MODULE_STATUS_MAX_ASIC_TEMP_W_WID                 (   0) // 1
//#define BRCMLIB_SB_MODULE_STATUS_INT_MEM_ERR_CNT_W_OFF               (  28) // 229
//#define BRCMLIB_SB_MODULE_STATUS_INT_MEM_ERR_CNT_W_WID               (   0) // 1
//#define BRCMLIB_SB_MODULE_STATUS_EXT_MEM_ERR_CNT_W_OFF               (  28) // 230
//#define BRCMLIB_SB_MODULE_STATUS_EXT_MEM_ERR_CNT_W_WID               (   0) // 1
//#define BRCMLIB_SB_MODULE_STATUS_SMBUS_ERR_W_OFF                     (  28) // 231
//#define BRCMLIB_SB_MODULE_STATUS_SMBUS_ERR_W_WID                     (   0) // 1
#define   BRCMLIB_SB_MODULE_STATUS_LOW_TEMP_THRES_W_OFF                (  29) // 232
#define   BRCMLIB_SB_MODULE_STATUS_LOW_TEMP_THRES_W_WID                (   1) // 8
#define   BRCMLIB_SB_MODULE_STATUS_HIGH_TEMP_THRES_W_OFF               (  30) // 240
#define   BRCMLIB_SB_MODULE_STATUS_HIGH_TEMP_THRES_W_WID               (   1) // 8
#define   BRCMLIB_SB_MODULE_STATUS_POWER_STATE_W_OFF                   (  31) // 248
#define   BRCMLIB_SB_MODULE_STATUS_POWER_STATE_W_WID                   (   1) // 8
#define   BRCMLIB_SB_MODULE_STATUS_SMBUS_I2C_FREQ_W_OFF                (  32) // 256
#define   BRCMLIB_SB_MODULE_STATUS_SMBUS_I2C_FREQ_W_WID                (   1) // 8
#define   BRCMLIB_SB_MODULE_STATUS_STATIC_TDP_W_OFF                    (  33) // 264
#define   BRCMLIB_SB_MODULE_STATUS_STATIC_TDP_W_WID                    (   1) // 8
#define   BRCMLIB_SB_MODULE_STATUS_PEC_BYTE_W_OFF                      (  34) // 272
#define   BRCMLIB_SB_MODULE_STATUS_PEC_BYTE_W_WID                      (   1) // 8
#define   BRCMLIB_SB_VERSIONS_LENGTH_W_OFF                             (  35) // 280
#define   BRCMLIB_SB_VERSIONS_LENGTH_W_WID                             (   1) // 8
#define   BRCMLIB_SB_VERSIONS_ASIC_W_OFF                               (  36) // 288
#define   BRCMLIB_SB_VERSIONS_ASIC_W_WID                               (   1) // 8
#define   BRCMLIB_SB_VERSIONS_M7_MAJOR_W_OFF                           (  37) // 296
#define   BRCMLIB_SB_VERSIONS_M7_MAJOR_W_WID                           (   1) // 8
#define   BRCMLIB_SB_VERSIONS_M7_MINOR_W_OFF                           (  38) // 304
#define   BRCMLIB_SB_VERSIONS_M7_MINOR_W_WID                           (   1) // 8
#define   BRCMLIB_SB_VERSIONS_M7_OPTIONAL_W_OFF                        (  39) // 312
#define   BRCMLIB_SB_VERSIONS_M7_OPTIONAL_W_WID                        (   1) // 8
#define   BRCMLIB_SB_VERSIONS_UNUSED_W_OFF                             (  40) // 320
#define   BRCMLIB_SB_VERSIONS_UNUSED_W_WID                             (   1) // 8
#define   BRCMLIB_SB_VERSIONS_PATCH_MAJOR_W_OFF                        (  41) // 328
#define   BRCMLIB_SB_VERSIONS_PATCH_MAJOR_W_WID                        (   1) // 8
#define   BRCMLIB_SB_VERSIONS_PEC_BYTE_W_OFF                           (  42) // 336
#define   BRCMLIB_SB_VERSIONS_PEC_BYTE_W_WID                           (   1) // 8
#define   BRCMLIB_SB_MONITOR_LENGTH_W_OFF                              (  43) // 344
#define   BRCMLIB_SB_MONITOR_LENGTH_W_WID                              (   1) // 8
#define   BRCMLIB_SB_MONITOR_RT_ASIC_CORE_VOLT1_W_OFF                  (  44) // 352
#define   BRCMLIB_SB_MONITOR_RT_ASIC_CORE_VOLT1_W_WID                  (   2) // 16
#define   BRCMLIB_SB_MONITOR_RT_ASIC_CORE_VOLT2_W_OFF                  (  46) // 368
#define   BRCMLIB_SB_MONITOR_RT_ASIC_CORE_VOLT2_W_WID                  (   2) // 16
#define   BRCMLIB_SB_MONITOR_RT_RAIL1_VOLT_W_OFF                       (  48) // 384
#define   BRCMLIB_SB_MONITOR_RT_RAIL1_VOLT_W_WID                       (   2) // 16
#define   BRCMLIB_SB_MONITOR_RT_RAIL2_VOLT_W_OFF                       (  50) // 400
#define   BRCMLIB_SB_MONITOR_RT_RAIL2_VOLT_W_WID                       (   2) // 16
#define   BRCMLIB_SB_MONITOR_PEC_BYTE_W_OFF                            (  52) // 416
#define   BRCMLIB_SB_MONITOR_PEC_BYTE_W_WID                            (   1) // 8
#define   BRCMLIB_SB_ERROR_REPORT_LENGTH_W_OFF                         (  53) // 424
#define   BRCMLIB_SB_ERROR_REPORT_LENGTH_W_WID                         (   1) // 8
#define   BRCMLIB_SB_ERROR_REPORT_ASIC_ERROR_TYPE_W_OFF                (  54) // 432
#define   BRCMLIB_SB_ERROR_REPORT_ASIC_ERROR_TYPE_W_WID                (   1) // 8
#define   BRCMLIB_SB_ERROR_REPORT_MODULE_ERROR_TYPE_W_OFF              (  55) // 440
#define   BRCMLIB_SB_ERROR_REPORT_MODULE_ERROR_TYPE_W_WID              (   1) // 8
#define   BRCMLIB_SB_ERROR_REPORT_WARNING_FLAG_W_OFF                   (  56) // 448
#define   BRCMLIB_SB_ERROR_REPORT_WARNING_FLAG_W_WID                   (   1) // 8
#define   BRCMLIB_SB_ERROR_REPORT_INTERRUPT_FLAG_W_OFF                 (  57) // 456
#define   BRCMLIB_SB_ERROR_REPORT_INTERRUPT_FLAG_W_WID                 (   1) // 8
#define   BRCMLIB_SB_ERROR_REPORT_MAX_ASIC_TEMP_W_OFF                  (  58) // 464
#define   BRCMLIB_SB_ERROR_REPORT_MAX_ASIC_TEMP_W_WID                  (   1) // 8
#define   BRCMLIB_SB_ERROR_REPORT_INT_MEM_ERR_CNT_W_OFF                (  59) // 472
#define   BRCMLIB_SB_ERROR_REPORT_INT_MEM_ERR_CNT_W_WID                (   1) // 8
#define   BRCMLIB_SB_ERROR_REPORT_EXT_MEM_ERR_CNT_W_OFF                (  60) // 480
#define   BRCMLIB_SB_ERROR_REPORT_EXT_MEM_ERR_CNT_W_WID                (   1) // 8
#define   BRCMLIB_SB_ERROR_REPORT_SMBUS_ERROR_W_OFF                    (  61) // 488
#define   BRCMLIB_SB_ERROR_REPORT_SMBUS_ERROR_W_WID                    (   1) // 8
#define   BRCMLIB_SB_ERROR_REPORT_PEC_BYTE_W_OFF                       (  62) // 496
#define   BRCMLIB_SB_ERROR_REPORT_PEC_BYTE_W_WID                       (   1) // 8
#define   BRCMLIB_SB_OWLN_LENGTH_W_OFF                                 (  63) // 504
#define   BRCMLIB_SB_OWLN_LENGTH_W_WID                                 (   1) // 8
#define   BRCMLIB_SB_OWLN_RESV1_W_OFF                                  (  64) // 512
#define   BRCMLIB_SB_OWLN_RESV1_W_WID                                  (   1) // 8
#define   BRCMLIB_SB_OWLN_M7_MAJOR_W_OFF                               (  65) // 520
#define   BRCMLIB_SB_OWLN_M7_MAJOR_W_WID                               (   1) // 8
#define   BRCMLIB_SB_OWLN_M7_MINOR_W_OFF                               (  66) // 528
#define   BRCMLIB_SB_OWLN_M7_MINOR_W_WID                               (   1) // 8
#define   BRCMLIB_SB_OWLN_OWL_CONFIG_W_OFF                             (  67) // 536
#define   BRCMLIB_SB_OWLN_OWL_CONFIG_W_WID                             (   4) // 32
//#define BRCMLIB_SB_OWLN_OWL_IDX_W_OFF                                (  67) // 536
//#define BRCMLIB_SB_OWLN_OWL_IDX_W_WID                                (   0) // 4
//#define BRCMLIB_SB_OWLN_NUM_T3CS_W_OFF                               (  67) // 540
//#define BRCMLIB_SB_OWLN_NUM_T3CS_W_WID                               (   0) // 4
//#define BRCMLIB_SB_OWLN_T3C0_PFS_W_OFF                               (  68) // 544
//#define BRCMLIB_SB_OWLN_T3C0_PFS_W_WID                               (   0) // 4
//#define BRCMLIB_SB_OWLN_T3C1_PFS_W_OFF                               (  68) // 548
//#define BRCMLIB_SB_OWLN_T3C1_PFS_W_WID                               (   0) // 4
//#define BRCMLIB_SB_OWLN_T3C2_PFS_W_OFF                               (  69) // 552
//#define BRCMLIB_SB_OWLN_T3C2_PFS_W_WID                               (   0) // 4
//#define BRCMLIB_SB_OWLN_T3C3_PFS_W_OFF                               (  69) // 556
//#define BRCMLIB_SB_OWLN_T3C3_PFS_W_WID                               (   0) // 4
//#define BRCMLIB_SB_OWLN_T3C4_PFS_W_OFF                               (  70) // 560
//#define BRCMLIB_SB_OWLN_T3C4_PFS_W_WID                               (   0) // 4
//#define BRCMLIB_SB_OWLN_T3C5_PFS_W_OFF                               (  70) // 564
//#define BRCMLIB_SB_OWLN_T3C5_PFS_W_WID                               (   0) // 4
#define   BRCMLIB_SB_OWLN_BOOT_SRC_W_OFF                               (  71) // 568
#define   BRCMLIB_SB_OWLN_BOOT_SRC_W_WID                               (   1) // 8
#define   BRCMLIB_SB_OWLN_BOOT_STATUS1_W_OFF                           (  71) // 568
#define   BRCMLIB_SB_OWLN_BOOT_STATUS1_W_WID                           (   4) // 32
//#define BRCMLIB_SB_OWLN_MEM_REPAIR_REI_FAIL_W_OFF                    (  72) // 576
//#define BRCMLIB_SB_OWLN_MEM_REPAIR_REI_FAIL_W_WID                    (   0) // 1
//#define BRCMLIB_SB_OWLN_MEM_REPAIR_REI_PASS_W_OFF                    (  72) // 577
//#define BRCMLIB_SB_OWLN_MEM_REPAIR_REI_PASS_W_WID                    (   0) // 1
//#define BRCMLIB_SB_OWLN_MEM_REPAIR_MBISR_DONE_W_OFF                  (  72) // 578
//#define BRCMLIB_SB_OWLN_MEM_REPAIR_MBISR_DONE_W_WID                  (   0) // 1
//#define BRCMLIB_SB_OWLN_MEM_REPAIR_MBISR_GO_W_OFF                    (  72) // 579
//#define BRCMLIB_SB_OWLN_MEM_REPAIR_MBISR_GO_W_WID                    (   0) // 1
//#define BRCMLIB_SB_OWLN_MEM_REPAIR_START_W_OFF                       (  72) // 580
//#define BRCMLIB_SB_OWLN_MEM_REPAIR_START_W_WID                       (   0) // 1
//#define BRCMLIB_SB_OWLN_MEM_REPAIR_TIMEOUT_W_OFF                     (  72) // 581
//#define BRCMLIB_SB_OWLN_MEM_REPAIR_TIMEOUT_W_WID                     (   0) // 1
//#define BRCMLIB_SB_OWLN_FSM_BISR_DONE_W_OFF                          (  72) // 582
//#define BRCMLIB_SB_OWLN_FSM_BISR_DONE_W_WID                          (   0) // 1
//#define BRCMLIB_SB_OWLN_FSM_BISR_GO_W_OFF                            (  72) // 583
//#define BRCMLIB_SB_OWLN_FSM_BISR_GO_W_WID                            (   0) // 1
//#define BRCMLIB_SB_OWLN_FSM_MEM_MAST_PASS_W_OFF                      (  73) // 584
//#define BRCMLIB_SB_OWLN_FSM_MEM_MAST_PASS_W_WID                      (   0) // 1
//#define BRCMLIB_SB_OWLN_FSM_MEM_MAST_FAIL_W_OFF                      (  73) // 585
//#define BRCMLIB_SB_OWLN_FSM_MEM_MAST_FAIL_W_WID                      (   0) // 1
//#define BRCMLIB_SB_OWLN_UART_INITIALIZE_W_OFF                        (  73) // 586
//#define BRCMLIB_SB_OWLN_UART_INITIALIZE_W_WID                        (   0) // 1
//#define BRCMLIB_SB_OWLN_UART_RX_LOCKED_W_OFF                         (  73) // 587
//#define BRCMLIB_SB_OWLN_UART_RX_LOCKED_W_WID                         (   0) // 1
//#define BRCMLIB_SB_OWLN_LCS_W_OFF                                    (  73) // 588
//#define BRCMLIB_SB_OWLN_LCS_W_WID                                    (   0) // 4
//#define BRCMLIB_SB_OWLN_SYS_PLL_LOCKED_W_OFF                         (  74) // 592
//#define BRCMLIB_SB_OWLN_SYS_PLL_LOCKED_W_WID                         (   0) // 1
//#define BRCMLIB_SB_OWLN_I2C_MASTER_W_OFF                             (  74) // 593
//#define BRCMLIB_SB_OWLN_I2C_MASTER_W_WID                             (   0) // 1
//#define BRCMLIB_SB_OWLN_I2C_SLAVE_W_OFF                              (  74) // 594
//#define BRCMLIB_SB_OWLN_I2C_SLAVE_W_WID                              (   0) // 1
//#define BRCMLIB_SB_OWLN_RESV3_W_OFF                                  (  74) // 595
//#define BRCMLIB_SB_OWLN_RESV3_W_WID                                  (   0) // 1
//#define BRCMLIB_SB_OWLN_MAX2_PLL_W_OFF                               (  74) // 596
//#define BRCMLIB_SB_OWLN_MAX2_PLL_W_WID                               (   0) // 1
//#define BRCMLIB_SB_OWLN_MAX2_PHY_W_OFF                               (  74) // 597
//#define BRCMLIB_SB_OWLN_MAX2_PHY_W_WID                               (   0) // 1
//#define BRCMLIB_SB_OWLN_MAX2_CHK_W_OFF                               (  74) // 598
//#define BRCMLIB_SB_OWLN_MAX2_CHK_W_WID                               (   0) // 1
//#define BRCMLIB_SB_OWLN_BOOT0_DONE_W_OFF                             (  74) // 599
//#define BRCMLIB_SB_OWLN_BOOT0_DONE_W_WID                             (   0) // 1
//#define BRCMLIB_SB_OWLN_T3C0_Q_ACCEPT_W_OFF                          (  75) // 600
//#define BRCMLIB_SB_OWLN_T3C0_Q_ACCEPT_W_WID                          (   0) // 1
#define   BRCMLIB_SB_OWLN_T3C_STATUS_W_OFF                             (  75) // 600
#define   BRCMLIB_SB_OWLN_T3C_STATUS_W_WID                             (   4) // 32
//#define BRCMLIB_SB_OWLN_T3C0_SBL_RDY_W_OFF                           (  75) // 601
//#define BRCMLIB_SB_OWLN_T3C0_SBL_RDY_W_WID                           (   0) // 1
//#define BRCMLIB_SB_OWLN_T3C0_BOOT_RDY_W_OFF                          (  75) // 602
//#define BRCMLIB_SB_OWLN_T3C0_BOOT_RDY_W_WID                          (   0) // 1
//#define BRCMLIB_SB_OWLN_T3C0_BOOT_CHK_PASS_W_OFF                     (  75) // 603
//#define BRCMLIB_SB_OWLN_T3C0_BOOT_CHK_PASS_W_WID                     (   0) // 1
//#define BRCMLIB_SB_OWLN_T3C1_Q_ACCEPT_W_OFF                          (  75) // 604
//#define BRCMLIB_SB_OWLN_T3C1_Q_ACCEPT_W_WID                          (   0) // 1
//#define BRCMLIB_SB_OWLN_T3C1_SBL_RDY_W_OFF                           (  75) // 605
//#define BRCMLIB_SB_OWLN_T3C1_SBL_RDY_W_WID                           (   0) // 1
//#define BRCMLIB_SB_OWLN_T3C1_BOOT_RDY_W_OFF                          (  75) // 606
//#define BRCMLIB_SB_OWLN_T3C1_BOOT_RDY_W_WID                          (   0) // 1
//#define BRCMLIB_SB_OWLN_T3C1_BOOT_CHK_PASS_W_OFF                     (  75) // 607
//#define BRCMLIB_SB_OWLN_T3C1_BOOT_CHK_PASS_W_WID                     (   0) // 1
//#define BRCMLIB_SB_OWLN_T3C2_Q_ACCEPT_W_OFF                          (  76) // 608
//#define BRCMLIB_SB_OWLN_T3C2_Q_ACCEPT_W_WID                          (   0) // 1
//#define BRCMLIB_SB_OWLN_T3C2_SBL_RDY_W_OFF                           (  76) // 609
//#define BRCMLIB_SB_OWLN_T3C2_SBL_RDY_W_WID                           (   0) // 1
//#define BRCMLIB_SB_OWLN_T3C2_BOOT_RDY_W_OFF                          (  76) // 610
//#define BRCMLIB_SB_OWLN_T3C2_BOOT_RDY_W_WID                          (   0) // 1
//#define BRCMLIB_SB_OWLN_T3C2_BOOT_CHK_PASS_W_OFF                     (  76) // 611
//#define BRCMLIB_SB_OWLN_T3C2_BOOT_CHK_PASS_W_WID                     (   0) // 1
//#define BRCMLIB_SB_OWLN_T3C3_Q_ACCEPT_W_OFF                          (  76) // 612
//#define BRCMLIB_SB_OWLN_T3C3_Q_ACCEPT_W_WID                          (   0) // 1
//#define BRCMLIB_SB_OWLN_T3C3_SBL_RDY_W_OFF                           (  76) // 613
//#define BRCMLIB_SB_OWLN_T3C3_SBL_RDY_W_WID                           (   0) // 1
//#define BRCMLIB_SB_OWLN_T3C3_BOOT_RDY_W_OFF                          (  76) // 614
//#define BRCMLIB_SB_OWLN_T3C3_BOOT_RDY_W_WID                          (   0) // 1
//#define BRCMLIB_SB_OWLN_T3C3_BOOT_CHK_PASS_W_OFF                     (  76) // 615
//#define BRCMLIB_SB_OWLN_T3C3_BOOT_CHK_PASS_W_WID                     (   0) // 1
//#define BRCMLIB_SB_OWLN_T3C4_Q_ACCEPT_W_OFF                          (  77) // 616
//#define BRCMLIB_SB_OWLN_T3C4_Q_ACCEPT_W_WID                          (   0) // 1
//#define BRCMLIB_SB_OWLN_T3C4_SBL_RDY_W_OFF                           (  77) // 617
//#define BRCMLIB_SB_OWLN_T3C4_SBL_RDY_W_WID                           (   0) // 1
//#define BRCMLIB_SB_OWLN_T3C4_BOOT_RDY_W_OFF                          (  77) // 618
//#define BRCMLIB_SB_OWLN_T3C4_BOOT_RDY_W_WID                          (   0) // 1
//#define BRCMLIB_SB_OWLN_T3C4_BOOT_CHK_PASS_W_OFF                     (  77) // 619
//#define BRCMLIB_SB_OWLN_T3C4_BOOT_CHK_PASS_W_WID                     (   0) // 1
//#define BRCMLIB_SB_OWLN_T3C5_Q_ACCEPT_W_OFF                          (  77) // 620
//#define BRCMLIB_SB_OWLN_T3C5_Q_ACCEPT_W_WID                          (   0) // 1
//#define BRCMLIB_SB_OWLN_T3C5_SBL_RDY_W_OFF                           (  77) // 621
//#define BRCMLIB_SB_OWLN_T3C5_SBL_RDY_W_WID                           (   0) // 1
//#define BRCMLIB_SB_OWLN_T3C5_BOOT_RDY_W_OFF                          (  77) // 622
//#define BRCMLIB_SB_OWLN_T3C5_BOOT_RDY_W_WID                          (   0) // 1
//#define BRCMLIB_SB_OWLN_T3C5_BOOT_CHK_PASS_W_OFF                     (  77) // 623
//#define BRCMLIB_SB_OWLN_T3C5_BOOT_CHK_PASS_W_WID                     (   0) // 1
//#define BRCMLIB_SB_OWLN_RESV4_W_OFF                                  (  78) // 624
//#define BRCMLIB_SB_OWLN_RESV4_W_WID                                  (   0) // 7
//#define BRCMLIB_SB_OWLN_T3C_STATUS_VALID_W_OFF                       (  78) // 631
//#define BRCMLIB_SB_OWLN_T3C_STATUS_VALID_W_WID                       (   0) // 1
#define   BRCMLIB_SB_OWLN_RT_ASIC_CORE_W_OFF                           (  79) // 632
#define   BRCMLIB_SB_OWLN_RT_ASIC_CORE_W_WID                           (   4) // 32
#define   BRCMLIB_SB_OWLN_RT_ASIC_CORE_VOLT1_W_OFF                     (  79) // 632
#define   BRCMLIB_SB_OWLN_RT_ASIC_CORE_VOLT1_W_WID                     (   2) // 16
#define   BRCMLIB_SB_OWLN_RT_ASIC_CORE_VOLT2_W_OFF                     (  81) // 648
#define   BRCMLIB_SB_OWLN_RT_ASIC_CORE_VOLT2_W_WID                     (   2) // 16
#define   BRCMLIB_SB_OWLN_RT_RAIL1_W_OFF                               (  83) // 664
#define   BRCMLIB_SB_OWLN_RT_RAIL1_W_WID                               (   4) // 32
#define   BRCMLIB_SB_OWLN_RT_RAIL1_VOLT_W_OFF                          (  83) // 664
#define   BRCMLIB_SB_OWLN_RT_RAIL1_VOLT_W_WID                          (   2) // 16
#define   BRCMLIB_SB_OWLN_RT_RAIL2_VOLT_W_OFF                          (  85) // 680
#define   BRCMLIB_SB_OWLN_RT_RAIL2_VOLT_W_WID                          (   2) // 16
#define   BRCMLIB_SB_OWLN_ERROR_W_OFF                                  (  87) // 696
#define   BRCMLIB_SB_OWLN_ERROR_W_WID                                  (   4) // 32
#define   BRCMLIB_SB_OWLN_PMRO_W_OFF                                   (  91) // 728
#define   BRCMLIB_SB_OWLN_PMRO_W_WID                                   (   4) // 32
#define   BRCMLIB_SB_OWLN_PEC_BYTE_W_OFF                               (  95) // 760
#define   BRCMLIB_SB_OWLN_PEC_BYTE_W_WID                               (   1) // 8
#define   BRCMLIB_SB_OWLS_LENGTH_W_OFF                                 (  96) // 768
#define   BRCMLIB_SB_OWLS_LENGTH_W_WID                                 (   1) // 8
#define   BRCMLIB_SB_OWLS_RESV1_W_OFF                                  (  97) // 776
#define   BRCMLIB_SB_OWLS_RESV1_W_WID                                  (   1) // 8
#define   BRCMLIB_SB_OWLS_M7_MAJOR_W_OFF                               (  98) // 784
#define   BRCMLIB_SB_OWLS_M7_MAJOR_W_WID                               (   1) // 8
#define   BRCMLIB_SB_OWLS_M7_MINOR_W_OFF                               (  99) // 792
#define   BRCMLIB_SB_OWLS_M7_MINOR_W_WID                               (   1) // 8
#define   BRCMLIB_SB_OWLS_OWL_CONFIG_W_OFF                             ( 100) // 800
#define   BRCMLIB_SB_OWLS_OWL_CONFIG_W_WID                             (   4) // 32
//#define BRCMLIB_SB_OWLS_OWL_IDX_W_OFF                                ( 100) // 800
//#define BRCMLIB_SB_OWLS_OWL_IDX_W_WID                                (   0) // 4
//#define BRCMLIB_SB_OWLS_NUM_T3CS_W_OFF                               ( 100) // 804
//#define BRCMLIB_SB_OWLS_NUM_T3CS_W_WID                               (   0) // 4
//#define BRCMLIB_SB_OWLS_T3C0_PFS_W_OFF                               ( 101) // 808
//#define BRCMLIB_SB_OWLS_T3C0_PFS_W_WID                               (   0) // 4
//#define BRCMLIB_SB_OWLS_T3C1_PFS_W_OFF                               ( 101) // 812
//#define BRCMLIB_SB_OWLS_T3C1_PFS_W_WID                               (   0) // 4
//#define BRCMLIB_SB_OWLS_T3C2_PFS_W_OFF                               ( 102) // 816
//#define BRCMLIB_SB_OWLS_T3C2_PFS_W_WID                               (   0) // 4
//#define BRCMLIB_SB_OWLS_T3C3_PFS_W_OFF                               ( 102) // 820
//#define BRCMLIB_SB_OWLS_T3C3_PFS_W_WID                               (   0) // 4
//#define BRCMLIB_SB_OWLS_T3C4_PFS_W_OFF                               ( 103) // 824
//#define BRCMLIB_SB_OWLS_T3C4_PFS_W_WID                               (   0) // 4
//#define BRCMLIB_SB_OWLS_T3C5_PFS_W_OFF                               ( 103) // 828
//#define BRCMLIB_SB_OWLS_T3C5_PFS_W_WID                               (   0) // 4
#define   BRCMLIB_SB_OWLS_BOOT_SRC_W_OFF                               ( 104) // 832
#define   BRCMLIB_SB_OWLS_BOOT_SRC_W_WID                               (   1) // 8
#define   BRCMLIB_SB_OWLS_BOOT_STATUS1_W_OFF                           ( 104) // 832
#define   BRCMLIB_SB_OWLS_BOOT_STATUS1_W_WID                           (   4) // 32
//#define BRCMLIB_SB_OWLS_MEM_REPAIR_REI_FAIL_W_OFF                    ( 105) // 840
//#define BRCMLIB_SB_OWLS_MEM_REPAIR_REI_FAIL_W_WID                    (   0) // 1
//#define BRCMLIB_SB_OWLS_MEM_REPAIR_REI_PASS_W_OFF                    ( 105) // 841
//#define BRCMLIB_SB_OWLS_MEM_REPAIR_REI_PASS_W_WID                    (   0) // 1
//#define BRCMLIB_SB_OWLS_MEM_REPAIR_MBISR_DONE_W_OFF                  ( 105) // 842
//#define BRCMLIB_SB_OWLS_MEM_REPAIR_MBISR_DONE_W_WID                  (   0) // 1
//#define BRCMLIB_SB_OWLS_MEM_REPAIR_MBISR_GO_W_OFF                    ( 105) // 843
//#define BRCMLIB_SB_OWLS_MEM_REPAIR_MBISR_GO_W_WID                    (   0) // 1
//#define BRCMLIB_SB_OWLS_MEM_REPAIR_START_W_OFF                       ( 105) // 844
//#define BRCMLIB_SB_OWLS_MEM_REPAIR_START_W_WID                       (   0) // 1
//#define BRCMLIB_SB_OWLS_MEM_REPAIR_TIMEOUT_W_OFF                     ( 105) // 845
//#define BRCMLIB_SB_OWLS_MEM_REPAIR_TIMEOUT_W_WID                     (   0) // 1
//#define BRCMLIB_SB_OWLS_FSM_BISR_DONE_W_OFF                          ( 105) // 846
//#define BRCMLIB_SB_OWLS_FSM_BISR_DONE_W_WID                          (   0) // 1
//#define BRCMLIB_SB_OWLS_FSM_BISR_GO_W_OFF                            ( 105) // 847
//#define BRCMLIB_SB_OWLS_FSM_BISR_GO_W_WID                            (   0) // 1
//#define BRCMLIB_SB_OWLS_FSM_MEM_MAST_PASS_W_OFF                      ( 106) // 848
//#define BRCMLIB_SB_OWLS_FSM_MEM_MAST_PASS_W_WID                      (   0) // 1
//#define BRCMLIB_SB_OWLS_FSM_MEM_MAST_FAIL_W_OFF                      ( 106) // 849
//#define BRCMLIB_SB_OWLS_FSM_MEM_MAST_FAIL_W_WID                      (   0) // 1
//#define BRCMLIB_SB_OWLS_UART_INITIALIZE_W_OFF                        ( 106) // 850
//#define BRCMLIB_SB_OWLS_UART_INITIALIZE_W_WID                        (   0) // 1
//#define BRCMLIB_SB_OWLS_UART_RX_LOCKED_W_OFF                         ( 106) // 851
//#define BRCMLIB_SB_OWLS_UART_RX_LOCKED_W_WID                         (   0) // 1
//#define BRCMLIB_SB_OWLS_LCS_W_OFF                                    ( 106) // 852
//#define BRCMLIB_SB_OWLS_LCS_W_WID                                    (   0) // 4
//#define BRCMLIB_SB_OWLS_SYS_PLL_LOCKED_W_OFF                         ( 107) // 856
//#define BRCMLIB_SB_OWLS_SYS_PLL_LOCKED_W_WID                         (   0) // 1
//#define BRCMLIB_SB_OWLS_I2C_MASTER_W_OFF                             ( 107) // 857
//#define BRCMLIB_SB_OWLS_I2C_MASTER_W_WID                             (   0) // 1
//#define BRCMLIB_SB_OWLS_I2C_SLAVE_W_OFF                              ( 107) // 858
//#define BRCMLIB_SB_OWLS_I2C_SLAVE_W_WID                              (   0) // 1
//#define BRCMLIB_SB_OWLS_RESV3_W_OFF                                  ( 107) // 859
//#define BRCMLIB_SB_OWLS_RESV3_W_WID                                  (   0) // 1
//#define BRCMLIB_SB_OWLS_MAX2_PLL_W_OFF                               ( 107) // 860
//#define BRCMLIB_SB_OWLS_MAX2_PLL_W_WID                               (   0) // 1
//#define BRCMLIB_SB_OWLS_MAX2_PHY_W_OFF                               ( 107) // 861
//#define BRCMLIB_SB_OWLS_MAX2_PHY_W_WID                               (   0) // 1
//#define BRCMLIB_SB_OWLS_MAX2_CHK_W_OFF                               ( 107) // 862
//#define BRCMLIB_SB_OWLS_MAX2_CHK_W_WID                               (   0) // 1
//#define BRCMLIB_SB_OWLS_BOOT0_DONE_W_OFF                             ( 107) // 863
//#define BRCMLIB_SB_OWLS_BOOT0_DONE_W_WID                             (   0) // 1
//#define BRCMLIB_SB_OWLS_T3C0_Q_ACCEPT_W_OFF                          ( 108) // 864
//#define BRCMLIB_SB_OWLS_T3C0_Q_ACCEPT_W_WID                          (   0) // 1
#define   BRCMLIB_SB_OWLS_T3C_STATUS_W_OFF                             ( 108) // 864
#define   BRCMLIB_SB_OWLS_T3C_STATUS_W_WID                             (   4) // 32
//#define BRCMLIB_SB_OWLS_T3C0_SBL_RDY_W_OFF                           ( 108) // 865
//#define BRCMLIB_SB_OWLS_T3C0_SBL_RDY_W_WID                           (   0) // 1
//#define BRCMLIB_SB_OWLS_T3C0_BOOT_RDY_W_OFF                          ( 108) // 866
//#define BRCMLIB_SB_OWLS_T3C0_BOOT_RDY_W_WID                          (   0) // 1
//#define BRCMLIB_SB_OWLS_T3C0_BOOT_CHK_PASS_W_OFF                     ( 108) // 867
//#define BRCMLIB_SB_OWLS_T3C0_BOOT_CHK_PASS_W_WID                     (   0) // 1
//#define BRCMLIB_SB_OWLS_T3C1_Q_ACCEPT_W_OFF                          ( 108) // 868
//#define BRCMLIB_SB_OWLS_T3C1_Q_ACCEPT_W_WID                          (   0) // 1
//#define BRCMLIB_SB_OWLS_T3C1_SBL_RDY_W_OFF                           ( 108) // 869
//#define BRCMLIB_SB_OWLS_T3C1_SBL_RDY_W_WID                           (   0) // 1
//#define BRCMLIB_SB_OWLS_T3C1_BOOT_RDY_W_OFF                          ( 108) // 870
//#define BRCMLIB_SB_OWLS_T3C1_BOOT_RDY_W_WID                          (   0) // 1
//#define BRCMLIB_SB_OWLS_T3C1_BOOT_CHK_PASS_W_OFF                     ( 108) // 871
//#define BRCMLIB_SB_OWLS_T3C1_BOOT_CHK_PASS_W_WID                     (   0) // 1
//#define BRCMLIB_SB_OWLS_T3C2_Q_ACCEPT_W_OFF                          ( 109) // 872
//#define BRCMLIB_SB_OWLS_T3C2_Q_ACCEPT_W_WID                          (   0) // 1
//#define BRCMLIB_SB_OWLS_T3C2_SBL_RDY_W_OFF                           ( 109) // 873
//#define BRCMLIB_SB_OWLS_T3C2_SBL_RDY_W_WID                           (   0) // 1
//#define BRCMLIB_SB_OWLS_T3C2_BOOT_RDY_W_OFF                          ( 109) // 874
//#define BRCMLIB_SB_OWLS_T3C2_BOOT_RDY_W_WID                          (   0) // 1
//#define BRCMLIB_SB_OWLS_T3C2_BOOT_CHK_PASS_W_OFF                     ( 109) // 875
//#define BRCMLIB_SB_OWLS_T3C2_BOOT_CHK_PASS_W_WID                     (   0) // 1
//#define BRCMLIB_SB_OWLS_T3C3_Q_ACCEPT_W_OFF                          ( 109) // 876
//#define BRCMLIB_SB_OWLS_T3C3_Q_ACCEPT_W_WID                          (   0) // 1
//#define BRCMLIB_SB_OWLS_T3C3_SBL_RDY_W_OFF                           ( 109) // 877
//#define BRCMLIB_SB_OWLS_T3C3_SBL_RDY_W_WID                           (   0) // 1
//#define BRCMLIB_SB_OWLS_T3C3_BOOT_RDY_W_OFF                          ( 109) // 878
//#define BRCMLIB_SB_OWLS_T3C3_BOOT_RDY_W_WID                          (   0) // 1
//#define BRCMLIB_SB_OWLS_T3C3_BOOT_CHK_PASS_W_OFF                     ( 109) // 879
//#define BRCMLIB_SB_OWLS_T3C3_BOOT_CHK_PASS_W_WID                     (   0) // 1
//#define BRCMLIB_SB_OWLS_T3C4_Q_ACCEPT_W_OFF                          ( 110) // 880
//#define BRCMLIB_SB_OWLS_T3C4_Q_ACCEPT_W_WID                          (   0) // 1
//#define BRCMLIB_SB_OWLS_T3C4_SBL_RDY_W_OFF                           ( 110) // 881
//#define BRCMLIB_SB_OWLS_T3C4_SBL_RDY_W_WID                           (   0) // 1
//#define BRCMLIB_SB_OWLS_T3C4_BOOT_RDY_W_OFF                          ( 110) // 882
//#define BRCMLIB_SB_OWLS_T3C4_BOOT_RDY_W_WID                          (   0) // 1
//#define BRCMLIB_SB_OWLS_T3C4_BOOT_CHK_PASS_W_OFF                     ( 110) // 883
//#define BRCMLIB_SB_OWLS_T3C4_BOOT_CHK_PASS_W_WID                     (   0) // 1
//#define BRCMLIB_SB_OWLS_T3C5_Q_ACCEPT_W_OFF                          ( 110) // 884
//#define BRCMLIB_SB_OWLS_T3C5_Q_ACCEPT_W_WID                          (   0) // 1
//#define BRCMLIB_SB_OWLS_T3C5_SBL_RDY_W_OFF                           ( 110) // 885
//#define BRCMLIB_SB_OWLS_T3C5_SBL_RDY_W_WID                           (   0) // 1
//#define BRCMLIB_SB_OWLS_T3C5_BOOT_RDY_W_OFF                          ( 110) // 886
//#define BRCMLIB_SB_OWLS_T3C5_BOOT_RDY_W_WID                          (   0) // 1
//#define BRCMLIB_SB_OWLS_T3C5_BOOT_CHK_PASS_W_OFF                     ( 110) // 887
//#define BRCMLIB_SB_OWLS_T3C5_BOOT_CHK_PASS_W_WID                     (   0) // 1
//#define BRCMLIB_SB_OWLS_RESV4_W_OFF                                  ( 111) // 888
//#define BRCMLIB_SB_OWLS_RESV4_W_WID                                  (   0) // 7
//#define BRCMLIB_SB_OWLS_T3C_STATUS_VALID_W_OFF                       ( 111) // 895
//#define BRCMLIB_SB_OWLS_T3C_STATUS_VALID_W_WID                       (   0) // 1
#define   BRCMLIB_SB_OWLS_RT_ASIC_CORE_W_OFF                           ( 112) // 896
#define   BRCMLIB_SB_OWLS_RT_ASIC_CORE_W_WID                           (   4) // 32
#define   BRCMLIB_SB_OWLS_RT_ASIC_CORE_VOLT1_W_OFF                     ( 112) // 896
#define   BRCMLIB_SB_OWLS_RT_ASIC_CORE_VOLT1_W_WID                     (   2) // 16
#define   BRCMLIB_SB_OWLS_RT_ASIC_CORE_VOLT2_W_OFF                     ( 114) // 912
#define   BRCMLIB_SB_OWLS_RT_ASIC_CORE_VOLT2_W_WID                     (   2) // 16
#define   BRCMLIB_SB_OWLS_RT_RAIL1_W_OFF                               ( 116) // 928
#define   BRCMLIB_SB_OWLS_RT_RAIL1_W_WID                               (   4) // 32
#define   BRCMLIB_SB_OWLS_RT_RAIL1_VOLT_W_OFF                          ( 116) // 928
#define   BRCMLIB_SB_OWLS_RT_RAIL1_VOLT_W_WID                          (   2) // 16
#define   BRCMLIB_SB_OWLS_RT_RAIL2_VOLT_W_OFF                          ( 118) // 944
#define   BRCMLIB_SB_OWLS_RT_RAIL2_VOLT_W_WID                          (   2) // 16
#define   BRCMLIB_SB_OWLS_ERROR_W_OFF                                  ( 120) // 960
#define   BRCMLIB_SB_OWLS_ERROR_W_WID                                  (   4) // 32
#define   BRCMLIB_SB_OWLS_PMRO_W_OFF                                   ( 124) // 992
#define   BRCMLIB_SB_OWLS_PMRO_W_WID                                   (   4) // 32
#define   BRCMLIB_SB_OWLS_PEC_BYTE_W_OFF                               ( 128) // 1024
#define   BRCMLIB_SB_OWLS_PEC_BYTE_W_WID                               (   1) // 8
#define   BRCMLIB_SB_FW_UPDATE_LENGTH_W_OFF                            ( 132) // 1056
#define   BRCMLIB_SB_FW_UPDATE_LENGTH_W_WID                            (   1) // 8
#define   BRCMLIB_SB_FW_UPDATE_CONTROL_W_OFF                           ( 133) // 1064
#define   BRCMLIB_SB_FW_UPDATE_CONTROL_W_WID                           (   1) // 8
//#define BRCMLIB_SB_FW_UPDATE_RESERVED0_W_OFF                         ( 133) // 1064
//#define BRCMLIB_SB_FW_UPDATE_RESERVED0_W_WID                         (   0) // 1
//#define BRCMLIB_SB_FW_UPDATE_IN_PROGRESS_W_OFF                       ( 133) // 1065
//#define BRCMLIB_SB_FW_UPDATE_IN_PROGRESS_W_WID                       (   0) // 1
//#define BRCMLIB_SB_FW_UPDATE_UPDATE_COMPLETE_W_OFF                   ( 133) // 1066
//#define BRCMLIB_SB_FW_UPDATE_UPDATE_COMPLETE_W_WID                   (   0) // 1
//#define BRCMLIB_SB_FW_UPDATE_TRANSFER_DONE_W_OFF                     ( 133) // 1067
//#define BRCMLIB_SB_FW_UPDATE_TRANSFER_DONE_W_WID                     (   0) // 1
//#define BRCMLIB_SB_FW_UPDATE_RESERVED4_W_OFF                         ( 133) // 1068
//#define BRCMLIB_SB_FW_UPDATE_RESERVED4_W_WID                         (   0) // 1
//#define BRCMLIB_SB_FW_UPDATE_SMBUS_ACCESS_W_OFF                      ( 133) // 1069
//#define BRCMLIB_SB_FW_UPDATE_SMBUS_ACCESS_W_WID                      (   0) // 1
//#define BRCMLIB_SB_FW_UPDATE_TRANSFER_READY_W_OFF                    ( 133) // 1070
//#define BRCMLIB_SB_FW_UPDATE_TRANSFER_READY_W_WID                    (   0) // 1
//#define BRCMLIB_SB_FW_UPDATE_INITIATE_DOWNLOAD_W_OFF                 ( 133) // 1071
//#define BRCMLIB_SB_FW_UPDATE_INITIATE_DOWNLOAD_W_WID                 (   0) // 1
#define   BRCMLIB_SB_FW_UPDATE_PEC_BYTE_W_OFF                          ( 134) // 1072
#define   BRCMLIB_SB_FW_UPDATE_PEC_BYTE_W_WID                          (   1) // 8
#define   BRCMLIB_SB_FW_UPDATE_EXT_LENGTH_W_OFF                        ( 135) // 1080
#define   BRCMLIB_SB_FW_UPDATE_EXT_LENGTH_W_WID                        (   1) // 8
#define   BRCMLIB_SB_FW_UPDATE_EXT_PAYLOAD_SIZE_W_OFF                  ( 136) // 1088
#define   BRCMLIB_SB_FW_UPDATE_EXT_PAYLOAD_SIZE_W_WID                  (   4) // 32
#define   BRCMLIB_SB_FW_UPDATE_EXT_PAYLOAD_ADDR_W_OFF                  ( 140) // 1120
#define   BRCMLIB_SB_FW_UPDATE_EXT_PAYLOAD_ADDR_W_WID                  (   4) // 32
#define   BRCMLIB_SB_FW_UPDATE_EXT_RESERVED_1152_W_OFF                 ( 144) // 1152
#define   BRCMLIB_SB_FW_UPDATE_EXT_RESERVED_1152_W_WID                 (   1) // 8
#define   BRCMLIB_SB_FW_UPDATE_EXT_RESERVED_1160_W_OFF                 ( 145) // 1160
#define   BRCMLIB_SB_FW_UPDATE_EXT_RESERVED_1160_W_WID                 (   1) // 8
//#define BRCMLIB_SB_FW_UPDATE_EXT_BOOT_W_OFF                          ( 146) // 1168
//#define BRCMLIB_SB_FW_UPDATE_EXT_BOOT_W_WID                          (   0) // 1
//#define BRCMLIB_SB_FW_UPDATE_EXT_VERIFY_W_OFF                        ( 146) // 1169
//#define BRCMLIB_SB_FW_UPDATE_EXT_VERIFY_W_WID                        (   0) // 1
//#define BRCMLIB_SB_FW_UPDATE_EXT_RESERVED_W_OFF                      ( 146) // 1170
//#define BRCMLIB_SB_FW_UPDATE_EXT_RESERVED_W_WID                      (   0) // 6
//#define BRCMLIB_SB_FW_UPDATE_EXT_RESERVED_1176_W_OFF                 ( 147) // 1176
//#define BRCMLIB_SB_FW_UPDATE_EXT_RESERVED_1176_W_WID                 (   0) // 4
//#define BRCMLIB_SB_FW_UPDATE_EXT_RESERVED_1180_W_OFF                 ( 147) // 1180
//#define BRCMLIB_SB_FW_UPDATE_EXT_RESERVED_1180_W_WID                 (   0) // 4
#define   BRCMLIB_SB_FW_UPDATE_EXT_ERROR_W_OFF                         ( 148) // 1184
#define   BRCMLIB_SB_FW_UPDATE_EXT_ERROR_W_WID                         (   4) // 32
//#define BRCMLIB_SB_FW_UPDATE_EXT_RESERVED_1216_W_OFF                 ( 152) // 1216
//#define BRCMLIB_SB_FW_UPDATE_EXT_RESERVED_1216_W_WID                 (   0) // 4
//#define BRCMLIB_SB_FW_UPDATE_EXT_RESERVED_1220_W_OFF                 ( 152) // 1220
//#define BRCMLIB_SB_FW_UPDATE_EXT_RESERVED_1220_W_WID                 (   0) // 4
#define   BRCMLIB_SB_FW_UPDATE_EXT_EXEC_FW_RESET_W_OFF                 ( 153) // 1224
#define   BRCMLIB_SB_FW_UPDATE_EXT_EXEC_FW_RESET_W_WID                 (   1) // 8
#define   BRCMLIB_SB_FW_UPDATE_EXT_RESERVED1_W_OFF                     ( 154) // 1232
#define   BRCMLIB_SB_FW_UPDATE_EXT_RESERVED1_W_WID                     (   1) // 8
#define   BRCMLIB_SB_FW_UPDATE_EXT_RESERVED2_W_OFF                     ( 155) // 1240
#define   BRCMLIB_SB_FW_UPDATE_EXT_RESERVED2_W_WID                     (   1) // 8
#define   BRCMLIB_SB_FW_UPDATE_EXT_CNT_W_OFF                           ( 156) // 1248
#define   BRCMLIB_SB_FW_UPDATE_EXT_CNT_W_WID                           (   3) // 24
#define   BRCMLIB_SB_FW_UPDATE_EXT_STATE_W_OFF                         ( 159) // 1272
#define   BRCMLIB_SB_FW_UPDATE_EXT_STATE_W_WID                         (   1) // 8
#define   BRCMLIB_SB_FW_UPDATE_EXT_PEC_BYTE_W_OFF                      ( 160) // 1280
#define   BRCMLIB_SB_FW_UPDATE_EXT_PEC_BYTE_W_WID                      (   1) // 8
#define   BRCMLIB_SB_ADVERTISE_LENGTH_W_OFF                            ( 161) // 1288
#define   BRCMLIB_SB_ADVERTISE_LENGTH_W_WID                            (   1) // 8
#define   BRCMLIB_SB_ADVERTISE_EXEC_LOC_W_OFF                          ( 162) // 1296
#define   BRCMLIB_SB_ADVERTISE_EXEC_LOC_W_WID                          (   1) // 8
#define   BRCMLIB_SB_ADVERTISE_RESERVED_1304_W_OFF                     ( 163) // 1304
#define   BRCMLIB_SB_ADVERTISE_RESERVED_1304_W_WID                     (   1) // 8
#define   BRCMLIB_SB_ADVERTISE_TRANSFER_MEM_START_W_OFF                ( 164) // 1312
#define   BRCMLIB_SB_ADVERTISE_TRANSFER_MEM_START_W_WID                (   4) // 32
#define   BRCMLIB_SB_ADVERTISE_TRANSFER_MEM_SIZE_W_OFF                 ( 168) // 1344
#define   BRCMLIB_SB_ADVERTISE_TRANSFER_MEM_SIZE_W_WID                 (   4) // 32
#define   BRCMLIB_SB_ADVERTISE_CIP_CFG1_W_OFF                          ( 172) // 1376
#define   BRCMLIB_SB_ADVERTISE_CIP_CFG1_W_WID                          (   4) // 32
#define   BRCMLIB_SB_ADVERTISE_CIP_CFG2_W_OFF                          ( 176) // 1408
#define   BRCMLIB_SB_ADVERTISE_CIP_CFG2_W_WID                          (   4) // 32
#define   BRCMLIB_SB_ADVERTISE_SOC_VER_W_OFF                           ( 180) // 1440
#define   BRCMLIB_SB_ADVERTISE_SOC_VER_W_WID                           (   4) // 32
#define   BRCMLIB_SB_ADVERTISE_BOOT_STATUS1_W_OFF                      ( 184) // 1472
#define   BRCMLIB_SB_ADVERTISE_BOOT_STATUS1_W_WID                      (   4) // 32
//#define BRCMLIB_SB_ADVERTISE_RESERVED_1472_W_OFF                     ( 184) // 1472
//#define BRCMLIB_SB_ADVERTISE_RESERVED_1472_W_WID                     (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_BRD_DEV_IDX_W_OFF                       ( 184) // 1473
//#define BRCMLIB_SB_ADVERTISE_BRD_DEV_IDX_W_WID                       (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_SYS_PLL_LOCKED_W_OFF                    ( 184) // 1474
//#define BRCMLIB_SB_ADVERTISE_SYS_PLL_LOCKED_W_WID                    (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_PCIE_PLL_LOCKED_W_OFF                   ( 184) // 1475
//#define BRCMLIB_SB_ADVERTISE_PCIE_PLL_LOCKED_W_WID                   (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_RESET_TYPE_W_OFF                        ( 184) // 1476
//#define BRCMLIB_SB_ADVERTISE_RESET_TYPE_W_WID                        (   0) // 4
//#define BRCMLIB_SB_ADVERTISE_FLASH_BLANK_CONTAINER_W_OFF             ( 185) // 1480
//#define BRCMLIB_SB_ADVERTISE_FLASH_BLANK_CONTAINER_W_WID             (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_FLASH_SBOOT_CONTAINER_W_OFF             ( 185) // 1481
//#define BRCMLIB_SB_ADVERTISE_FLASH_SBOOT_CONTAINER_W_WID             (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_FLASH_OS_CONTAINER_W_OFF                ( 185) // 1482
//#define BRCMLIB_SB_ADVERTISE_FLASH_OS_CONTAINER_W_WID                (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_FLASH_USER_CONTAINER_W_OFF              ( 185) // 1483
//#define BRCMLIB_SB_ADVERTISE_FLASH_USER_CONTAINER_W_WID              (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_FLASH_PATCH_W_OFF                       ( 185) // 1484
//#define BRCMLIB_SB_ADVERTISE_FLASH_PATCH_W_WID                       (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_SPIPE_ROM_W_OFF                         ( 185) // 1485
//#define BRCMLIB_SB_ADVERTISE_SPIPE_ROM_W_WID                         (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_SPIPE_PROT_W_OFF                        ( 185) // 1486
//#define BRCMLIB_SB_ADVERTISE_SPIPE_PROT_W_WID                        (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_SPIPE_SET_VALID_W_OFF                   ( 185) // 1487
//#define BRCMLIB_SB_ADVERTISE_SPIPE_SET_VALID_W_WID                   (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_SERDES_CONFIG_LOAD_W_OFF                ( 186) // 1488
//#define BRCMLIB_SB_ADVERTISE_SERDES_CONFIG_LOAD_W_WID                (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_SERDES_PHY_READY_W_OFF                  ( 186) // 1489
//#define BRCMLIB_SB_ADVERTISE_SERDES_PHY_READY_W_WID                  (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_MEM_REPAIR_GP1_TMOUT_W_OFF              ( 186) // 1490
//#define BRCMLIB_SB_ADVERTISE_MEM_REPAIR_GP1_TMOUT_W_WID              (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_MEM_REPAIR_GP2_TMOUT_W_OFF              ( 186) // 1491
//#define BRCMLIB_SB_ADVERTISE_MEM_REPAIR_GP2_TMOUT_W_WID              (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_MEM_REPAIR_GP1_STATUS_W_OFF             ( 186) // 1492
//#define BRCMLIB_SB_ADVERTISE_MEM_REPAIR_GP1_STATUS_W_WID             (   0) // 2
//#define BRCMLIB_SB_ADVERTISE_MEM_REPAIR_GP2_STATUS_W_OFF             ( 186) // 1494
//#define BRCMLIB_SB_ADVERTISE_MEM_REPAIR_GP2_STATUS_W_WID             (   0) // 2
//#define BRCMLIB_SB_ADVERTISE_CIP_OV_CLK_EN_W_OFF                     ( 187) // 1496
//#define BRCMLIB_SB_ADVERTISE_CIP_OV_CLK_EN_W_WID                     (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_CIP_OV_LCK_W_OFF                        ( 187) // 1497
//#define BRCMLIB_SB_ADVERTISE_CIP_OV_LCK_W_WID                        (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_CIP_LCK_W_OFF                           ( 187) // 1498
//#define BRCMLIB_SB_ADVERTISE_CIP_LCK_W_WID                           (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_UART_RX_LOCKED_W_OFF                    ( 187) // 1499
//#define BRCMLIB_SB_ADVERTISE_UART_RX_LOCKED_W_WID                    (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_CIP_OV_CLK_IDX_W_OFF                    ( 187) // 1500
//#define BRCMLIB_SB_ADVERTISE_CIP_OV_CLK_IDX_W_WID                    (   0) // 4
#define   BRCMLIB_SB_ADVERTISE_BOOT_STATUS2_W_OFF                      ( 188) // 1504
#define   BRCMLIB_SB_ADVERTISE_BOOT_STATUS2_W_WID                      (   4) // 32
//#define BRCMLIB_SB_ADVERTISE_SERDES_INITIALIZE_W_OFF                 ( 188) // 1504
//#define BRCMLIB_SB_ADVERTISE_SERDES_INITIALIZE_W_WID                 (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_PCIE_LINK_UP_W_OFF                      ( 188) // 1505
//#define BRCMLIB_SB_ADVERTISE_PCIE_LINK_UP_W_WID                      (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_PCIE_INITIALIZE_W_OFF                   ( 188) // 1506
//#define BRCMLIB_SB_ADVERTISE_PCIE_INITIALIZE_W_WID                   (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_UART_INITIALIZE_W_OFF                   ( 188) // 1507
//#define BRCMLIB_SB_ADVERTISE_UART_INITIALIZE_W_WID                   (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_SMBUS_INITIALIZE_W_OFF                  ( 188) // 1508
//#define BRCMLIB_SB_ADVERTISE_SMBUS_INITIALIZE_W_WID                  (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_QSPI_INITIALIZE_W_OFF                   ( 188) // 1509
//#define BRCMLIB_SB_ADVERTISE_QSPI_INITIALIZE_W_WID                   (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_GPIO_INITIALIZE_W_OFF                   ( 188) // 1510
//#define BRCMLIB_SB_ADVERTISE_GPIO_INITIALIZE_W_WID                   (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_RT630_INIT_W_OFF                        ( 188) // 1511
//#define BRCMLIB_SB_ADVERTISE_RT630_INIT_W_WID                        (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_REACHED_MAIN_LOOP_W_OFF                 ( 189) // 1512
//#define BRCMLIB_SB_ADVERTISE_REACHED_MAIN_LOOP_W_WID                 (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_REACHED_UART_POLLING_W_OFF              ( 189) // 1513
//#define BRCMLIB_SB_ADVERTISE_REACHED_UART_POLLING_W_WID              (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_REACHED_SMBUS_POLLING_W_OFF             ( 189) // 1514
//#define BRCMLIB_SB_ADVERTISE_REACHED_SMBUS_POLLING_W_WID             (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_REACHED_UPGRADE_POLLING_W_OFF           ( 189) // 1515
//#define BRCMLIB_SB_ADVERTISE_REACHED_UPGRADE_POLLING_W_WID           (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_REACHED_SIGNS_OF_LIFE_W_OFF             ( 189) // 1516
//#define BRCMLIB_SB_ADVERTISE_REACHED_SIGNS_OF_LIFE_W_WID             (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_FLASH_SAFEMODE_W_OFF                    ( 189) // 1517
//#define BRCMLIB_SB_ADVERTISE_FLASH_SAFEMODE_W_WID                    (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_QSPI_TIMEOUT_W_OFF                      ( 189) // 1518
//#define BRCMLIB_SB_ADVERTISE_QSPI_TIMEOUT_W_WID                      (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_SBUS_TIMEOUT_W_OFF                      ( 189) // 1519
//#define BRCMLIB_SB_ADVERTISE_SBUS_TIMEOUT_W_WID                      (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_TZC_SRAM_TIMEOUT_W_OFF                  ( 190) // 1520
//#define BRCMLIB_SB_ADVERTISE_TZC_SRAM_TIMEOUT_W_WID                  (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_RESERVED_1521_W_OFF                     ( 190) // 1521
//#define BRCMLIB_SB_ADVERTISE_RESERVED_1521_W_WID                     (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_RESERVED_1522_W_OFF                     ( 190) // 1522
//#define BRCMLIB_SB_ADVERTISE_RESERVED_1522_W_WID                     (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_TZC_PCIE_TIMEOUT_W_OFF                  ( 190) // 1523
//#define BRCMLIB_SB_ADVERTISE_TZC_PCIE_TIMEOUT_W_WID                  (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_TZC_SRAM_ERROR_W_OFF                    ( 190) // 1524
//#define BRCMLIB_SB_ADVERTISE_TZC_SRAM_ERROR_W_WID                    (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_FLASH_FBOOT_CONTAINER_W_OFF             ( 190) // 1525
//#define BRCMLIB_SB_ADVERTISE_FLASH_FBOOT_CONTAINER_W_WID             (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_RESERVED_1526_W_OFF                     ( 190) // 1526
//#define BRCMLIB_SB_ADVERTISE_RESERVED_1526_W_WID                     (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_TZC_PCIE_ERROR_W_OFF                    ( 190) // 1527
//#define BRCMLIB_SB_ADVERTISE_TZC_PCIE_ERROR_W_WID                    (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_LCS_STATE_W_OFF                         ( 191) // 1528
//#define BRCMLIB_SB_ADVERTISE_LCS_STATE_W_WID                         (   0) // 3
//#define BRCMLIB_SB_ADVERTISE_RESERVED_1531_W_OFF                     ( 191) // 1531
//#define BRCMLIB_SB_ADVERTISE_RESERVED_1531_W_WID                     (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_RESERVED_1532_W_OFF                     ( 191) // 1532
//#define BRCMLIB_SB_ADVERTISE_RESERVED_1532_W_WID                     (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_RESERVED_1533_W_OFF                     ( 191) // 1533
//#define BRCMLIB_SB_ADVERTISE_RESERVED_1533_W_WID                     (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_HBM_INITIALIZE_W_OFF                    ( 191) // 1534
//#define BRCMLIB_SB_ADVERTISE_HBM_INITIALIZE_W_WID                    (   0) // 1
//#define BRCMLIB_SB_ADVERTISE_BOOT1_COMPLETE_W_OFF                    ( 191) // 1535
//#define BRCMLIB_SB_ADVERTISE_BOOT1_COMPLETE_W_WID                    (   0) // 1
#define   BRCMLIB_SB_ADVERTISE_PEC_BYTE_W_OFF                          ( 192) // 1536
#define   BRCMLIB_SB_ADVERTISE_PEC_BYTE_W_WID                          (   1) // 8
#define   BRCMLIB_SB_PAYLOAD_EXEC_LENGTH_W_OFF                         ( 193) // 1544
#define   BRCMLIB_SB_PAYLOAD_EXEC_LENGTH_W_WID                         (   1) // 8
#define   BRCMLIB_SB_PAYLOAD_EXEC_EXEC_ID_W_OFF                        ( 194) // 1552
#define   BRCMLIB_SB_PAYLOAD_EXEC_EXEC_ID_W_WID                        (   1) // 8
#define   BRCMLIB_SB_PAYLOAD_EXEC_EXEC_STATUS_W_OFF                    ( 195) // 1560
#define   BRCMLIB_SB_PAYLOAD_EXEC_EXEC_STATUS_W_WID                    (   1) // 8
#define   BRCMLIB_SB_PAYLOAD_EXEC_EXEC_STATE_W_OFF                     ( 196) // 1568
#define   BRCMLIB_SB_PAYLOAD_EXEC_EXEC_STATE_W_WID                     (   1) // 8
#define   BRCMLIB_SB_PAYLOAD_EXEC_EXEC_RESULT_W_OFF                    ( 197) // 1576
#define   BRCMLIB_SB_PAYLOAD_EXEC_EXEC_RESULT_W_WID                    (   1) // 8
#define   BRCMLIB_SB_PAYLOAD_EXEC_FORCED_STATE_FAIL_W_OFF              ( 198) // 1584
#define   BRCMLIB_SB_PAYLOAD_EXEC_FORCED_STATE_FAIL_W_WID              (   1) // 8
#define   BRCMLIB_SB_PAYLOAD_EXEC_ABORT_W_OFF                          ( 199) // 1592
#define   BRCMLIB_SB_PAYLOAD_EXEC_ABORT_W_WID                          (   1) // 8
#define   BRCMLIB_SB_PAYLOAD_EXEC_PEC_BYTE_W_OFF                       ( 200) // 1600
#define   BRCMLIB_SB_PAYLOAD_EXEC_PEC_BYTE_W_WID                       (   1) // 8
#define   BRCMLIB_SB_DDR_CONFIG_LENGTH_W_OFF                           ( 201) // 1608
#define   BRCMLIB_SB_DDR_CONFIG_LENGTH_W_WID                           (   1) // 8
#define   BRCMLIB_SB_DDR_CONFIG_ECC_W_OFF                              ( 202) // 1616
#define   BRCMLIB_SB_DDR_CONFIG_ECC_W_WID                              (   1) // 8
#define   BRCMLIB_SB_DDR_CONFIG_BIST_W_OFF                             ( 203) // 1624
#define   BRCMLIB_SB_DDR_CONFIG_BIST_W_WID                             (   1) // 8
#define   BRCMLIB_SB_DDR_CONFIG_PEC_BYTE_W_OFF                         ( 204) // 1632
#define   BRCMLIB_SB_DDR_CONFIG_PEC_BYTE_W_WID                         (   1) // 8
#define   BRCMLIB_SB_OWL_T3C_CONFIG_LENGTH_W_OFF                       ( 205) // 1640
#define   BRCMLIB_SB_OWL_T3C_CONFIG_LENGTH_W_WID                       (   1) // 8
#define   BRCMLIB_SB_OWL_T3C_CONFIG_OWL_W_OFF                          ( 206) // 1648
#define   BRCMLIB_SB_OWL_T3C_CONFIG_OWL_W_WID                          (   4) // 32
#define   BRCMLIB_SB_OWL_T3C_CONFIG_PEC_BYTE_W_OFF                     ( 210) // 1680
#define   BRCMLIB_SB_OWL_T3C_CONFIG_PEC_BYTE_W_WID                     (   1) // 8
#define   BRCMLIB_SB_SMBUS_INDIRECT_LENGTH_W_OFF                       ( 211) // 1688
#define   BRCMLIB_SB_SMBUS_INDIRECT_LENGTH_W_WID                       (   1) // 8
#define   BRCMLIB_SB_SMBUS_INDIRECT_OFFSET_W_OFF                       ( 212) // 1696
#define   BRCMLIB_SB_SMBUS_INDIRECT_OFFSET_W_WID                       (   4) // 32
#define   BRCMLIB_SB_SMBUS_INDIRECT_REGION_W_OFF                       ( 216) // 1728
#define   BRCMLIB_SB_SMBUS_INDIRECT_REGION_W_WID                       (   1) // 8
//#define BRCMLIB_SB_SMBUS_INDIRECT_FLUSH_W_OFF                        ( 217) // 1736
//#define BRCMLIB_SB_SMBUS_INDIRECT_FLUSH_W_WID                        (   0) // 7
//#define BRCMLIB_SB_SMBUS_INDIRECT_NO_INCR_W_OFF                      ( 217) // 1743
//#define BRCMLIB_SB_SMBUS_INDIRECT_NO_INCR_W_WID                      (   0) // 1
#define   BRCMLIB_SB_SMBUS_INDIRECT_PEC_BYTE_W_OFF                     ( 218) // 1744
#define   BRCMLIB_SB_SMBUS_INDIRECT_PEC_BYTE_W_WID                     (   1) // 8

#define MONITORING_STRUCT_HEADER_VERSION (10)
