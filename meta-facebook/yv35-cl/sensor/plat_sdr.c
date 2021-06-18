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

SDR_Full_sensor full_sensor_table[] = {

  {   // TMP75 on board temperature
    0x00,0x00,  // record ID
    IPMI_SDR_VER_15,  // SDR ver
    IPMI_SDR_FULL_SENSOR,   // record type
    IPMI_SDR_HEADER_LEN + IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

    Self_I2C_ADDRESS<<1,   // owner id
    0x00,   // owner lun
    SENSOR_NUM_TEMP_TMP75,    // sensor number
    
    IPMI_SDR_ENTITY_ID_SYS_BOARD,   // entity id
    0x00,   // entity instance
    IPMI_SDR_SENSOR_INIT_SETTABLE| IPMI_SDR_SENSOR_INIT_SCAN| IPMI_SDR_SENSOR_INIT_THRESHOLD| IPMI_SDR_SENSOR_INIT_TYPE| IPMI_SDR_SENSOR_INIT_DEF_EVENT| IPMI_SDR_SENSOR_INIT_DEF_SCAN,   // sensor init
    IPMI_SDR_SENSOR_CAP_THRESHOLD_RW| IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO,    // sensor capabilities
    IPMI_SDR_SENSOR_TYPE_TEMPERATURE,   // sensor type
    IPMI_SDR_EVENT_TYPE_THRESHOLD,    // event/reading type
    0x00,   // assert event mask
    IPMI_SDR_CMP_RETURN_LCT| IPMI_SDR_ASSERT_MASK_UCT_HI| IPMI_SDR_ASSERT_MASK_LCT_LO,    // assert threshold reading mask
    0x00,   // deassert event mask
    IPMI_SDR_CMP_RETURN_UCT| IPMI_SDR_DEASSERT_MASK_UCT_LO| IPMI_SDR_DEASSERT_MASK_LCT_HI,    // deassert threshold reading mask
    0x00,   // discrete reading mask/ settable
    IPMI_SDR_UCT_SETTABLE| IPMI_SDR_LCT_SETTABLE| IPMI_SDR_UCT_READABLE| IPMI_SDR_LCT_READABLE,   // threshold mask/ readable threshold mask
    0x00,   // sensor unit
    0x00,   // base unit 
    0x00,   // modifier unit
    IPMI_SDR_LINEAR_LINEAR,   // linearization
    0x01,   // [7:0] M bits
    0x00,   // [9:8] M bits, tolerance
    0x00,   // [7:0] B bits
    0x00,   // [9:8] B bits, tolerance
    0x00,   // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
    0x00,   // Rexp, Bexp
    0x00,   // analog characteristic
    0x00,   // nominal reading
    0x00,   // normal maximum
    0x00,   // normal minimum
    0x00,   // sensor maximum reading
    0x00,   // sensor minimum reading
    0x00,   // UNRT
    0x00,   // UCT
    0x00,   // UNCT
    0x00,   // LNRT
    0x00,   // LCT
    0x00,   // LNCT
    0x00,   // positive-going threshold
    0x00,   // negative-going threshold
    0x00,   // reserved
    0x00,   // reserved
    0x00,   // OEM
    0x00,   // ID len, should be same as "size of struct"
    "TMP75",
    
  },
  {   // SDR test
    0x00,0x00,  // record ID, fixed in SDR init
    IPMI_SDR_VER_15,  // SDR ver
    IPMI_SDR_FULL_SENSOR,   // record type
    IPMI_SDR_HEADER_LEN + IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

    0x40,   // owner id
    0x00,   // owner lun
    SENSOR_NUM_TEST,    // sensor number

    IPMI_SDR_ENTITY_ID_SYS_BOARD,   // entity id
    0x00,   // entity instance
    IPMI_SDR_SENSOR_INIT_SETTABLE| IPMI_SDR_SENSOR_INIT_SCAN| IPMI_SDR_SENSOR_INIT_THRESHOLD| IPMI_SDR_SENSOR_INIT_TYPE| IPMI_SDR_SENSOR_INIT_DEF_EVENT| IPMI_SDR_SENSOR_INIT_DEF_SCAN,   // sensor init
    IPMI_SDR_SENSOR_CAP_THRESHOLD_RW| IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO,    // sensor capabilities
    IPMI_SDR_SENSOR_TYPE_TEMPERATURE,   // sensor type
    IPMI_SDR_EVENT_TYPE_THRESHOLD,    // event/reading type
    0x00,   // assert event mask
    IPMI_SDR_CMP_RETURN_LCT| IPMI_SDR_ASSERT_MASK_UCT_HI| IPMI_SDR_ASSERT_MASK_LCT_LO,    // assert threshold reading mask
    0x00,   // deassert event mask
    IPMI_SDR_CMP_RETURN_UCT| IPMI_SDR_DEASSERT_MASK_UCT_LO| IPMI_SDR_DEASSERT_MASK_LCT_HI,    // deassert threshold reading mask
    0x00,   // discrete reading mask/ settable
    IPMI_SDR_UCT_SETTABLE| IPMI_SDR_LCT_SETTABLE| IPMI_SDR_UCT_READABLE| IPMI_SDR_LCT_READABLE,   // threshold mask/ readable threshold mask
    0x00,   // sensor unit
    0x00,   // base unit
    0x00,   // modifier unit
    IPMI_SDR_LINEAR_LINEAR,   // linearization
    0x5A,   // [7:0] M bits
    0x12,   // [9:8] M bits, tolerance
    0x34,   // [7:0] B bits
    0x56,   // [9:8] B bits, tolerance
    0x78,   // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
    0x12,   // Rexp, Bexp
    0x34,   // analog characteristic
    0x56,   // nominal reading
    0x78,   // normal maximum
    0x12,   // normal minimum
    0x23,   // sensor maximum reading
    0x34,   // sensor minimum reading
    0x45,   // UNRT
    0x56,   // UCT
    0x67,   // UNCT
    0x78,   // LNRT
    0x89,   // LCT
    0x9a,   // LNCT
    0xab,   // positive-going threshold
    0xbc,   // negative-going threshold
    0xcd,   // reserved
    0xde,   // reserved
    0xef,   // OEM
    0xff,   // ID len, fixed in SDR init
    "TEST_SNR",

  },  

  {   // SDR test0
    0x00,0x00,  // record ID, fixed in SDR init
    IPMI_SDR_VER_15,  // SDR ver
    IPMI_SDR_FULL_SENSOR,   // record type
    IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct, fixed in SDR init

    0x40,   // owner id
    0x00,   // owner lun
    SENSOR_NUM_TEST,    // sensor number

    IPMI_SDR_ENTITY_ID_SYS_BOARD,   // entity id
    0x00,   // entity instance
    IPMI_SDR_SENSOR_INIT_SETTABLE| IPMI_SDR_SENSOR_INIT_SCAN| IPMI_SDR_SENSOR_INIT_THRESHOLD| IPMI_SDR_SENSOR_INIT_TYPE| IPMI_SDR_SENSOR_INIT_DEF_EVENT| IPMI_SDR_SENSOR_INIT_DEF_SCAN,   // sensor init
    IPMI_SDR_SENSOR_CAP_THRESHOLD_RW| IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO,    // sensor capabilities
    IPMI_SDR_SENSOR_TYPE_TEMPERATURE,   // sensor type
    IPMI_SDR_EVENT_TYPE_THRESHOLD,    // event/reading type
    0x00,   // assert event mask
    IPMI_SDR_CMP_RETURN_LCT| IPMI_SDR_ASSERT_MASK_UCT_HI| IPMI_SDR_ASSERT_MASK_LCT_LO,    // assert threshold reading mask
    0x00,   // deassert event mask
    IPMI_SDR_CMP_RETURN_UCT| IPMI_SDR_DEASSERT_MASK_UCT_LO| IPMI_SDR_DEASSERT_MASK_LCT_HI,    // deassert threshold reading mask
    0x00,   // discrete reading mask/ settable
    IPMI_SDR_UCT_SETTABLE| IPMI_SDR_LCT_SETTABLE| IPMI_SDR_UCT_READABLE| IPMI_SDR_LCT_READABLE,   // threshold mask/ readable threshold mask
    0x00,   // sensor unit
    0x00,   // base unit
    0x00,   // modifier unit
    IPMI_SDR_LINEAR_LINEAR,   // linearization
    0x5A,   // [7:0] M bits
    0x00,   // [9:8] M bits, tolerance
    0x00,   // [7:0] B bits
    0x00,   // [9:8] B bits, tolerance
    0x00,   // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
    0xF0,   // Rexp, Bexp
    0x00,   // analog characteristic
    0x00,   // nominal reading
    0x00,   // normal maximum
    0x00,   // normal minimum
    0x00,   // sensor maximum reading
    0x00,   // sensor minimum reading
    0x00,   // UNRT
    0x00,   // UCT
    0x00,   // UNCT
    0x00,   // LNRT
    0x00,   // LCT
    0x00,   // LNCT
    0x00,   // positive-going threshold
    0x00,   // negative-going threshold
    0x00,   // reserved
    0x00,   // reserved
    0x00,   // OEM
    0x00,   // ID len, fixed in SDR init
    "TEST0_SNR",

  },  

};

