#include <stdio.h>
#include <string.h>
#include "sdr.h"
#include "pal.h"
#include "sensor.h"
#include "sensor_def.h"
#include "plat_ipmb.h"

SDR_Full_sensor plat_sensor_table[] = {
  {   // TMP75 on board temperature
    0x00,0x00,  // record ID
    IPMI_SDR_VER_15,  // SDR ver
    IPMI_SDR_FULL_SENSOR,   // record type
    IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

    Self_I2C_ADDRESS<<1,   // owner id
    0x00,   // owner lun
    SENSOR_NUM_TEMP_TMP75_IN,    // sensor number
    
    IPMI_SDR_ENTITY_ID_SYS_BOARD,   // entity id
    0x00,   // entity instance
    IPMI_SDR_SENSOR_INIT_SCAN| IPMI_SDR_SENSOR_INIT_EVENT| IPMI_SDR_SENSOR_INIT_THRESHOLD| IPMI_SDR_SENSOR_INIT_TYPE| IPMI_SDR_SENSOR_INIT_DEF_EVENT| IPMI_SDR_SENSOR_INIT_DEF_SCAN,   // sensor init
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
    0x32,   // UCT
    0x00,   // UNCT
    0x00,   // LNRT
    0x00,   // LCT
    0x00,   // LNCT
    0x00,   // positive-going threshold
    0x00,   // negative-going threshold
    0x00,   // reserved
    0x00,   // reserved
    0x00,   // OEM
    IPMI_SDR_STRING_TYPE_ASCII_8,   // ID len, should be same as "size of struct"
    "MB Inlet Temp",
  },
  {   // TMP75 on board temperature
    0x00,0x00,  // record ID
    IPMI_SDR_VER_15,  // SDR ver
    IPMI_SDR_FULL_SENSOR,   // record type
    IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

    Self_I2C_ADDRESS<<1,   // owner id
    0x00,   // owner lun
    SENSOR_NUM_TEMP_TMP75_OUT,    // sensor number

    IPMI_SDR_ENTITY_ID_SYS_BOARD,   // entity id
    0x00,   // entity instance
    IPMI_SDR_SENSOR_INIT_SCAN| IPMI_SDR_SENSOR_INIT_EVENT| IPMI_SDR_SENSOR_INIT_THRESHOLD| IPMI_SDR_SENSOR_INIT_TYPE| IPMI_SDR_SENSOR_INIT_DEF_EVENT| IPMI_SDR_SENSOR_INIT_DEF_SCAN,   // sensor init
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
    0x32,   // UCT
    0x00,   // UNCT
    0x00,   // LNRT
    0x00,   // LCT
    0x00,   // LNCT
    0x00,   // positive-going threshold
    0x00,   // negative-going threshold
    0x00,   // reserved
    0x00,   // reserved
    0x00,   // OEM
    IPMI_SDR_STRING_TYPE_ASCII_8,   // ID len, should be same as "size of struct"
    "MB Outlet Temp",
  },
  {   // HSC temperature
    0x00,0x00,  // record ID
    IPMI_SDR_VER_15,  // SDR ver
    IPMI_SDR_FULL_SENSOR,   // record type
    IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

    Self_I2C_ADDRESS<<1,   // owner id
    0x00,   // owner lun
    SENSOR_NUM_TEMP_HSC,    // sensor number

    IPMI_SDR_ENTITY_ID_SYS_BOARD,   // entity id
    0x00,   // entity instance
    IPMI_SDR_SENSOR_INIT_SCAN| IPMI_SDR_SENSOR_INIT_EVENT| IPMI_SDR_SENSOR_INIT_THRESHOLD| IPMI_SDR_SENSOR_INIT_TYPE| IPMI_SDR_SENSOR_INIT_DEF_EVENT| IPMI_SDR_SENSOR_INIT_DEF_SCAN,   // sensor init
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
    0x55,   // UCT
    0x00,   // UNCT
    0x00,   // LNRT
    0x00,   // LCT
    0x00,   // LNCT
    0x00,   // positive-going threshold
    0x00,   // negative-going threshold
    0x00,   // reserved
    0x00,   // reserved
    0x00,   // OEM
    IPMI_SDR_STRING_TYPE_ASCII_8,   // ID len, should be same as "size of struct"
    "HSC Temp",
  },
  {   // P5V_STBY ADC voltage 
    0x00,0x00,  // record ID
    IPMI_SDR_VER_15,  // SDR ver
    IPMI_SDR_FULL_SENSOR,   // record type
    IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

    Self_I2C_ADDRESS<<1,   // owner id
    0x00,   // owner lun
    SENSOR_NUM_VOL_P5V_STBY,    // sensor number
    
    IPMI_SDR_ENTITY_ID_SYS_BOARD,   // entity id
    0x00,   // entity instance
    IPMI_SDR_SENSOR_INIT_SCAN| IPMI_SDR_SENSOR_INIT_EVENT| IPMI_SDR_SENSOR_INIT_THRESHOLD| IPMI_SDR_SENSOR_INIT_TYPE| IPMI_SDR_SENSOR_INIT_DEF_EVENT| IPMI_SDR_SENSOR_INIT_DEF_SCAN,   // sensor init
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
    0x40,   // [7:0] M bits
    0x00,   // [9:8] M bits, tolerance
    0x00,   // [7:0] B bits
    0x00,   // [9:8] B bits, tolerance
    0x00,   // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
    0xD0,   // Rexp, Bexp
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
    IPMI_SDR_STRING_TYPE_ASCII_8,   // ID len, should be same as "size of struct"
    "P5V_STBY Vol",
  },
  {   // P12V_STBY ADC voltage 
    0x00,0x00,  // record ID
    IPMI_SDR_VER_15,  // SDR ver
    IPMI_SDR_FULL_SENSOR,   // record type
    IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

    Self_I2C_ADDRESS<<1,   // owner id
    0x00,   // owner lun
    SENSOR_NUM_VOL_P12V_STBY,    // sensor number
    
    IPMI_SDR_ENTITY_ID_SYS_BOARD,   // entity id
    0x00,   // entity instance
    IPMI_SDR_SENSOR_INIT_SCAN| IPMI_SDR_SENSOR_INIT_EVENT| IPMI_SDR_SENSOR_INIT_THRESHOLD| IPMI_SDR_SENSOR_INIT_TYPE| IPMI_SDR_SENSOR_INIT_DEF_EVENT| IPMI_SDR_SENSOR_INIT_DEF_SCAN,   // sensor init
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
    0x11,   // [7:0] M bits
    0x00,   // [9:8] M bits, tolerance
    0x00,   // [7:0] B bits
    0x00,   // [9:8] B bits, tolerance
    0x00,   // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
    0xD0,   // Rexp, Bexp
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
    IPMI_SDR_STRING_TYPE_ASCII_8,   // ID len, should be same as "size of struct"
    "P12V_STBY Vol",
  },
  {   // P3V3 STBY ADC voltage 
    0x00,0x00,  // record ID
    IPMI_SDR_VER_15,  // SDR ver
    IPMI_SDR_FULL_SENSOR,   // record type
    IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

    Self_I2C_ADDRESS<<1,   // owner id
    0x00,   // owner lun
    SENSOR_NUM_VOL_P3V3_STBY,    // sensor number
    
    IPMI_SDR_ENTITY_ID_SYS_BOARD,   // entity id
    0x00,   // entity instance
    IPMI_SDR_SENSOR_INIT_SCAN| IPMI_SDR_SENSOR_INIT_EVENT| IPMI_SDR_SENSOR_INIT_THRESHOLD| IPMI_SDR_SENSOR_INIT_TYPE| IPMI_SDR_SENSOR_INIT_DEF_EVENT| IPMI_SDR_SENSOR_INIT_DEF_SCAN,   // sensor init
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
    0x11,   // [7:0] M bits
    0x00,   // [9:8] M bits, tolerance
    0x00,   // [7:0] B bits
    0x00,   // [9:8] B bits, tolerance
    0x00,   // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
    0xD0,   // Rexp, Bexp
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
    IPMI_SDR_STRING_TYPE_ASCII_8,   // ID len, should be same as "size of struct"
    "P3V3_STBY Vol",
  },
  {   // P5V_USB ADC voltage 
    0x00,0x00,  // record ID
    IPMI_SDR_VER_15,  // SDR ver
    IPMI_SDR_FULL_SENSOR,   // record type
    IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

    Self_I2C_ADDRESS<<1,   // owner id
    0x00,   // owner lun
    SENSOR_NUM_VOL_P5V_USB,    // sensor number
    
    IPMI_SDR_ENTITY_ID_SYS_BOARD,   // entity id
    0x00,   // entity instance
    IPMI_SDR_SENSOR_INIT_SCAN| IPMI_SDR_SENSOR_INIT_EVENT| IPMI_SDR_SENSOR_INIT_THRESHOLD| IPMI_SDR_SENSOR_INIT_TYPE| IPMI_SDR_SENSOR_INIT_DEF_EVENT| IPMI_SDR_SENSOR_INIT_DEF_SCAN,   // sensor init
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
    0x37,   // [7:0] M bits
    0x00,   // [9:8] M bits, tolerance
    0x00,   // [7:0] B bits
    0x00,   // [9:8] B bits, tolerance
    0x00,   // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
    0xC0,   // Rexp, Bexp
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
    IPMI_SDR_STRING_TYPE_ASCII_8,   // ID len, should be same as "size of struct"
    "P5V_USB Vol",
  },
  {   // P1V2_BIC_STBY ADC voltage 
    0x00,0x00,  // record ID
    IPMI_SDR_VER_15,  // SDR ver
    IPMI_SDR_FULL_SENSOR,   // record type
    IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

    Self_I2C_ADDRESS<<1,   // owner id
    0x00,   // owner lun
    SENSOR_NUM_VOL_P1V2_BIC_STBY,    // sensor number
    
    IPMI_SDR_ENTITY_ID_SYS_BOARD,   // entity id
    0x00,   // entity instance
    IPMI_SDR_SENSOR_INIT_SCAN| IPMI_SDR_SENSOR_INIT_EVENT| IPMI_SDR_SENSOR_INIT_THRESHOLD| IPMI_SDR_SENSOR_INIT_TYPE| IPMI_SDR_SENSOR_INIT_DEF_EVENT| IPMI_SDR_SENSOR_INIT_DEF_SCAN,   // sensor init
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
    0xE0,   // Rexp, Bexp
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
    IPMI_SDR_STRING_TYPE_ASCII_8,   // ID len, should be same as "size of struct"
    "P1V2_BIC_STBY Vol",
  },
  {   // P1V0_STBY voltage
    0x00,0x00,  // record ID
    IPMI_SDR_VER_15,  // SDR ver
    IPMI_SDR_FULL_SENSOR,   // record type
    IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

    Self_I2C_ADDRESS<<1,   // owner id
    0x00,   // owner lun
    SENSOR_NUM_VOL_P1V0_STBY,    // sensor number

    IPMI_SDR_ENTITY_ID_SYS_BOARD,   // entity id
    0x00,   // entity instance
    IPMI_SDR_SENSOR_INIT_SCAN| IPMI_SDR_SENSOR_INIT_EVENT| IPMI_SDR_SENSOR_INIT_THRESHOLD| IPMI_SDR_SENSOR_INIT_TYPE| IPMI_SDR_SENSOR_INIT_DEF_EVENT| IPMI_SDR_SENSOR_INIT_DEF_SCAN,   // sensor init
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
    0x5E,   // [7:0] M bits
    0x00,   // [9:8] M bits, tolerance
    0x00,   // [7:0] B bits
    0x00,   // [9:8] B bits, tolerance
    0x00,   // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
    0xC0,   // Rexp, Bexp
    0x00,   // analog characteristic
    0x00,   // nominal reading
    0x00,   // normal maximum
    0x00,   // normal minimum
    0x00,   // sensor maximum reading
    0x00,   // sensor minimum reading
    0x00,   // UNRT
    0xC8,   // UCT
    0x00,   // UNCT
    0x00,   // LNRT
    0x00,   // LCT
    0x00,   // LNCT
    0x00,   // positive-going threshold
    0x00,   // negative-going threshold
    0x00,   // reserved
    0x00,   // reserved
    0x00,   // OEM
    IPMI_SDR_STRING_TYPE_ASCII_8,   // ID len, should be same as "size of struct"
    "P1V0_STBY Vol",
  },
  {   // MEDUSA_12V_IN voltage
    0x00,0x00,  // record ID
    IPMI_SDR_VER_15,  // SDR ver
    IPMI_SDR_FULL_SENSOR,   // record type
    IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

    Self_I2C_ADDRESS<<1,   // owner id
    0x00,   // owner lun
    SENSOR_NUM_VOL_MEDUSA_12V_IN,    // sensor number

    IPMI_SDR_ENTITY_ID_SYS_BOARD,   // entity id
    0x00,   // entity instance
    IPMI_SDR_SENSOR_INIT_SCAN| IPMI_SDR_SENSOR_INIT_EVENT| IPMI_SDR_SENSOR_INIT_THRESHOLD| IPMI_SDR_SENSOR_INIT_TYPE| IPMI_SDR_SENSOR_INIT_DEF_EVENT| IPMI_SDR_SENSOR_INIT_DEF_SCAN,   // sensor init
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
    0x5E,   // [7:0] M bits
    0x00,   // [9:8] M bits, tolerance
    0x00,   // [7:0] B bits
    0x00,   // [9:8] B bits, tolerance
    0x00,   // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
    0xC0,   // Rexp, Bexp
    0x00,   // analog characteristic
    0x00,   // nominal reading
    0x00,   // normal maximum
    0x00,   // normal minimum
    0x00,   // sensor maximum reading
    0x00,   // sensor minimum reading
    0x00,   // UNRT
    0xC8,   // UCT
    0x00,   // UNCT
    0x00,   // LNRT
    0x00,   // LCT
    0x00,   // LNCT
    0x00,   // positive-going threshold
    0x00,   // negative-going threshold
    0x00,   // reserved
    0x00,   // reserved
    0x00,   // OEM
    IPMI_SDR_STRING_TYPE_ASCII_8,   // ID len, should be same as "size of struct"
    "MEDUSA_12V_IN Vol",
  },
  {   // MEDUSA_12V_OUT voltage
    0x00,0x00,  // record ID
    IPMI_SDR_VER_15,  // SDR ver
    IPMI_SDR_FULL_SENSOR,   // record type
    IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

    Self_I2C_ADDRESS<<1,   // owner id
    0x00,   // owner lun
    SENSOR_NUM_VOL_MEDUSA_12V_OUT,    // sensor number

    IPMI_SDR_ENTITY_ID_SYS_BOARD,   // entity id
    0x00,   // entity instance
    IPMI_SDR_SENSOR_INIT_SCAN| IPMI_SDR_SENSOR_INIT_EVENT| IPMI_SDR_SENSOR_INIT_THRESHOLD| IPMI_SDR_SENSOR_INIT_TYPE| IPMI_SDR_SENSOR_INIT_DEF_EVENT| IPMI_SDR_SENSOR_INIT_DEF_SCAN,   // sensor init
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
    0x5E,   // [7:0] M bits
    0x00,   // [9:8] M bits, tolerance
    0x00,   // [7:0] B bits
    0x00,   // [9:8] B bits, tolerance
    0x00,   // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
    0xC0,   // Rexp, Bexp
    0x00,   // analog characteristic
    0x00,   // nominal reading
    0x00,   // normal maximum
    0x00,   // normal minimum
    0x00,   // sensor maximum reading
    0x00,   // sensor minimum reading
    0x00,   // UNRT
    0xC8,   // UCT
    0x00,   // UNCT
    0x00,   // LNRT
    0x00,   // LCT
    0x00,   // LNCT
    0x00,   // positive-going threshold
    0x00,   // negative-going threshold
    0x00,   // reserved
    0x00,   // reserved
    0x00,   // OEM
    IPMI_SDR_STRING_TYPE_ASCII_8,   // ID len, should be same as "size of struct"
    "MEDUSA_12V_OUT Vol",
  },
  {   // HSCIN voltage
   0x00,0x00,  // record ID
   IPMI_SDR_VER_15,  // SDR ver
   IPMI_SDR_FULL_SENSOR,   // record type
   IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

   Self_I2C_ADDRESS<<1,   // owner id
   0x00,   // owner lun
   SENSOR_NUM_VOL_HSCIN,    // sensor number

   IPMI_SDR_ENTITY_ID_SYS_BOARD,   // entity id
   0x00,   // entity instance
   IPMI_SDR_SENSOR_INIT_SCAN| IPMI_SDR_SENSOR_INIT_EVENT| IPMI_SDR_SENSOR_INIT_THRESHOLD| IPMI_SDR_SENSOR_INIT_TYPE| IPMI_SDR_SENSOR_INIT_DEF_EVENT| IPMI_SDR_SENSOR_INIT_DEF_SCAN,   // sensor init
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
   0x41,   // [7:0] M bits
   0x80,   // [9:8] M bits, tolerance
   0x00,   // [7:0] B bits
   0x00,   // [9:8] B bits, tolerance
   0x00,   // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
   0xB0,   // Rexp, Bexp
   0x00,   // analog characteristic
   0x00,   // nominal reading
   0x00,   // normal maximum
   0x00,   // normal minimum
   0x00,   // sensor maximum reading
   0x00,   // sensor minimum reading
   0x00,   // UNRT
   0xC8,   // UCT
   0x00,   // UNCT
   0x00,   // LNRT
   0x00,   // LCT
   0x00,   // LNCT
   0x00,   // positive-going threshold
   0x00,   // negative-going threshold
   0x00,   // reserved
   0x00,   // reserved
   0x00,   // OEM
   IPMI_SDR_STRING_TYPE_ASCII_8,   // ID len, should be same as "size of struct"
   "HSCIN Vol",
 },
 {   // MEDUSA_IOUT current
  0x00,0x00,  // record ID
  IPMI_SDR_VER_15,  // SDR ver
  IPMI_SDR_FULL_SENSOR,   // record type
  IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

  Self_I2C_ADDRESS<<1,   // owner id
  0x00,   // owner lun
  SENSOR_NUM_CUR_MEDUSA_IOUT,    // sensor number

  IPMI_SDR_ENTITY_ID_SYS_BOARD,   // entity id
  0x00,   // entity instance
  IPMI_SDR_SENSOR_INIT_SCAN| IPMI_SDR_SENSOR_INIT_EVENT| IPMI_SDR_SENSOR_INIT_THRESHOLD| IPMI_SDR_SENSOR_INIT_TYPE| IPMI_SDR_SENSOR_INIT_DEF_EVENT| IPMI_SDR_SENSOR_INIT_DEF_SCAN,   // sensor init
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
  0x7D,   // [7:0] M bits
  0x00,   // [9:8] M bits, tolerance
  0x00,   // [7:0] B bits
  0x00,   // [9:8] B bits, tolerance
  0x00,   // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
  0xD0,   // Rexp, Bexp
  0x00,   // analog characteristic
  0x00,   // nominal reading
  0x00,   // normal maximum
  0x00,   // normal minimum
  0x00,   // sensor maximum reading
  0x00,   // sensor minimum reading
  0x00,   // UNRT
  0xFF,   // UCT
  0x00,   // UNCT
  0x00,   // LNRT
  0x00,   // LCT
  0x00,   // LNCT
  0x00,   // positive-going threshold
  0x00,   // negative-going threshold
  0x00,   // reserved
  0x00,   // reserved
  0x00,   // OEM
  IPMI_SDR_STRING_TYPE_ASCII_8,   // ID len, should be same as "size of struct"
  "MEDUSA_IOUT Cur",
 },
  {   // HSCOUT current
   0x00,0x00,  // record ID
   IPMI_SDR_VER_15,  // SDR ver
   IPMI_SDR_FULL_SENSOR,   // record type
   IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

   Self_I2C_ADDRESS<<1,   // owner id
   0x00,   // owner lun
   SENSOR_NUM_CUR_HSCOUT,    // sensor number

   IPMI_SDR_ENTITY_ID_SYS_BOARD,   // entity id
   0x00,   // entity instance
   IPMI_SDR_SENSOR_INIT_SCAN| IPMI_SDR_SENSOR_INIT_EVENT| IPMI_SDR_SENSOR_INIT_THRESHOLD| IPMI_SDR_SENSOR_INIT_TYPE| IPMI_SDR_SENSOR_INIT_DEF_EVENT| IPMI_SDR_SENSOR_INIT_DEF_SCAN,   // sensor init
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
   0x2E,   // [7:0] M bits
   0x00,   // [9:8] M bits, tolerance
   0x00,   // [7:0] B bits
   0x00,   // [9:8] B bits, tolerance
   0x00,   // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
   0xE0,   // Rexp, Bexp
   0x00,   // analog characteristic
   0x00,   // nominal reading
   0x00,   // normal maximum
   0x00,   // normal minimum
   0x00,   // sensor maximum reading
   0x00,   // sensor minimum reading
   0x00,   // UNRT
   0xC8,   // UCT
   0x00,   // UNCT
   0x00,   // LNRT
   0x00,   // LCT
   0x00,   // LNCT
   0x00,   // positive-going threshold
   0x00,   // negative-going threshold
   0x00,   // reserved
   0x00,   // reserved
   0x00,   // OEM
   IPMI_SDR_STRING_TYPE_ASCII_8,   // ID len, should be same as "size of struct"
   "HSCOUT Cur",
 },
 {   // P12V_FAN current
   0x00,0x00,  // record ID
   IPMI_SDR_VER_15,  // SDR ver
   IPMI_SDR_FULL_SENSOR,   // record type
   IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

   Self_I2C_ADDRESS<<1,   // owner id
   0x00,   // owner lun
   SENSOR_NUM_CUR_P12V_FAN,    // sensor number

   IPMI_SDR_ENTITY_ID_SYS_BOARD,   // entity id
   0x00,   // entity instance
   IPMI_SDR_SENSOR_INIT_SCAN| IPMI_SDR_SENSOR_INIT_EVENT| IPMI_SDR_SENSOR_INIT_THRESHOLD| IPMI_SDR_SENSOR_INIT_TYPE| IPMI_SDR_SENSOR_INIT_DEF_EVENT| IPMI_SDR_SENSOR_INIT_DEF_SCAN,   // sensor init
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
   0x1A,   // [7:0] M bits
   0x00,   // [9:8] M bits, tolerance
   0x00,   // [7:0] B bits
   0x00,   // [9:8] B bits, tolerance
   0x00,   // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
   0xE0,   // Rexp, Bexp
   0x00,   // analog characteristic
   0x00,   // nominal reading
   0x00,   // normal maximum
   0x00,   // normal minimum
   0x00,   // sensor maximum reading
   0x00,   // sensor minimum reading
   0x00,   // UNRT
   0xC8,   // UCT
   0x00,   // UNCT
   0x00,   // LNRT
   0x00,   // LCT
   0x00,   // LNCT
   0x00,   // positive-going threshold
   0x00,   // negative-going threshold
   0x00,   // reserved
   0x00,   // reserved
   0x00,   // OEM
   IPMI_SDR_STRING_TYPE_ASCII_8,   // ID len, should be same as "size of struct"
   "P12V_FAN Cur",
 },
 {   // MEDUSA_12V power
  0x00,0x00,  // record ID
  IPMI_SDR_VER_15,  // SDR ver
  IPMI_SDR_FULL_SENSOR,   // record type
  IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

  Self_I2C_ADDRESS<<1,   // owner id
  0x00,   // owner lun
  SENSOR_NUM_PWR_MEDUSA_12V,    // sensor number

  IPMI_SDR_ENTITY_ID_SYS_BOARD,   // entity id
  0x00,   // entity instance
  IPMI_SDR_SENSOR_INIT_SCAN| IPMI_SDR_SENSOR_INIT_EVENT| IPMI_SDR_SENSOR_INIT_THRESHOLD| IPMI_SDR_SENSOR_INIT_TYPE| IPMI_SDR_SENSOR_INIT_DEF_EVENT| IPMI_SDR_SENSOR_INIT_DEF_SCAN,   // sensor init
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
  0x9C,   // [7:0] M bits
  0x00,   // [9:8] M bits, tolerance
  0x00,   // [7:0] B bits
  0x00,   // [9:8] B bits, tolerance
  0x00,   // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
  0xE0,   // Rexp, Bexp
  0x00,   // analog characteristic
  0x00,   // nominal reading
  0x00,   // normal maximum
  0x00,   // normal minimum
  0x00,   // sensor maximum reading
  0x00,   // sensor minimum reading
  0x00,   // UNRT
  0xFF,   // UCT
  0x00,   // UNCT
  0x00,   // LNRT
  0x00,   // LCT
  0x00,   // LNCT
  0x00,   // positive-going threshold
  0x00,   // negative-going threshold
  0x00,   // reserved
  0x00,   // reserved
  0x00,   // OEM
  IPMI_SDR_STRING_TYPE_ASCII_8,   // ID len, should be same as "size of struct"
  "MEDUSA_12V Pwr",
 },
 {   // HSCIN power
   0x00,0x00,  // record ID
   IPMI_SDR_VER_15,  // SDR ver
   IPMI_SDR_FULL_SENSOR,   // record type
   IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

   Self_I2C_ADDRESS<<1,   // owner id
   0x00,   // owner lun
   SENSOR_NUM_PWR_HSCIN,    // sensor number

   IPMI_SDR_ENTITY_ID_SYS_BOARD,   // entity id
   0x00,   // entity instance
   IPMI_SDR_SENSOR_INIT_SCAN| IPMI_SDR_SENSOR_INIT_EVENT| IPMI_SDR_SENSOR_INIT_THRESHOLD| IPMI_SDR_SENSOR_INIT_TYPE| IPMI_SDR_SENSOR_INIT_DEF_EVENT| IPMI_SDR_SENSOR_INIT_DEF_SCAN,   // sensor init
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
   0x52,   // [7:0] M bits
   0x00,   // [9:8] M bits, tolerance
   0x00,   // [7:0] B bits
   0x00,   // [9:8] B bits, tolerance
   0x00,   // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
   0xE0,   // Rexp, Bexp
   0x00,   // analog characteristic
   0x00,   // nominal reading
   0x00,   // normal maximum
   0x00,   // normal minimum
   0x00,   // sensor maximum reading
   0x00,   // sensor minimum reading
   0x00,   // UNRT
   0xC8,   // UCT
   0x00,   // UNCT
   0x00,   // LNRT
   0x00,   // LCT
   0x00,   // LNCT
   0x00,   // positive-going threshold
   0x00,   // negative-going threshold
   0x00,   // reserved
   0x00,   // reserved
   0x00,   // OEM
   IPMI_SDR_STRING_TYPE_ASCII_8,   // ID len, should be same as "size of struct"
   "HSCIN Pwr",
 },
 {   // DUAL_FAN_BMC_TACH_0
   0x00,0x00,  // record ID
   IPMI_SDR_VER_15,  // SDR ver
   IPMI_SDR_FULL_SENSOR,   // record type
   IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

   Self_I2C_ADDRESS<<1,   // owner id
   0x00,   // owner lun
   SENSOR_NUM_DUAL_FAN_BMC_TACH_0,    // sensor number

   IPMI_SDR_ENTITY_ID_SYS_BOARD,   // entity id
   0x00,   // entity instance
   IPMI_SDR_SENSOR_INIT_SCAN| IPMI_SDR_SENSOR_INIT_EVENT| IPMI_SDR_SENSOR_INIT_THRESHOLD| IPMI_SDR_SENSOR_INIT_TYPE| IPMI_SDR_SENSOR_INIT_DEF_EVENT| IPMI_SDR_SENSOR_INIT_DEF_SCAN,   // sensor init
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
   0x17,   // [7:0] M bits
   0x00,   // [9:8] M bits, tolerance
   0x00,   // [7:0] B bits
   0x00,   // [9:8] B bits, tolerance
   0x00,   // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
   0xE0,   // Rexp, Bexp
   0x00,   // analog characteristic
   0x00,   // nominal reading
   0x00,   // normal maximum
   0x00,   // normal minimum
   0x00,   // sensor maximum reading
   0x00,   // sensor minimum reading
   0x00,   // UNRT
   0xC8,   // UCT
   0x00,   // UNCT
   0x00,   // LNRT
   0x00,   // LCT
   0x00,   // LNCT
   0x00,   // positive-going threshold
   0x00,   // negative-going threshold
   0x00,   // reserved
   0x00,   // reserved
   0x00,   // OEM
   IPMI_SDR_STRING_TYPE_ASCII_8,   // ID len, should be same as "size of struct"
   "DUAL_FAN_BMC_TACH_0",
 },
 {   // DUAL_FAN_BMC_TACH_1
   0x00,0x00,  // record ID
   IPMI_SDR_VER_15,  // SDR ver
   IPMI_SDR_FULL_SENSOR,   // record type
   IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

   Self_I2C_ADDRESS<<1,   // owner id
   0x00,   // owner lun
   SENSOR_NUM_DUAL_FAN_BMC_TACH_1,    // sensor number

   IPMI_SDR_ENTITY_ID_SYS_BOARD,   // entity id
   0x00,   // entity instance
   IPMI_SDR_SENSOR_INIT_SCAN| IPMI_SDR_SENSOR_INIT_EVENT| IPMI_SDR_SENSOR_INIT_THRESHOLD| IPMI_SDR_SENSOR_INIT_TYPE| IPMI_SDR_SENSOR_INIT_DEF_EVENT| IPMI_SDR_SENSOR_INIT_DEF_SCAN,   // sensor init
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
   0x17,   // [7:0] M bits
   0x00,   // [9:8] M bits, tolerance
   0x00,   // [7:0] B bits
   0x00,   // [9:8] B bits, tolerance
   0x00,   // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
   0xE0,   // Rexp, Bexp
   0x00,   // analog characteristic
   0x00,   // nominal reading
   0x00,   // normal maximum
   0x00,   // normal minimum
   0x00,   // sensor maximum reading
   0x00,   // sensor minimum reading
   0x00,   // UNRT
   0xC8,   // UCT
   0x00,   // UNCT
   0x00,   // LNRT
   0x00,   // LCT
   0x00,   // LNCT
   0x00,   // positive-going threshold
   0x00,   // negative-going threshold
   0x00,   // reserved
   0x00,   // reserved
   0x00,   // OEM
   IPMI_SDR_STRING_TYPE_ASCII_8,   // ID len, should be same as "size of struct"
   "DUAL_FAN_BMC_TACH_1",
 },
 {   // DUAL_FAN_BMC_TACH_2
   0x00,0x00,  // record ID
   IPMI_SDR_VER_15,  // SDR ver
   IPMI_SDR_FULL_SENSOR,   // record type
   IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

   Self_I2C_ADDRESS<<1,   // owner id
   0x00,   // owner lun
   SENSOR_NUM_DUAL_FAN_BMC_TACH_2,    // sensor number

   IPMI_SDR_ENTITY_ID_SYS_BOARD,   // entity id
   0x00,   // entity instance
   IPMI_SDR_SENSOR_INIT_SCAN| IPMI_SDR_SENSOR_INIT_EVENT| IPMI_SDR_SENSOR_INIT_THRESHOLD| IPMI_SDR_SENSOR_INIT_TYPE| IPMI_SDR_SENSOR_INIT_DEF_EVENT| IPMI_SDR_SENSOR_INIT_DEF_SCAN,   // sensor init
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
   0x17,   // [7:0] M bits
   0x00,   // [9:8] M bits, tolerance
   0x00,   // [7:0] B bits
   0x00,   // [9:8] B bits, tolerance
   0x00,   // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
   0xE0,   // Rexp, Bexp
   0x00,   // analog characteristic
   0x00,   // nominal reading
   0x00,   // normal maximum
   0x00,   // normal minimum
   0x00,   // sensor maximum reading
   0x00,   // sensor minimum reading
   0x00,   // UNRT
   0xC8,   // UCT
   0x00,   // UNCT
   0x00,   // LNRT
   0x00,   // LCT
   0x00,   // LNCT
   0x00,   // positive-going threshold
   0x00,   // negative-going threshold
   0x00,   // reserved
   0x00,   // reserved
   0x00,   // OEM
   IPMI_SDR_STRING_TYPE_ASCII_8,   // ID len, should be same as "size of struct"
   "DUAL_FAN_BMC_TACH_2",
 },
 {   // DUAL_FAN_BMC_TACH_3
   0x00,0x00,  // record ID
   IPMI_SDR_VER_15,  // SDR ver
   IPMI_SDR_FULL_SENSOR,   // record type
   IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

   Self_I2C_ADDRESS<<1,   // owner id
   0x00,   // owner lun
   SENSOR_NUM_DUAL_FAN_BMC_TACH_3,    // sensor number

   IPMI_SDR_ENTITY_ID_SYS_BOARD,   // entity id
   0x00,   // entity instance
   IPMI_SDR_SENSOR_INIT_SCAN| IPMI_SDR_SENSOR_INIT_EVENT| IPMI_SDR_SENSOR_INIT_THRESHOLD| IPMI_SDR_SENSOR_INIT_TYPE| IPMI_SDR_SENSOR_INIT_DEF_EVENT| IPMI_SDR_SENSOR_INIT_DEF_SCAN,   // sensor init
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
   0x17,   // [7:0] M bits
   0x00,   // [9:8] M bits, tolerance
   0x00,   // [7:0] B bits
   0x00,   // [9:8] B bits, tolerance
   0x00,   // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
   0xE0,   // Rexp, Bexp
   0x00,   // analog characteristic
   0x00,   // nominal reading
   0x00,   // normal maximum
   0x00,   // normal minimum
   0x00,   // sensor maximum reading
   0x00,   // sensor minimum reading
   0x00,   // UNRT
   0xC8,   // UCT
   0x00,   // UNCT
   0x00,   // LNRT
   0x00,   // LCT
   0x00,   // LNCT
   0x00,   // positive-going threshold
   0x00,   // negative-going threshold
   0x00,   // reserved
   0x00,   // reserved
   0x00,   // OEM
   IPMI_SDR_STRING_TYPE_ASCII_8,   // ID len, should be same as "size of struct"
   "DUAL_FAN_BMC_TACH_3",
 },
 {   // DUAL_FAN_BMC_TACH_4
   0x00,0x00,  // record ID
   IPMI_SDR_VER_15,  // SDR ver
   IPMI_SDR_FULL_SENSOR,   // record type
   IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

   Self_I2C_ADDRESS<<1,   // owner id
   0x00,   // owner lun
   SENSOR_NUM_DUAL_FAN_BMC_TACH_4,    // sensor number

   IPMI_SDR_ENTITY_ID_SYS_BOARD,   // entity id
   0x00,   // entity instance
   IPMI_SDR_SENSOR_INIT_SCAN| IPMI_SDR_SENSOR_INIT_EVENT| IPMI_SDR_SENSOR_INIT_THRESHOLD| IPMI_SDR_SENSOR_INIT_TYPE| IPMI_SDR_SENSOR_INIT_DEF_EVENT| IPMI_SDR_SENSOR_INIT_DEF_SCAN,   // sensor init
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
   0x17,   // [7:0] M bits
   0x00,   // [9:8] M bits, tolerance
   0x00,   // [7:0] B bits
   0x00,   // [9:8] B bits, tolerance
   0x00,   // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
   0xE0,   // Rexp, Bexp
   0x00,   // analog characteristic
   0x00,   // nominal reading
   0x00,   // normal maximum
   0x00,   // normal minimum
   0x00,   // sensor maximum reading
   0x00,   // sensor minimum reading
   0x00,   // UNRT
   0xC8,   // UCT
   0x00,   // UNCT
   0x00,   // LNRT
   0x00,   // LCT
   0x00,   // LNCT
   0x00,   // positive-going threshold
   0x00,   // negative-going threshold
   0x00,   // reserved
   0x00,   // reserved
   0x00,   // OEM
   IPMI_SDR_STRING_TYPE_ASCII_8,   // ID len, should be same as "size of struct"
   "DUAL_FAN_BMC_TACH_4",
 },
 {   // DUAL_FAN_BMC_TACH_5
   0x00,0x00,  // record ID
   IPMI_SDR_VER_15,  // SDR ver
   IPMI_SDR_FULL_SENSOR,   // record type
   IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

   Self_I2C_ADDRESS<<1,   // owner id
   0x00,   // owner lun
   SENSOR_NUM_DUAL_FAN_BMC_TACH_5,    // sensor number

   IPMI_SDR_ENTITY_ID_SYS_BOARD,   // entity id
   0x00,   // entity instance
   IPMI_SDR_SENSOR_INIT_SCAN| IPMI_SDR_SENSOR_INIT_EVENT| IPMI_SDR_SENSOR_INIT_THRESHOLD| IPMI_SDR_SENSOR_INIT_TYPE| IPMI_SDR_SENSOR_INIT_DEF_EVENT| IPMI_SDR_SENSOR_INIT_DEF_SCAN,   // sensor init
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
   0x17,   // [7:0] M bits
   0x00,   // [9:8] M bits, tolerance
   0x00,   // [7:0] B bits
   0x00,   // [9:8] B bits, tolerance
   0x00,   // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
   0xE0,   // Rexp, Bexp
   0x00,   // analog characteristic
   0x00,   // nominal reading
   0x00,   // normal maximum
   0x00,   // normal minimum
   0x00,   // sensor maximum reading
   0x00,   // sensor minimum reading
   0x00,   // UNRT
   0xC8,   // UCT
   0x00,   // UNCT
   0x00,   // LNRT
   0x00,   // LCT
   0x00,   // LNCT
   0x00,   // positive-going threshold
   0x00,   // negative-going threshold
   0x00,   // reserved
   0x00,   // reserved
   0x00,   // OEM
   IPMI_SDR_STRING_TYPE_ASCII_8,   // ID len, should be same as "size of struct"
   "DUAL_FAN_BMC_TACH_5",
 },
 {   // DUAL_FAN_BMC_TACH_6
   0x00,0x00,  // record ID
   IPMI_SDR_VER_15,  // SDR ver
   IPMI_SDR_FULL_SENSOR,   // record type
   IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

   Self_I2C_ADDRESS<<1,   // owner id
   0x00,   // owner lun
   SENSOR_NUM_DUAL_FAN_BMC_TACH_6,    // sensor number

   IPMI_SDR_ENTITY_ID_SYS_BOARD,   // entity id
   0x00,   // entity instance
   IPMI_SDR_SENSOR_INIT_SCAN| IPMI_SDR_SENSOR_INIT_EVENT| IPMI_SDR_SENSOR_INIT_THRESHOLD| IPMI_SDR_SENSOR_INIT_TYPE| IPMI_SDR_SENSOR_INIT_DEF_EVENT| IPMI_SDR_SENSOR_INIT_DEF_SCAN,   // sensor init
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
   0x17,   // [7:0] M bits
   0x00,   // [9:8] M bits, tolerance
   0x00,   // [7:0] B bits
   0x00,   // [9:8] B bits, tolerance
   0x00,   // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
   0xE0,   // Rexp, Bexp
   0x00,   // analog characteristic
   0x00,   // nominal reading
   0x00,   // normal maximum
   0x00,   // normal minimum
   0x00,   // sensor maximum reading
   0x00,   // sensor minimum reading
   0x00,   // UNRT
   0xC8,   // UCT
   0x00,   // UNCT
   0x00,   // LNRT
   0x00,   // LCT
   0x00,   // LNCT
   0x00,   // positive-going threshold
   0x00,   // negative-going threshold
   0x00,   // reserved
   0x00,   // reserved
   0x00,   // OEM
   IPMI_SDR_STRING_TYPE_ASCII_8,   // ID len, should be same as "size of struct"
   "DUAL_FAN_BMC_TACH_6",
 },
 {   // DUAL_FAN_BMC_TACH_7
   0x00,0x00,  // record ID
   IPMI_SDR_VER_15,  // SDR ver
   IPMI_SDR_FULL_SENSOR,   // record type
   IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

   Self_I2C_ADDRESS<<1,   // owner id
   0x00,   // owner lun
   SENSOR_NUM_DUAL_FAN_BMC_TACH_7,    // sensor number

   IPMI_SDR_ENTITY_ID_SYS_BOARD,   // entity id
   0x00,   // entity instance
   IPMI_SDR_SENSOR_INIT_SCAN| IPMI_SDR_SENSOR_INIT_EVENT| IPMI_SDR_SENSOR_INIT_THRESHOLD| IPMI_SDR_SENSOR_INIT_TYPE| IPMI_SDR_SENSOR_INIT_DEF_EVENT| IPMI_SDR_SENSOR_INIT_DEF_SCAN,   // sensor init
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
   0x17,   // [7:0] M bits
   0x00,   // [9:8] M bits, tolerance
   0x00,   // [7:0] B bits
   0x00,   // [9:8] B bits, tolerance
   0x00,   // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
   0xE0,   // Rexp, Bexp
   0x00,   // analog characteristic
   0x00,   // nominal reading
   0x00,   // normal maximum
   0x00,   // normal minimum
   0x00,   // sensor maximum reading
   0x00,   // sensor minimum reading
   0x00,   // UNRT
   0xC8,   // UCT
   0x00,   // UNCT
   0x00,   // LNRT
   0x00,   // LCT
   0x00,   // LNCT
   0x00,   // positive-going threshold
   0x00,   // negative-going threshold
   0x00,   // reserved
   0x00,   // reserved
   0x00,   // OEM
   IPMI_SDR_STRING_TYPE_ASCII_8,   // ID len, should be same as "size of struct"
   "DUAL_FAN_BMC_TACH_7",
 },
 {   // SINGLE_FAN_BMC_TACH_0
   0x00,0x00,  // record ID
   IPMI_SDR_VER_15,  // SDR ver
   IPMI_SDR_FULL_SENSOR,   // record type
   IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

   Self_I2C_ADDRESS<<1,   // owner id
   0x00,   // owner lun
   SENSOR_NUM_SINGLE_FAN_BMC_TACH_0,    // sensor number

   IPMI_SDR_ENTITY_ID_SYS_BOARD,   // entity id
   0x00,   // entity instance
   IPMI_SDR_SENSOR_INIT_SCAN| IPMI_SDR_SENSOR_INIT_EVENT| IPMI_SDR_SENSOR_INIT_THRESHOLD| IPMI_SDR_SENSOR_INIT_TYPE| IPMI_SDR_SENSOR_INIT_DEF_EVENT| IPMI_SDR_SENSOR_INIT_DEF_SCAN,   // sensor init
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
   0x17,   // [7:0] M bits
   0x00,   // [9:8] M bits, tolerance
   0x00,   // [7:0] B bits
   0x00,   // [9:8] B bits, tolerance
   0x00,   // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
   0xE0,   // Rexp, Bexp
   0x00,   // analog characteristic
   0x00,   // nominal reading
   0x00,   // normal maximum
   0x00,   // normal minimum
   0x00,   // sensor maximum reading
   0x00,   // sensor minimum reading
   0x00,   // UNRT
   0xC8,   // UCT
   0x00,   // UNCT
   0x00,   // LNRT
   0x00,   // LCT
   0x00,   // LNCT
   0x00,   // positive-going threshold
   0x00,   // negative-going threshold
   0x00,   // reserved
   0x00,   // reserved
   0x00,   // OEM
   IPMI_SDR_STRING_TYPE_ASCII_8,   // ID len, should be same as "size of struct"
   "SINGLE_FAN_BMC_TACH_0",
 },
 {   // SINGLE_FAN_BMC_TACH_1
   0x00,0x00,  // record ID
   IPMI_SDR_VER_15,  // SDR ver
   IPMI_SDR_FULL_SENSOR,   // record type
   IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

   Self_I2C_ADDRESS<<1,   // owner id
   0x00,   // owner lun
   SENSOR_NUM_SINGLE_FAN_BMC_TACH_1,    // sensor number

   IPMI_SDR_ENTITY_ID_SYS_BOARD,   // entity id
   0x00,   // entity instance
   IPMI_SDR_SENSOR_INIT_SCAN| IPMI_SDR_SENSOR_INIT_EVENT| IPMI_SDR_SENSOR_INIT_THRESHOLD| IPMI_SDR_SENSOR_INIT_TYPE| IPMI_SDR_SENSOR_INIT_DEF_EVENT| IPMI_SDR_SENSOR_INIT_DEF_SCAN,   // sensor init
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
   0x17,   // [7:0] M bits
   0x00,   // [9:8] M bits, tolerance
   0x00,   // [7:0] B bits
   0x00,   // [9:8] B bits, tolerance
   0x00,   // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
   0xE0,   // Rexp, Bexp
   0x00,   // analog characteristic
   0x00,   // nominal reading
   0x00,   // normal maximum
   0x00,   // normal minimum
   0x00,   // sensor maximum reading
   0x00,   // sensor minimum reading
   0x00,   // UNRT
   0xC8,   // UCT
   0x00,   // UNCT
   0x00,   // LNRT
   0x00,   // LCT
   0x00,   // LNCT
   0x00,   // positive-going threshold
   0x00,   // negative-going threshold
   0x00,   // reserved
   0x00,   // reserved
   0x00,   // OEM
   IPMI_SDR_STRING_TYPE_ASCII_8,   // ID len, should be same as "size of struct"
   "SINGLE_FAN_BMC_TACH_1",
 },
 {   // SINGLE_FAN_BMC_TACH_2
   0x00,0x00,  // record ID
   IPMI_SDR_VER_15,  // SDR ver
   IPMI_SDR_FULL_SENSOR,   // record type
   IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

   Self_I2C_ADDRESS<<1,   // owner id
   0x00,   // owner lun
   SENSOR_NUM_SINGLE_FAN_BMC_TACH_2,    // sensor number

   IPMI_SDR_ENTITY_ID_SYS_BOARD,   // entity id
   0x00,   // entity instance
   IPMI_SDR_SENSOR_INIT_SCAN| IPMI_SDR_SENSOR_INIT_EVENT| IPMI_SDR_SENSOR_INIT_THRESHOLD| IPMI_SDR_SENSOR_INIT_TYPE| IPMI_SDR_SENSOR_INIT_DEF_EVENT| IPMI_SDR_SENSOR_INIT_DEF_SCAN,   // sensor init
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
   0x17,   // [7:0] M bits
   0x00,   // [9:8] M bits, tolerance
   0x00,   // [7:0] B bits
   0x00,   // [9:8] B bits, tolerance
   0x00,   // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
   0xE0,   // Rexp, Bexp
   0x00,   // analog characteristic
   0x00,   // nominal reading
   0x00,   // normal maximum
   0x00,   // normal minimum
   0x00,   // sensor maximum reading
   0x00,   // sensor minimum reading
   0x00,   // UNRT
   0xC8,   // UCT
   0x00,   // UNCT
   0x00,   // LNRT
   0x00,   // LCT
   0x00,   // LNCT
   0x00,   // positive-going threshold
   0x00,   // negative-going threshold
   0x00,   // reserved
   0x00,   // reserved
   0x00,   // OEM
   IPMI_SDR_STRING_TYPE_ASCII_8,   // ID len, should be same as "size of struct"
   "SINGLE_FAN_BMC_TACH_2",
 },
 {   // SINGLE_FAN_BMC_TACH_3
   0x00,0x00,  // record ID
   IPMI_SDR_VER_15,  // SDR ver
   IPMI_SDR_FULL_SENSOR,   // record type
   IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

   Self_I2C_ADDRESS<<1,   // owner id
   0x00,   // owner lun
   SENSOR_NUM_SINGLE_FAN_BMC_TACH_3,    // sensor number

   IPMI_SDR_ENTITY_ID_SYS_BOARD,   // entity id
   0x00,   // entity instance
   IPMI_SDR_SENSOR_INIT_SCAN| IPMI_SDR_SENSOR_INIT_EVENT| IPMI_SDR_SENSOR_INIT_THRESHOLD| IPMI_SDR_SENSOR_INIT_TYPE| IPMI_SDR_SENSOR_INIT_DEF_EVENT| IPMI_SDR_SENSOR_INIT_DEF_SCAN,   // sensor init
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
   0x17,   // [7:0] M bits
   0x00,   // [9:8] M bits, tolerance
   0x00,   // [7:0] B bits
   0x00,   // [9:8] B bits, tolerance
   0x00,   // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
   0xE0,   // Rexp, Bexp
   0x00,   // analog characteristic
   0x00,   // nominal reading
   0x00,   // normal maximum
   0x00,   // normal minimum
   0x00,   // sensor maximum reading
   0x00,   // sensor minimum reading
   0x00,   // UNRT
   0xC8,   // UCT
   0x00,   // UNCT
   0x00,   // LNRT
   0x00,   // LCT
   0x00,   // LNCT
   0x00,   // positive-going threshold
   0x00,   // negative-going threshold
   0x00,   // reserved
   0x00,   // reserved
   0x00,   // OEM
   IPMI_SDR_STRING_TYPE_ASCII_8,   // ID len, should be same as "size of struct"
   "SINGLE_FAN_BMC_TACH_3",
 },
};


uint8_t pal_load_sdr_table(void) {
  memcpy(&full_sensor_table, &plat_sensor_table, sizeof(plat_sensor_table));
  return (sizeof(plat_sensor_table) / sizeof(plat_sensor_table[0]));
};
