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
  {   // TMP75 on board temperature
    0x00,0x00,  // record ID
    IPMI_SDR_VER_15,  // SDR ver
    IPMI_SDR_FULL_SENSOR,   // record type
    IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

    Self_I2C_ADDRESS<<1,   // owner id
    0x00,   // owner lun
    SENSOR_NUM_TEMP_TMP75_FIO,    // sensor number

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
    "Front IO Temp",
  },
  {   // CPU margin on board temperature
    0x00,0x00,  // record ID
    IPMI_SDR_VER_15,  // SDR ver
    IPMI_SDR_FULL_SENSOR,   // record type
    IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

    Self_I2C_ADDRESS<<1,   // owner id
    0x00,   // owner lun
    SENSOR_NUM_TEMP_CPU_MARGIN,    // sensor number
    
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
    0x80,   // sensor unit
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
    IPMI_SDR_STRING_TYPE_ASCII_8,   // ID len, should be same as "size of struct"
    "SOC Therm Margin",
  },
  {   // CPU TEMP on board temperature
    0x00,0x00,  // record ID
    IPMI_SDR_VER_15,  // SDR ver
    IPMI_SDR_FULL_SENSOR,   // record type
    IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

    Self_I2C_ADDRESS<<1,   // owner id
    0x00,   // owner lun
    SENSOR_NUM_TEMP_CPU,    // sensor number
    
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
    0x4A,   // UCT
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
    "CPU Temp",
  },
  {   // CPU TJMAX on board temperature
    0x00,0x00,  // record ID
    IPMI_SDR_VER_15,  // SDR ver
    IPMI_SDR_FULL_SENSOR,   // record type
    IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

    Self_I2C_ADDRESS<<1,   // owner id
    0x00,   // owner lun
    SENSOR_NUM_TEMP_CPU_TJMAX,    // sensor number
    
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
    "CPU TjMax",
  },
  {   // DIMM A on board temperature
    0x00,0x00,  // record ID
    IPMI_SDR_VER_15,  // SDR ver
    IPMI_SDR_FULL_SENSOR,   // record type
    IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

    Self_I2C_ADDRESS<<1,   // owner id
    0x00,   // owner lun
    SENSOR_NUM_TEMP_DIMM_A,    // sensor number
    
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
    "DIMMA Temp",
  },
  {   // DIMM C on board temperature
    0x00,0x00,  // record ID
    IPMI_SDR_VER_15,  // SDR ver
    IPMI_SDR_FULL_SENSOR,   // record type
    IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

    Self_I2C_ADDRESS<<1,   // owner id
    0x00,   // owner lun
    SENSOR_NUM_TEMP_DIMM_C,    // sensor number
    
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
    "DIMMC Temp",
  },
  {   // DIMM D on board temperature
    0x00,0x00,  // record ID
    IPMI_SDR_VER_15,  // SDR ver
    IPMI_SDR_FULL_SENSOR,   // record type
    IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

    Self_I2C_ADDRESS<<1,   // owner id
    0x00,   // owner lun
    SENSOR_NUM_TEMP_DIMM_D,    // sensor number
    
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
    "DIMMD Temp",
  },
  {   // DIMM E on board temperature
    0x00,0x00,  // record ID
    IPMI_SDR_VER_15,  // SDR ver
    IPMI_SDR_FULL_SENSOR,   // record type
    IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

    Self_I2C_ADDRESS<<1,   // owner id
    0x00,   // owner lun
    SENSOR_NUM_TEMP_DIMM_E,    // sensor number
    
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
    "DIMME Temp",
  },
  {   // DIMM G on board temperature
    0x00,0x00,  // record ID
    IPMI_SDR_VER_15,  // SDR ver
    IPMI_SDR_FULL_SENSOR,   // record type
    IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

    Self_I2C_ADDRESS<<1,   // owner id
    0x00,   // owner lun
    SENSOR_NUM_TEMP_DIMM_G,    // sensor number
    
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
    "DIMMG Temp",
  },
  {   // DIMM H on board temperature
    0x00,0x00,  // record ID
    IPMI_SDR_VER_15,  // SDR ver
    IPMI_SDR_FULL_SENSOR,   // record type
    IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

    Self_I2C_ADDRESS<<1,   // owner id
    0x00,   // owner lun
    SENSOR_NUM_TEMP_DIMM_H,    // sensor number
    
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
    "DIMMH Temp",
  },
  {   // SSD0 temperature
    0x00,0x00,  // record ID
    IPMI_SDR_VER_15,  // SDR ver
    IPMI_SDR_FULL_SENSOR,   // record type
    IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

    Self_I2C_ADDRESS<<1,   // owner id
    0x00,   // owner lun
    SENSOR_NUM_TEMP_SSD0,    // sensor number

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
    IPMI_SENSOR_UNIT_DEGREE_C,   // base unit
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
    0x4B,   // UCT
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
    "SSD0 Temp",
  },
  {   // PCH temperature
    0x00,0x00,  // record ID
    IPMI_SDR_VER_15,  // SDR ver
    IPMI_SDR_FULL_SENSOR,   // record type
    IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

    Self_I2C_ADDRESS<<1,   // owner id
    0x00,   // owner lun
    SENSOR_NUM_TEMP_PCH,    // sensor number

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
    0x4D,   // UCT
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
    "PCH Temp", 
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
  {   // PVCCIN VR temperature
    0x00,0x00,  // record ID
    IPMI_SDR_VER_15,  // SDR ver
    IPMI_SDR_FULL_SENSOR,   // record type
    IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

    Self_I2C_ADDRESS<<1,   // owner id
    0x00,   // owner lun
    SENSOR_NUM_TEMP_PVCCIN,    // sensor number

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
    0x64,   // UCT
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
    "PVCCIN Temp",
  },
  {   // PVCCFA_EHV_FIVRA VR temperature
    0x00,0x00,  // record ID
    IPMI_SDR_VER_15,  // SDR ver
    IPMI_SDR_FULL_SENSOR,   // record type
    IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

    Self_I2C_ADDRESS<<1,   // owner id
    0x00,   // owner lun
    SENSOR_NUM_TEMP_PVCCFA_EHV_FIVRA,    // sensor number

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
    0x64,   // UCT
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
    "EHV_FIVRA Temp",
  },
  {   // PVCCFA_EHV VR temperature
    0x00,0x00,  // record ID
    IPMI_SDR_VER_15,  // SDR ver
    IPMI_SDR_FULL_SENSOR,   // record type
    IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

    Self_I2C_ADDRESS<<1,   // owner id
    0x00,   // owner lun
    SENSOR_NUM_TEMP_PVCCFA_EHV,    // sensor number

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
    0x64,   // UCT
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
    "PVCCFA_EHV Temp",
  },
  {   // PVCCD_HV VR temperature
    0x00,0x00,  // record ID
    IPMI_SDR_VER_15,  // SDR ver
    IPMI_SDR_FULL_SENSOR,   // record type
    IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

    Self_I2C_ADDRESS<<1,   // owner id
    0x00,   // owner lun
    SENSOR_NUM_TEMP_PVCCD_HV,    // sensor number

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
    0x64,   // UCT
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
    "PVCCD_HV Temp",
  },
  {   // PVCCINFAON VR temperature
    0x00,0x00,  // record ID
    IPMI_SDR_VER_15,  // SDR ver
    IPMI_SDR_FULL_SENSOR,   // record type
    IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

    Self_I2C_ADDRESS<<1,   // owner id
    0x00,   // owner lun
    SENSOR_NUM_TEMP_PVCCINFAON,    // sensor number

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
    0x64,   // UCT
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
    "PVCCINFAON Temp",
  },
  {   // P12V STBY ADC voltage 
    0x00,0x00,  // record ID
    IPMI_SDR_VER_15,  // SDR ver
    IPMI_SDR_FULL_SENSOR,   // record type
    IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

    Self_I2C_ADDRESS<<1,   // owner id
    0x00,   // owner lun
    SENSOR_NUM_VOL_STBY12V,    // sensor number
    
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
    "P12V_STBY Vol",
  },
  {   // BATTERY 3V ADC voltage 
    0x00,0x00,  // record ID
    IPMI_SDR_VER_15,  // SDR ver
    IPMI_SDR_FULL_SENSOR,   // record type
    IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

    Self_I2C_ADDRESS<<1,   // owner id
    0x00,   // owner lun
    SENSOR_NUM_VOL_BAT3V,    // sensor number
    
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
    "P3V_BAT Vol",
  },
  {   // P3V3 STBY ADC voltage 
    0x00,0x00,  // record ID
    IPMI_SDR_VER_15,  // SDR ver
    IPMI_SDR_FULL_SENSOR,   // record type
    IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

    Self_I2C_ADDRESS<<1,   // owner id
    0x00,   // owner lun
    SENSOR_NUM_VOL_STBY3V,    // sensor number
    
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
  {   // P1V05 STBY ADC voltage 
    0x00,0x00,  // record ID
    IPMI_SDR_VER_15,  // SDR ver
    IPMI_SDR_FULL_SENSOR,   // record type
    IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

    Self_I2C_ADDRESS<<1,   // owner id
    0x00,   // owner lun
    SENSOR_NUM_VOL_STBY1V05,    // sensor number
    
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
    "P1V05_PCH Vol",
  },
  {   // P1V8 STBY ADC voltage 
    0x00,0x00,  // record ID
    IPMI_SDR_VER_15,  // SDR ver
    IPMI_SDR_FULL_SENSOR,   // record type
    IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

    Self_I2C_ADDRESS<<1,   // owner id
    0x00,   // owner lun
    SENSOR_NUM_VOL_STBY1V8,    // sensor number
    
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
    "P1V8_STBY Vol",
  },
  {   // PVCCIN VR voltage
    0x00,0x00,  // record ID
    IPMI_SDR_VER_15,  // SDR ver
    IPMI_SDR_FULL_SENSOR,   // record type
    IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

    Self_I2C_ADDRESS<<1,   // owner id
    0x00,   // owner lun
    SENSOR_NUM_VOL_PVCCIN,    // sensor number

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
    "PVCCIN Vol",
  },
  {   // PVCCFA_EHV_FIVRA VR voltage
    0x00,0x00,  // record ID
    IPMI_SDR_VER_15,  // SDR ver
    IPMI_SDR_FULL_SENSOR,   // record type
    IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

    Self_I2C_ADDRESS<<1,   // owner id
    0x00,   // owner lun
    SENSOR_NUM_VOL_PVCCFA_EHV_FIVRA,    // sensor number

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
    "EHV_FIVRA Vol",
  },
  {   // PVCCFA_EHV VR voltage
    0x00,0x00,  // record ID
    IPMI_SDR_VER_15,  // SDR ver
    IPMI_SDR_FULL_SENSOR,   // record type
    IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

    Self_I2C_ADDRESS<<1,   // owner id
    0x00,   // owner lun
    SENSOR_NUM_VOL_PVCCFA_EHV,    // sensor number

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
    "PVCCFA_EHV Vol",
  },
  {   // PVCCD_HV VR voltage
   0x00,0x00,  // record ID
   IPMI_SDR_VER_15,  // SDR ver
   IPMI_SDR_FULL_SENSOR,   // record type
   IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

   Self_I2C_ADDRESS<<1,   // owner id
   0x00,   // owner lun
   SENSOR_NUM_VOL_PVCCD_HV,    // sensor number

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
   "PVCCD_HV Vol",
 },
 {   // PVCCINFAON VR voltage
   0x00,0x00,  // record ID
   IPMI_SDR_VER_15,  // SDR ver
   IPMI_SDR_FULL_SENSOR,   // record type
   IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

   Self_I2C_ADDRESS<<1,   // owner id
   0x00,   // owner lun
   SENSOR_NUM_VOL_PVCCINFAON,    // sensor number

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
   0x36,   // [7:0] M bits
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
   "PVCCINFAON Vol",
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
    0x33,   // [7:0] M bits
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
    "HSC Input Vol",
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
   "HSC Output Cur",
  },
  {   // PVCCIN VR current
   0x00,0x00,  // record ID
   IPMI_SDR_VER_15,  // SDR ver
   IPMI_SDR_FULL_SENSOR,   // record type
   IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

   Self_I2C_ADDRESS<<1,   // owner id
   0x00,   // owner lun
   SENSOR_NUM_CUR_PVCCIN,    // sensor number

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
   "PVCCIN Cur",
 },
 {   // PVCCFA_EHV_FIVRA VR current
   0x00,0x00,  // record ID
   IPMI_SDR_VER_15,  // SDR ver
   IPMI_SDR_FULL_SENSOR,   // record type
   IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

   Self_I2C_ADDRESS<<1,   // owner id
   0x00,   // owner lun
   SENSOR_NUM_CUR_PVCCFA_EHV_FIVRA,    // sensor number

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
   "EHV_FIVRA Cur",
 },
 {   // PVCCFA_EHV VR current
   0x00,0x00,  // record ID
   IPMI_SDR_VER_15,  // SDR ver
   IPMI_SDR_FULL_SENSOR,   // record type
   IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

   Self_I2C_ADDRESS<<1,   // owner id
   0x00,   // owner lun
   SENSOR_NUM_CUR_PVCCFA_EHV,    // sensor number

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
   0x1F,   // [7:0] M bits
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
   "PVCCFA_EHV Cur",
 },
 {   // PVCCD_HV VR current
   0x00,0x00,  // record ID
   IPMI_SDR_VER_15,  // SDR ver
   IPMI_SDR_FULL_SENSOR,   // record type
   IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

   Self_I2C_ADDRESS<<1,   // owner id
   0x00,   // owner lun
   SENSOR_NUM_CUR_PVCCD_HV,    // sensor number

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
   0x0F,   // [7:0] M bits
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
   "PVCCD_HV Cur",
 },
 {   // PVCCINFAON VR current
   0x00,0x00,  // record ID
   IPMI_SDR_VER_15,  // SDR ver
   IPMI_SDR_FULL_SENSOR,   // record type
   IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

   Self_I2C_ADDRESS<<1,   // owner id
   0x00,   // owner lun
   SENSOR_NUM_CUR_PVCCINFAON,    // sensor number

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
   "PVCCINFAON Cur",
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
   "HSC Input Pwr",
  },
  {   // PVCCIN power
   0x00,0x00,  // record ID
   IPMI_SDR_VER_15,  // SDR ver
   IPMI_SDR_FULL_SENSOR,   // record type
   IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

   Self_I2C_ADDRESS<<1,   // owner id
   0x00,   // owner lun
   SENSOR_NUM_PWR_PVCCIN,    // sensor number

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
   "PVCCIN Pwr",
 },
 {   // PVCCFA_EHV_FIVRA power
   0x00,0x00,  // record ID
   IPMI_SDR_VER_15,  // SDR ver
   IPMI_SDR_FULL_SENSOR,   // record type
   IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

   Self_I2C_ADDRESS<<1,   // owner id
   0x00,   // owner lun
   SENSOR_NUM_PWR_PVCCFA_EHV_FIVRA,    // sensor number

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
   0x2F,   // [7:0] M bits
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
   "EHV_FIVRA Pwr",
 },
 {   // PVCCFA_EHV power
   0x00,0x00,  // record ID
   IPMI_SDR_VER_15,  // SDR ver
   IPMI_SDR_FULL_SENSOR,   // record type
   IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

   Self_I2C_ADDRESS<<1,   // owner id
   0x00,   // owner lun
   SENSOR_NUM_PWR_PVCCFA_EHV,    // sensor number

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
   0x38,   // [7:0] M bits
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
   "PVCCFA_EHV Pwr",
 },
 {   // PVCCD_HV power
   0x00,0x00,  // record ID
   IPMI_SDR_VER_15,  // SDR ver
   IPMI_SDR_FULL_SENSOR,   // record type
   IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

   Self_I2C_ADDRESS<<1,   // owner id
   0x00,   // owner lun
   SENSOR_NUM_PWR_PVCCD_HV,    // sensor number

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
   0xA5,   // [7:0] M bits
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
   "PVCCD_HV Pwr",
 },
 {   // PVCCINFAON power
   0x00,0x00,  // record ID
   IPMI_SDR_VER_15,  // SDR ver
   IPMI_SDR_FULL_SENSOR,   // record type
   IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

   Self_I2C_ADDRESS<<1,   // owner id
   0x00,   // owner lun
   SENSOR_NUM_PWR_PVCCINFAON,    // sensor number

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
   "PVCCINFAON Pwr",
 },
};


uint8_t pal_load_sdr_table(void) {
  memcpy(&full_sensor_table, &plat_sensor_table, sizeof(plat_sensor_table));
  return (sizeof(plat_sensor_table) / sizeof(plat_sensor_table[0]));
};
