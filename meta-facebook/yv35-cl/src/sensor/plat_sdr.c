#include <stdio.h>
#include <string.h>
#include "sdr.h"
#include "pal.h"
#include "sensor.h"
#include "sensor_def.h"
#include "plat_ipmb.h"

enum{
  threshold_UNR ,
  threshold_UCR ,
  threshold_UNC ,
  threshold_LNR ,
  threshold_LCR ,
  threshold_LNC ,
  MBR_M ,
  MBR_B ,
  MBR_R ,
};

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
    0x96,   // UNRT
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
    0x96,   // UNRT
    0x5D,   // UCT
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
    0x96,   // UNRT
    0x28,   // UCT
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
    0x54,   // UCT
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
    0x69,   // UNRT
    0x57,   // UCT
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
    0x7D,   // UNRT
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
    0x7D,   // UNRT
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
    0x7D,   // UNRT
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
    0x7D,   // UNRT
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
    0x7D,   // UNRT
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
    IPMI_SENSOR_UNIT_VOL,   // base unit 
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
    0xE0,   // UNRT
    0xCF,   // UCT
    0xCB,   // UNCT
    0x9E,   // LNRT
    0xA9,   // LCT
    0xAD,   // LNCT
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
    IPMI_SENSOR_UNIT_VOL,   // base unit 
    0x00,   // modifier unit
    IPMI_SDR_LINEAR_LINEAR,   // linearization
    0x12,   // [7:0] M bits
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
    0xC4,   // UCT
    0xC1,   // UNCT
    0x00,   // LNRT
    0xAB,   // LCT
    0xAE,   // LNCT
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
    IPMI_SENSOR_UNIT_VOL,   // base unit 
    0x00,   // modifier unit
    IPMI_SDR_LINEAR_LINEAR,   // linearization
    0x12,   // [7:0] M bits
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
    0xDE,   // UNRT
    0xC4,   // UCT
    0xC1,   // UNCT
    0x80,   // LNRT
    0xAB,   // LCT
    0xAE,   // LNCT
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
    IPMI_SENSOR_UNIT_VOL,   // base unit 
    0x00,   // modifier unit
    IPMI_SDR_LINEAR_LINEAR,   // linearization
    0x05,   // [7:0] M bits
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
    0xF4,   // UNRT
    0xDD,   // UCT
    0xD8,   // UNCT
    0xA8,   // LNRT
    0xC4,   // LCT
    0xC8,   // LNCT
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
    IPMI_SENSOR_UNIT_VOL,   // base unit 
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
    0xD1,   // UNRT
    0xBE,   // UCT
    0xBA,   // UNCT
    0x90,   // LNRT
    0xA9,   // LCT
    0xAD,   // LNCT
    0x00,   // positive-going threshold
    0x00,   // negative-going threshold
    0x00,   // reserved
    0x00,   // reserved
    0x00,   // OEM
    IPMI_SDR_STRING_TYPE_ASCII_8,   // ID len, should be same as "size of struct"
    "P1V8_STBY Vol",
  },
  {   // P5V STBY ADC voltage
    0x00,0x00,  // record ID
    IPMI_SDR_VER_15,  // SDR ver
    IPMI_SDR_FULL_SENSOR,   // record type
    IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

    Self_I2C_ADDRESS<<1,   // owner id
    0x00,   // owner lun
    SENSOR_NUM_VOL_STBY5V,    // sensor number

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
    IPMI_SENSOR_UNIT_VOL,   // base unit
    0x00,   // modifier unit
    IPMI_SDR_LINEAR_LINEAR,   // linearization
    0x1B,   // [7:0] M bits
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
    0xD7,   // UNRT
    0xC6,   // UCT
    0xC2,   // UNCT
    0x94,   // LNRT
    0xAC,   // LCT
    0xB0,   // LNCT
    0x00,   // positive-going threshold
    0x00,   // negative-going threshold
    0x00,   // reserved
    0x00,   // reserved
    0x00,   // OEM
    IPMI_SDR_STRING_TYPE_ASCII_8,   // ID len, should be same as "size of struct"
    "P5V_STBY Vol",
  },
  {   // P12V DIMM ADC voltage
    0x00,0x00,  // record ID
    IPMI_SDR_VER_15,  // SDR ver
    IPMI_SDR_FULL_SENSOR,   // record type
    IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

    Self_I2C_ADDRESS<<1,   // owner id
    0x00,   // owner lun
    SENSOR_NUM_VOL_DIMM12V,    // sensor number

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
    IPMI_SENSOR_UNIT_VOL,   // base unit
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
    0xDC,   // UCT
    0xD8,   // UNCT
    0x00,   // LNRT
    0x9C,   // LCT
    0x9F,   // LNCT
    0x00,   // positive-going threshold
    0x00,   // negative-going threshold
    0x00,   // reserved
    0x00,   // reserved
    0x00,   // OEM
    IPMI_SDR_STRING_TYPE_ASCII_8,   // ID len, should be same as "size of struct"
    "P12V_DIMM Vol",
  },
  {   // P1V2 STBY ADC voltage
    0x00,0x00,  // record ID
    IPMI_SDR_VER_15,  // SDR ver
    IPMI_SDR_FULL_SENSOR,   // record type
    IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

    Self_I2C_ADDRESS<<1,   // owner id
    0x00,   // owner lun
    SENSOR_NUM_VOL_STBY1V2,    // sensor number

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
    IPMI_SENSOR_UNIT_VOL,   // base unit
    0x00,   // modifier unit
    IPMI_SDR_LINEAR_LINEAR,   // linearization
    0x06,   // [7:0] M bits
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
    0xE8,   // UNRT
    0xD6,   // UCT
    0xD2,   // UNCT
    0xA0,   // LNRT
    0xBA,   // LCT
    0xBE,   // LNCT
    0x00,   // positive-going threshold
    0x00,   // negative-going threshold
    0x00,   // reserved
    0x00,   // reserved
    0x00,   // OEM
    IPMI_SDR_STRING_TYPE_ASCII_8,   // ID len, should be same as "size of struct"
    "P1V2_STBY Vol",
  },
  {   // P3V3 M2 ADC voltage
    0x00,0x00,  // record ID
    IPMI_SDR_VER_15,  // SDR ver
    IPMI_SDR_FULL_SENSOR,   // record type
    IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

    Self_I2C_ADDRESS<<1,   // owner id
    0x00,   // owner lun
    SENSOR_NUM_VOL_M2_3V3,    // sensor number

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
    IPMI_SENSOR_UNIT_VOL,   // base unit
    0x00,   // modifier unit
    IPMI_SDR_LINEAR_LINEAR,   // linearization
    0x12,   // [7:0] M bits
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
    0xCC,   // UCT
    0xC8,   // UNCT
    0x00,   // LNRT
    0xA4,   // LCT
    0xA7,   // LNCT
    0x00,   // positive-going threshold
    0x00,   // negative-going threshold
    0x00,   // reserved
    0x00,   // reserved
    0x00,   // OEM
    IPMI_SDR_STRING_TYPE_ASCII_8,   // ID len, should be same as "size of struct"
    "P3V3_M2 Vol",
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
    IPMI_SENSOR_UNIT_VOL,   // base unit
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
    0xDC,   // UNRT
    0xB7,   // UCT
    0xBB,   // UNCT
    0x28,   // LNRT
    0xAC,   // LCT
    0xA9,   // LNCT
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
    IPMI_SENSOR_UNIT_VOL,   // base unit
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
    0xDC,   // UNRT
    0xB6,   // UCT
    0xBA,   // UNCT
    0x28,   // LNRT
    0xB1,   // LCT
    0xAE,   // LNCT
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
    IPMI_SENSOR_UNIT_VOL,   // base unit
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
    0xDC,   // UNRT
    0xBC,   // UCT
    0xB8,   // UNCT
    0x28,   // LNRT
    0xAB,   // LCT
    0xAF,   // LNCT
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
   IPMI_SENSOR_UNIT_VOL,   // base unit
   0x00,   // modifier unit
   IPMI_SDR_LINEAR_LINEAR,   // linearization
   0x07,   // [7:0] M bits
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
   0xD6,   // UNRT
   0xAD,   // UCT
   0xAA,   // UNCT
   0x39,   // LNRT
   0x97,   // LCT
   0x9A,   // LNCT
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
   IPMI_SENSOR_UNIT_VOL,   // base unit
   0x00,   // modifier unit
   IPMI_SDR_LINEAR_LINEAR,   // linearization
   0x07,   // [7:0] M bits
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
   0xC8,   // UNRT
   0x99,   // UCT
   0x96,   // UNCT
   0x39,   // LNRT
   0x82,   // LCT
   0x85,   // LNCT
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
    IPMI_SENSOR_UNIT_VOL,   // base unit
    0x00,   // modifier unit
    IPMI_SDR_LINEAR_LINEAR,   // linearization
    0x44,   // [7:0] M bits
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
    0xD3,   // UNRT
    0xC2,   // UCT
    0xBF,   // UNCT
    0x94,   // LNRT
    0x9F,   // LCT
    0xA2,   // LNCT
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
   IPMI_SENSOR_UNIT_AMP,   // base unit
   0x00,   // modifier unit
   IPMI_SDR_LINEAR_LINEAR,   // linearization
   0x13,   // [7:0] M bits
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
   0xD3,   // UCT
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
   IPMI_SENSOR_UNIT_AMP,   // base unit
   0x00,   // modifier unit
   IPMI_SDR_LINEAR_LINEAR,   // linearization
   0x4D,   // [7:0] M bits
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
   0xDD,   // UCT
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
   IPMI_SENSOR_UNIT_AMP,   // base unit
   0x00,   // modifier unit
   IPMI_SDR_LINEAR_LINEAR,   // linearization
   0x27,   // [7:0] M bits
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
   0xDA,   // UCT
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
   IPMI_SENSOR_UNIT_AMP,   // base unit
   0x00,   // modifier unit
   IPMI_SDR_LINEAR_LINEAR,   // linearization
   0x08,   // [7:0] M bits
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
   0xE1,   // UCT
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
   IPMI_SENSOR_UNIT_AMP,   // base unit
   0x00,   // modifier unit
   IPMI_SDR_LINEAR_LINEAR,   // linearization
   0x22,   // [7:0] M bits
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
   0xDA,   // UCT
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
   IPMI_SENSOR_UNIT_AMP,   // base unit
   0x00,   // modifier unit
   IPMI_SDR_LINEAR_LINEAR,   // linearization
   0x18,   // [7:0] M bits
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
   0xD9,   // UCT
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
  {   // CPU power
   0x00,0x00,  // record ID
   IPMI_SDR_VER_15,  // SDR ver
   IPMI_SDR_FULL_SENSOR,   // record type
   IPMI_SDR_FULL_SENSOR_MIN_LEN,   // size of struct

   Self_I2C_ADDRESS<<1,   // owner id
   0x00,   // owner lun
   SENSOR_NUM_PWR_CPU,    // sensor number

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
   IPMI_SENSOR_UNIT_WATT,   // base unit
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
   "CPU Pwr",
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
   IPMI_SENSOR_UNIT_WATT,   // base unit
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
   0xB2,   // UCT
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
   IPMI_SENSOR_UNIT_WATT,   // base unit
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
   IPMI_SENSOR_UNIT_WATT,   // base unit
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
   IPMI_SENSOR_UNIT_WATT,   // base unit
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
   IPMI_SENSOR_UNIT_WATT,   // base unit
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
   IPMI_SENSOR_UNIT_WATT,   // base unit
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
   "PVCCINFAON Pwr",
 },
};

uint8_t fix_C2SDR_table[][10]= {
// sensor_num , UNR , UCR , UNC , LNR , LCR , LNC , ( M_tolerance >> 6 ) || M , ( B_accuracy >> 6 ) || B , R
};

SDR_Full_sensor fix_1ouSDR_table[] = {
// SDR_Full_sensor struct member
};
SDR_Full_sensor fix_DVPSDR_table[] = {
// SDR_Full_sensor struct member
};

uint8_t pal_load_sdr_table(void) {
  memcpy(&full_sensor_table, &plat_sensor_table, sizeof(plat_sensor_table));
  return (sizeof(plat_sensor_table) / sizeof(plat_sensor_table[0]));
};

uint8_t map_SnrNum_fullSDR( uint8_t sensor_num ) {
  uint8_t i , j;
  for ( i = 0 ; i < SENSOR_NUM_MAX ; i++ ) {
    for ( j = 0 ; j < SDR_NUM ; ++j ) {
      if ( sensor_num == full_sensor_table[j].sensor_num ) {
        return j;
      } else if ( i == SDR_NUM ) {
        return 0xFF;
      }
    }
  }
};

void change_sensor_threshold( uint8_t sensor_num , uint8_t threshold_type , uint8_t change_value ) {
  uint8_t SnrNum_SDR_map = map_SnrNum_fullSDR( sensor_num );
  if( SnrNum_SDR_map == 0xFF ) {
    printk( "Not found change threshold sensor\n" );
    return;
  }
  if ( threshold_type == threshold_UNR ) {
    full_sensor_table[ SnrNum_SDR_map ].UNRT = change_value;
  } else if ( threshold_type == threshold_UCR ) {
    full_sensor_table[ SnrNum_SDR_map ].UCT = change_value;
  } else if ( threshold_type == threshold_UNC ) {
    full_sensor_table[ SnrNum_SDR_map ].UNCT = change_value;
  } else if ( threshold_type == threshold_LNR ) {
    full_sensor_table[ SnrNum_SDR_map ].LNRT = change_value;
  } else if ( threshold_type == threshold_LCR ) {
    full_sensor_table[ SnrNum_SDR_map ].LCT = change_value;
  } else if ( threshold_type == threshold_LNC ) {
    full_sensor_table[ SnrNum_SDR_map ].LNCT = change_value;
  } else {
    printk( "Not found want changing threshold\n" );
    return;
  }
};

void change_sensor_MBR( uint8_t sensor_num , uint8_t MBR_type , uint16_t change_value ) {
  uint8_t SnrNum_SDR_map = map_SnrNum_fullSDR( sensor_num );
  if( SnrNum_SDR_map == 0xFF ) {
    printk( "Not found change threshold sensor\n" );
    return;
  }
  if ( MBR_type == MBR_M ) {
    full_sensor_table[ SnrNum_SDR_map ].M = change_value & 0xFF;
    if ( change_value >> 8 ) {
      full_sensor_table[ SnrNum_SDR_map ].M_tolerance = ( ( change_value >> 8 ) << 6 ) & 0xFF;
    } else {
      full_sensor_table[ SnrNum_SDR_map ].M_tolerance = 0;
    }
  } else if( MBR_type == MBR_B ) {
    full_sensor_table[ SnrNum_SDR_map ].B = change_value;
    if ( change_value >> 8 ) {
      full_sensor_table[ SnrNum_SDR_map ].B_accuracy = ( ( change_value >> 8 ) << 6 ) & 0xFF;
    } else {
      full_sensor_table[ SnrNum_SDR_map ].B_accuracy = 0;
    }
  } else if( MBR_type == MBR_R ) {
    full_sensor_table[ SnrNum_SDR_map ].RexpBexp = change_value & 0xFF;
  } else {
    printk( "Not found want changing MBR type\n" );
    return;
  }
};

void add_fullSDR_table( SDR_Full_sensor add_item ) {
  if ( map_SnrNum_fullSDR( add_item.sensor_num ) != 0xFF ) {
    printk( "add sensor num is already exists\n" );
    return;
  }
  full_sensor_table[ SDR_NUM++ ] = add_item;
};

void fix_fullSDR_table() {
  uint8_t fix_array_num;
  if ( get_bic_class() ) {
    // fix usage when fix_C2SDR_table is defined
    fix_array_num = sizeof( fix_C2SDR_table ) / sizeof( fix_C2SDR_table[0] );
    while( fix_array_num ) {
      for ( int i = MBR_R ; i >= threshold_UNR ; --i ) {
        if ( i < MBR_M ) {
          change_sensor_threshold( fix_C2SDR_table[ fix_array_num - 1 ][0] , i , fix_C2SDR_table[ fix_array_num - 1 ][ i + 1 ] );
        } else {
          change_sensor_MBR( fix_C2SDR_table[ fix_array_num - 1 ][0] , i , fix_C2SDR_table[ fix_array_num - 1 ][ i + 1 ] );
        }
      }
      fix_array_num--;
    }
  }
  if ( get_1ou_status() ) {
    // fix usage when fix_1ouSDR_table is defined
    fix_array_num = sizeof( fix_1ouSDR_table ) / sizeof( fix_1ouSDR_table[0] );
    while( fix_array_num ) {
      add_fullSDR_table( fix_1ouSDR_table[ fix_array_num - 1 ] );
      fix_array_num--;
    }
  }
  if ( get_2ou_status() ) {
    // fix usage when fix_DVPSDR_table is defined
    fix_array_num = sizeof( fix_DVPSDR_table ) / sizeof( fix_DVPSDR_table[0] );
    while( fix_array_num ) {
      add_fullSDR_table( fix_DVPSDR_table[ fix_array_num - 1 ] );
      fix_array_num--;
    }
  }
};
