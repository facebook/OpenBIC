#include "plat_sdr_table.h"

#include <stdio.h>
#include <string.h>

#include "sdr.h"
#include "plat_class.h"
#include "plat_ipmb.h"
#include "plat_sensor_table.h"

SDR_Full_sensor plat_sdr_table[] = {
	{
		/* TEMPARUTURE SENSOR */
		// TMP75 on board temperature - inlet
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_TEMP_TMP75_IN, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_TEMPERATURE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		0x00, // assert event mask
		IPMI_SDR_CMP_RETURN_LCT | IPMI_SDR_ASSERT_MASK_UCT_HI |
			IPMI_SDR_ASSERT_MASK_LCT_LO, // assert threshold reading mask
		0x00, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT | IPMI_SDR_DEASSERT_MASK_UCT_LO |
			IPMI_SDR_DEASSERT_MASK_LCT_HI, // deassert threshold reading mask
		0x00, // discrete reading mask/ settable
		IPMI_SDR_UCT_SETTABLE | IPMI_SDR_LCT_SETTABLE | IPMI_SDR_UCT_READABLE |
			IPMI_SDR_LCT_READABLE, // threshold mask/ readable threshold mask
		0x80, // sensor unit
		IPMI_SENSOR_UNIT_DEGREE_C, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		0x01, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		0x00, // Rexp, Bexp
		0x00, // analog characteristic
		0x00, // nominal reading
		0x00, // normal maximum
		0x00, // normal minimum
		0x00, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		0x2D, // UCT
		0x00, // UNCT
		0x00, // LNRT
		0x0A, // LCT
		0x00, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"MB Inlet Temp",
	},
	{
		// TMP75 on board temperature - outlet
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_TEMP_TMP75_OUT, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_TEMPERATURE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		0x00, // assert event mask
		IPMI_SDR_CMP_RETURN_LCT | IPMI_SDR_ASSERT_MASK_UCT_HI |
			IPMI_SDR_ASSERT_MASK_LCT_LO, // assert threshold reading mask
		0x00, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT | IPMI_SDR_DEASSERT_MASK_UCT_LO |
			IPMI_SDR_DEASSERT_MASK_LCT_HI, // deassert threshold reading mask
		0x00, // discrete reading mask/ settable
		IPMI_SDR_UCT_SETTABLE | IPMI_SDR_LCT_SETTABLE | IPMI_SDR_UCT_READABLE |
			IPMI_SDR_LCT_READABLE, // threshold mask/ readable threshold mask
		0x80, // sensor unit
		IPMI_SENSOR_UNIT_DEGREE_C, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		0x01, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		0x00, // Rexp, Bexp
		0x00, // analog characteristic
		0x00, // nominal reading
		0x00, // normal maximum
		0x00, // normal minimum
		0x00, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		0x46, // UCT
		0x00, // UNCT
		0x00, // LNRT
		0x0A, // LCT
		0x00, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"MB Outlet Temp",
	},
	{
		// TMP75 on board temperature - IOM
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_TEMP_TMP75_IOM, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_TEMPERATURE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		0x00, // assert event mask
		IPMI_SDR_CMP_RETURN_LCT | IPMI_SDR_ASSERT_MASK_UCT_HI |
			IPMI_SDR_ASSERT_MASK_LCT_LO, // assert threshold reading mask
		0x00, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT | IPMI_SDR_DEASSERT_MASK_UCT_LO |
			IPMI_SDR_DEASSERT_MASK_LCT_HI, // deassert threshold reading mask
		0x00, // discrete reading mask/ settable
		IPMI_SDR_UCT_SETTABLE | IPMI_SDR_LCT_SETTABLE | IPMI_SDR_UCT_READABLE |
			IPMI_SDR_LCT_READABLE, // threshold mask/ readable threshold mask
		0x80, // sensor unit
		IPMI_SENSOR_UNIT_DEGREE_C, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		0x01, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		0x00, // Rexp, Bexp
		0x00, // analog characteristic
		0x00, // nominal reading
		0x00, // normal maximum
		0x00, // normal minimum
		0x00, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		0x28, // UCT
		0x00, // UNCT
		0x00, // LNRT
		0x0A, // LCT
		0x00, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"IOM Temp",
	},
	{
		// PCH temperature
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_TEMP_PCH, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_TEMPERATURE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		0x00, // assert event mask
		IPMI_SDR_CMP_RETURN_LCT | IPMI_SDR_ASSERT_MASK_UCT_HI |
			IPMI_SDR_ASSERT_MASK_LCT_LO, // assert threshold reading mask
		0x00, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT | IPMI_SDR_DEASSERT_MASK_UCT_LO |
			IPMI_SDR_DEASSERT_MASK_LCT_HI, // deassert threshold reading mask
		0x00, // discrete reading mask/ settable
		IPMI_SDR_UCT_SETTABLE | IPMI_SDR_LCT_SETTABLE | IPMI_SDR_UCT_READABLE |
			IPMI_SDR_LCT_READABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_DEGREE_C, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		0x01, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		0x00, // Rexp, Bexp
		0x00, // analog characteristic
		0x00, // nominal reading
		0x00, // normal maximum
		0x00, // normal minimum
		0x00, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		0x4A, // UCT
		0x00, // UNCT
		0x00, // LNRT
		0x00, // LCT
		0x00, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"PCH Temp",
	},
	{
		// CPU TEMP on board temperature
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_TEMP_CPU, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_TEMPERATURE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		0x00, // assert event mask
		IPMI_SDR_CMP_RETURN_LCT | IPMI_SDR_ASSERT_MASK_UCT_HI |
			IPMI_SDR_ASSERT_MASK_LCT_LO, // assert threshold reading mask
		0x00, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT | IPMI_SDR_DEASSERT_MASK_UCT_LO |
			IPMI_SDR_DEASSERT_MASK_LCT_HI, // deassert threshold reading mask
		0x00, // discrete reading mask/ settable
		IPMI_SDR_UCT_SETTABLE | IPMI_SDR_LCT_SETTABLE | IPMI_SDR_UCT_READABLE |
			IPMI_SDR_LCT_READABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_DEGREE_C, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		0x01, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		0x00, // Rexp, Bexp
		0x00, // analog characteristic
		0x00, // nominal reading
		0x00, // normal maximum
		0x00, // normal minimum
		0x00, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		0x54, // UCT
		0x00, // UNCT
		0x00, // LNRT
		0x00, // LCT
		0x00, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"SOC CPU Temp",
	},
	{
		// CPU margin on board temperature
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_TEMP_CPU_MARGIN, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_TEMPERATURE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		0x00, // assert event mask
		IPMI_SDR_CMP_RETURN_LCT | IPMI_SDR_ASSERT_MASK_UCT_HI |
			IPMI_SDR_ASSERT_MASK_LCT_LO, // assert threshold reading mask
		0x00, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT | IPMI_SDR_DEASSERT_MASK_UCT_LO |
			IPMI_SDR_DEASSERT_MASK_LCT_HI, // deassert threshold reading mask
		0x00, // discrete reading mask/ settable
		IPMI_SDR_UCT_SETTABLE | IPMI_SDR_LCT_SETTABLE | IPMI_SDR_UCT_READABLE |
			IPMI_SDR_LCT_READABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_DEGREE_C, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		0x01, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		0x00, // Rexp, Bexp
		0x00, // analog characteristic
		0x00, // nominal reading
		0x00, // normal maximum
		0x00, // normal minimum
		0x00, // sensor maximum reading
		0x00, // sensor minimum reading
		0x7D, // UNRT
		0x00, // UCT
		0x00, // UNCT
		0x00, // LNRT
		0x00, // LCT
		0x00, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"SOC Therm Margin",
	},
	{
		// CPU TJMAX on board temperature
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_TEMP_CPU_TJMAX, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_TEMPERATURE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		0x00, // assert event mask
		IPMI_SDR_CMP_RETURN_LCT | IPMI_SDR_ASSERT_MASK_UCT_HI |
			IPMI_SDR_ASSERT_MASK_LCT_LO, // assert threshold reading mask
		0x00, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT | IPMI_SDR_DEASSERT_MASK_UCT_LO |
			IPMI_SDR_DEASSERT_MASK_LCT_HI, // deassert threshold reading mask
		0x00, // discrete reading mask/ settable
		IPMI_SDR_UCT_SETTABLE | IPMI_SDR_LCT_SETTABLE | IPMI_SDR_UCT_READABLE |
			IPMI_SDR_LCT_READABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_DEGREE_C, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		0x01, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		0x00, // Rexp, Bexp
		0x00, // analog characteristic
		0x00, // nominal reading
		0x00, // normal maximum
		0x00, // normal minimum
		0x00, // sensor maximum reading
		0x00, // sensor minimum reading
		0x7D, // UNRT
		0x00, // UCT
		0x00, // UNCT
		0x00, // LNRT
		0x00, // LCT
		0x00, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"CPU TjMax",
	},
	{
		// SSD0 temperature
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_TEMP_SSD0, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_TEMPERATURE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		0x00, // assert event mask
		IPMI_SDR_CMP_RETURN_LCT | IPMI_SDR_ASSERT_MASK_UCT_HI |
			IPMI_SDR_ASSERT_MASK_LCT_LO, // assert threshold reading mask
		0x00, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT | IPMI_SDR_DEASSERT_MASK_UCT_LO |
			IPMI_SDR_DEASSERT_MASK_LCT_HI, // deassert threshold reading mask
		0x00, // discrete reading mask/ settable
		IPMI_SDR_UCT_SETTABLE | IPMI_SDR_LCT_SETTABLE | IPMI_SDR_UCT_READABLE |
			IPMI_SDR_LCT_READABLE, // threshold mask/ readable threshold mask
		0x80, // sensor unit
		IPMI_SENSOR_UNIT_DEGREE_C, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		0x01, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		0x00, // Rexp, Bexp
		0x00, // analog characteristic
		0x00, // nominal reading
		0x00, // normal maximum
		0x00, // normal minimum
		0x00, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		0x4B, // UCT
		0x00, // UNCT
		0x00, // LNRT
		0x00, // LCT
		0x00, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"SSD0 Temp",
	},
	{
		// HSC temperature
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_TEMP_HSC, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_TEMPERATURE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		0x00, // assert event mask
		IPMI_SDR_CMP_RETURN_LCT | IPMI_SDR_ASSERT_MASK_UCT_HI |
			IPMI_SDR_ASSERT_MASK_LCT_LO, // assert threshold reading mask
		0x00, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT | IPMI_SDR_DEASSERT_MASK_UCT_LO |
			IPMI_SDR_DEASSERT_MASK_LCT_HI, // deassert threshold reading mask
		0x00, // discrete reading mask/ settable
		IPMI_SDR_UCT_SETTABLE | IPMI_SDR_LCT_SETTABLE | IPMI_SDR_UCT_READABLE |
			IPMI_SDR_LCT_READABLE, // threshold mask/ readable threshold mask
		0x80, // sensor unit
		IPMI_SENSOR_UNIT_DEGREE_C, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		0x01, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		0x00, // Rexp, Bexp
		0x00, // analog characteristic
		0x00, // nominal reading
		0x00, // normal maximum
		0x00, // normal minimum
		0x00, // sensor maximum reading
		0x00, // sensor minimum reading
		0x7D, // UNRT
		0x56, // UCT
		0x00, // UNCT
		0x00, // LNRT
		0x00, // LCT
		0x00, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"HSC Temp",
	},
	{
		// DIMM A on board temperature
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_TEMP_DIMM_A, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_TEMPERATURE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		0x00, // assert event mask
		IPMI_SDR_CMP_RETURN_LCT | IPMI_SDR_ASSERT_MASK_UCT_HI |
			IPMI_SDR_ASSERT_MASK_LCT_LO, // assert threshold reading mask
		0x00, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT | IPMI_SDR_DEASSERT_MASK_UCT_LO |
			IPMI_SDR_DEASSERT_MASK_LCT_HI, // deassert threshold reading mask
		0x00, // discrete reading mask/ settable
		IPMI_SDR_UCT_SETTABLE | IPMI_SDR_LCT_SETTABLE | IPMI_SDR_UCT_READABLE |
			IPMI_SDR_LCT_READABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_DEGREE_C, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		0x01, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		0x00, // Rexp, Bexp
		0x00, // analog characteristic
		0x00, // nominal reading
		0x00, // normal maximum
		0x00, // normal minimum
		0x00, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		0x55, // UCT
		0x00, // UNCT
		0x00, // LNRT
		0x00, // LCT
		0x00, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"DIMMA Temp",
	},
	{
		// DIMM C on board temperature
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_TEMP_DIMM_C, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_TEMPERATURE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		0x00, // assert event mask
		IPMI_SDR_CMP_RETURN_LCT | IPMI_SDR_ASSERT_MASK_UCT_HI |
			IPMI_SDR_ASSERT_MASK_LCT_LO, // assert threshold reading mask
		0x00, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT | IPMI_SDR_DEASSERT_MASK_UCT_LO |
			IPMI_SDR_DEASSERT_MASK_LCT_HI, // deassert threshold reading mask
		0x00, // discrete reading mask/ settable
		IPMI_SDR_UCT_SETTABLE | IPMI_SDR_LCT_SETTABLE | IPMI_SDR_UCT_READABLE |
			IPMI_SDR_LCT_READABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_DEGREE_C, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		0x01, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		0x00, // Rexp, Bexp
		0x00, // analog characteristic
		0x00, // nominal reading
		0x00, // normal maximum
		0x00, // normal minimum
		0x00, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		0x55, // UCT
		0x00, // UNCT
		0x00, // LNRT
		0x00, // LCT
		0x00, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"DIMMC Temp",
	},
	{
		// DIMM E on board temperature
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_TEMP_DIMM_E, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_TEMPERATURE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		0x00, // assert event mask
		IPMI_SDR_CMP_RETURN_LCT | IPMI_SDR_ASSERT_MASK_UCT_HI |
			IPMI_SDR_ASSERT_MASK_LCT_LO, // assert threshold reading mask
		0x00, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT | IPMI_SDR_DEASSERT_MASK_UCT_LO |
			IPMI_SDR_DEASSERT_MASK_LCT_HI, // deassert threshold reading mask
		0x00, // discrete reading mask/ settable
		IPMI_SDR_UCT_SETTABLE | IPMI_SDR_LCT_SETTABLE | IPMI_SDR_UCT_READABLE |
			IPMI_SDR_LCT_READABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_DEGREE_C, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		0x01, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		0x00, // Rexp, Bexp
		0x00, // analog characteristic
		0x00, // nominal reading
		0x00, // normal maximum
		0x00, // normal minimum
		0x00, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		0x55, // UCT
		0x00, // UNCT
		0x00, // LNRT
		0x00, // LCT
		0x00, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"DIMME Temp",
	},
	{
		// DIMM G on board temperature
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_TEMP_DIMM_G, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_TEMPERATURE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		0x00, // assert event mask
		IPMI_SDR_CMP_RETURN_LCT | IPMI_SDR_ASSERT_MASK_UCT_HI |
			IPMI_SDR_ASSERT_MASK_LCT_LO, // assert threshold reading mask
		0x00, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT | IPMI_SDR_DEASSERT_MASK_UCT_LO |
			IPMI_SDR_DEASSERT_MASK_LCT_HI, // deassert threshold reading mask
		0x00, // discrete reading mask/ settable
		IPMI_SDR_UCT_SETTABLE | IPMI_SDR_LCT_SETTABLE | IPMI_SDR_UCT_READABLE |
			IPMI_SDR_LCT_READABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_DEGREE_C, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		0x01, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		0x00, // Rexp, Bexp
		0x00, // analog characteristic
		0x00, // nominal reading
		0x00, // normal maximum
		0x00, // normal minimum
		0x00, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		0x55, // UCT
		0x00, // UNCT
		0x00, // LNRT
		0x00, // LCT
		0x00, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"DIMMG Temp",
	},

	/* VOLTAGE SENSOR */
	{
		// P12V STBY ADC voltage
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_VOL_STBY12V, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_TEMPERATURE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		0x00, // assert event mask
		IPMI_SDR_CMP_RETURN_LCT | IPMI_SDR_ASSERT_MASK_UCT_HI |
			IPMI_SDR_ASSERT_MASK_LCT_LO, // assert threshold reading mask
		0x00, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT | IPMI_SDR_DEASSERT_MASK_UCT_LO |
			IPMI_SDR_DEASSERT_MASK_LCT_HI, // deassert threshold reading mask
		0x00, // discrete reading mask/ settable
		IPMI_SDR_UCT_SETTABLE | IPMI_SDR_LCT_SETTABLE | IPMI_SDR_UCT_READABLE |
			IPMI_SDR_LCT_READABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_VOL, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		0x3F, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		0xD0, // Rexp, Bexp
		0x00, // analog characteristic
		0x00, // nominal reading
		0x00, // normal maximum
		0x00, // normal minimum
		0x00, // sensor maximum reading
		0x00, // sensor minimum reading
		0xE4, // UNRT
		0xD4, // UCT
		0xD2, // UNCT
		0xA0, // LNRT
		0xAA, // LCT
		0xAC, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"P12V_STBY Vol",
	},
	{
		// BATTERY 3V ADC voltage
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_VOL_BAT3V, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_TEMPERATURE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		0x00, // assert event mask
		IPMI_SDR_CMP_RETURN_LCT | IPMI_SDR_ASSERT_MASK_UCT_HI |
			IPMI_SDR_ASSERT_MASK_LCT_LO, // assert threshold reading mask
		0x00, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT | IPMI_SDR_DEASSERT_MASK_UCT_LO |
			IPMI_SDR_DEASSERT_MASK_LCT_HI, // deassert threshold reading mask
		0x00, // discrete reading mask/ settable
		IPMI_SDR_UCT_SETTABLE | IPMI_SDR_LCT_SETTABLE | IPMI_SDR_UCT_READABLE |
			IPMI_SDR_LCT_READABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_VOL, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		0x0F, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		0xD0, // Rexp, Bexp
		0x00, // analog characteristic
		0x00, // nominal reading
		0x00, // normal maximum
		0x00, // normal minimum
		0x00, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		0xEA, // UCT
		0xE7, // UNCT
		0x00, // LNRT
		0xB8, // LCT
		0xBA, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"P3V_BAT Vol",
	},
	{
		// P3V3 STBY ADC voltage
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_VOL_STBY3V, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_TEMPERATURE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		0x00, // assert event mask
		IPMI_SDR_CMP_RETURN_LCT | IPMI_SDR_ASSERT_MASK_UCT_HI |
			IPMI_SDR_ASSERT_MASK_LCT_LO, // assert threshold reading mask
		0x00, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT | IPMI_SDR_DEASSERT_MASK_UCT_LO |
			IPMI_SDR_DEASSERT_MASK_LCT_HI, // deassert threshold reading mask
		0x00, // discrete reading mask/ settable
		IPMI_SDR_UCT_SETTABLE | IPMI_SDR_LCT_SETTABLE | IPMI_SDR_UCT_READABLE |
			IPMI_SDR_LCT_READABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_VOL, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		0x12, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		0xD0, // Rexp, Bexp
		0x00, // analog characteristic
		0x00, // nominal reading
		0x00, // normal maximum
		0x00, // normal minimum
		0x00, // sensor maximum reading
		0x00, // sensor minimum reading
		0xDE, // UNRT
		0xC6, // UCT
		0xC4, // UNCT
		0x80, // LNRT
		0xA9, // LCT
		0xAB, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"P3V3_STBY Vol",
	},
	{
		// P1V8 STBY ADC voltage
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_VOL_STBY1V8, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_TEMPERATURE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		0x00, // assert event mask
		IPMI_SDR_CMP_RETURN_LCT | IPMI_SDR_ASSERT_MASK_UCT_HI |
			IPMI_SDR_ASSERT_MASK_LCT_LO, // assert threshold reading mask
		0x00, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT | IPMI_SDR_DEASSERT_MASK_UCT_LO |
			IPMI_SDR_DEASSERT_MASK_LCT_HI, // deassert threshold reading mask
		0x00, // discrete reading mask/ settable
		IPMI_SDR_UCT_SETTABLE | IPMI_SDR_LCT_SETTABLE | IPMI_SDR_UCT_READABLE |
			IPMI_SDR_LCT_READABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_VOL, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		0x09, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		0xD0, // Rexp, Bexp
		0x00, // analog characteristic
		0x00, // nominal reading
		0x00, // normal maximum
		0x00, // normal minimum
		0x00, // sensor maximum reading
		0x00, // sensor minimum reading
		0xE8, // UNRT
		0xD5, // UCT
		0xD3, // UNCT
		0xA0, // LNRT
		0xBA, // LCT
		0xBC, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"P1V8_STBY Vol",
	},
	{
		// P1V05 STBY ADC voltage
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_VOL_STBY1V05, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_TEMPERATURE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		0x00, // assert event mask
		IPMI_SDR_CMP_RETURN_LCT | IPMI_SDR_ASSERT_MASK_UCT_HI |
			IPMI_SDR_ASSERT_MASK_LCT_LO, // assert threshold reading mask
		0x00, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT | IPMI_SDR_DEASSERT_MASK_UCT_LO |
			IPMI_SDR_DEASSERT_MASK_LCT_HI, // deassert threshold reading mask
		0x00, // discrete reading mask/ settable
		IPMI_SDR_UCT_SETTABLE | IPMI_SDR_LCT_SETTABLE | IPMI_SDR_UCT_READABLE |
			IPMI_SDR_LCT_READABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_VOL, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		0x06, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		0xD0, // Rexp, Bexp
		0x00, // analog characteristic
		0x00, // nominal reading
		0x00, // normal maximum
		0x00, // normal minimum
		0x00, // sensor maximum reading
		0x00, // sensor minimum reading
		0xCB, // UNRT
		0xBA, // UCT
		0xB8, // UNCT
		0x8C, // LNRT
		0xA1, // LCT
		0xA3, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"P1V05_PCH Vol",
	},
	{
		// P5V STBY ADC voltage
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_VOL_STBY5V, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_TEMPERATURE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		0x00, // assert event mask
		IPMI_SDR_CMP_RETURN_LCT | IPMI_SDR_ASSERT_MASK_UCT_HI |
			IPMI_SDR_ASSERT_MASK_LCT_LO, // assert threshold reading mask
		0x00, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT | IPMI_SDR_DEASSERT_MASK_UCT_LO |
			IPMI_SDR_DEASSERT_MASK_LCT_HI, // deassert threshold reading mask
		0x00, // discrete reading mask/ settable
		IPMI_SDR_UCT_SETTABLE | IPMI_SDR_LCT_SETTABLE | IPMI_SDR_UCT_READABLE |
			IPMI_SDR_LCT_READABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_VOL, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		0x1B, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		0xD0, // Rexp, Bexp
		0x00, // analog characteristic
		0x00, // nominal reading
		0x00, // normal maximum
		0x00, // normal minimum
		0x00, // sensor maximum reading
		0x00, // sensor minimum reading
		0xD7, // UNRT
		0xC8, // UCT
		0xC6, // UNCT
		0x94, // LNRT
		0xAA, // LCT
		0xAC, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"P5V_STBY Vol",
	},
	{
		// P12V DIMM ADC voltage
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_VOL_DIMM12V, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_TEMPERATURE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		0x00, // assert event mask
		IPMI_SDR_CMP_RETURN_LCT | IPMI_SDR_ASSERT_MASK_UCT_HI |
			IPMI_SDR_ASSERT_MASK_LCT_LO, // assert threshold reading mask
		0x00, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT | IPMI_SDR_DEASSERT_MASK_UCT_LO |
			IPMI_SDR_DEASSERT_MASK_LCT_HI, // deassert threshold reading mask
		0x00, // discrete reading mask/ settable
		IPMI_SDR_UCT_SETTABLE | IPMI_SDR_LCT_SETTABLE | IPMI_SDR_UCT_READABLE |
			IPMI_SDR_LCT_READABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_VOL, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		0x07, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		0xE0, // Rexp, Bexp
		0x00, // analog characteristic
		0x00, // nominal reading
		0x00, // normal maximum
		0x00, // normal minimum
		0x00, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		0xCB, // UCT
		0xC9, // UNCT
		0x00, // LNRT
		0x8D, // LCT
		0x8F, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"P12V_DIMM Vol",
	},
	{
		// P1V2 STBY ADC voltage
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_VOL_STBY1V2, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_TEMPERATURE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		0x00, // assert event mask
		IPMI_SDR_CMP_RETURN_LCT | IPMI_SDR_ASSERT_MASK_UCT_HI |
			IPMI_SDR_ASSERT_MASK_LCT_LO, // assert threshold reading mask
		0x00, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT | IPMI_SDR_DEASSERT_MASK_UCT_LO |
			IPMI_SDR_DEASSERT_MASK_LCT_HI, // deassert threshold reading mask
		0x00, // discrete reading mask/ settable
		IPMI_SDR_UCT_SETTABLE | IPMI_SDR_LCT_SETTABLE | IPMI_SDR_UCT_READABLE |
			IPMI_SDR_LCT_READABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_VOL, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		0x06, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		0xD0, // Rexp, Bexp
		0x00, // analog characteristic
		0x00, // nominal reading
		0x00, // normal maximum
		0x00, // normal minimum
		0x00, // sensor maximum reading
		0x00, // sensor minimum reading
		0xE8, // UNRT
		0xD8, // UCT
		0xD6, // UNCT
		0xA0, // LNRT
		0xB8, // LCT
		0xBA, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"P1V2_STBY Vol",
	},
	{
		// P3V3 M2 ADC voltage
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_VOL_M2_3V3, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_TEMPERATURE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		0x00, // assert event mask
		IPMI_SDR_CMP_RETURN_LCT | IPMI_SDR_ASSERT_MASK_UCT_HI |
			IPMI_SDR_ASSERT_MASK_LCT_LO, // assert threshold reading mask
		0x00, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT | IPMI_SDR_DEASSERT_MASK_UCT_LO |
			IPMI_SDR_DEASSERT_MASK_LCT_HI, // deassert threshold reading mask
		0x00, // discrete reading mask/ settable
		IPMI_SDR_UCT_SETTABLE | IPMI_SDR_LCT_SETTABLE | IPMI_SDR_UCT_READABLE |
			IPMI_SDR_LCT_READABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_VOL, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		0x11, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		0xD0, // Rexp, Bexp
		0x00, // analog characteristic
		0x00, // nominal reading
		0x00, // normal maximum
		0x00, // normal minimum
		0x00, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		0xDA, // UCT
		0xD8, // UNCT
		0x00, // LNRT
		0xAB, // LCT
		0xAD, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"P3V3_M2 Vol",
	},
	{
		// HSCIN voltage
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_VOL_HSCIN, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_TEMPERATURE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		0x00, // assert event mask
		IPMI_SDR_CMP_RETURN_LCT | IPMI_SDR_ASSERT_MASK_UCT_HI |
			IPMI_SDR_ASSERT_MASK_LCT_LO, // assert threshold reading mask
		0x00, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT | IPMI_SDR_DEASSERT_MASK_UCT_LO |
			IPMI_SDR_DEASSERT_MASK_LCT_HI, // deassert threshold reading mask
		0x00, // discrete reading mask/ settable
		IPMI_SDR_UCT_SETTABLE | IPMI_SDR_LCT_SETTABLE | IPMI_SDR_UCT_READABLE |
			IPMI_SDR_LCT_READABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_VOL, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		0x3F, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		0xD0, // Rexp, Bexp
		0x00, // analog characteristic
		0x00, // nominal reading
		0x00, // normal maximum
		0x00, // normal minimum
		0x00, // sensor maximum reading
		0x00, // sensor minimum reading
		0xE4, // UNRT
		0xD4, // UCT
		0xD2, // UNCT
		0xA0, // LNRT
		0xAA, // LCT
		0xAC, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"HSC Input Vol",
	},
	{
		// IOM INA230 voltage
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_VOL_IOM_INA, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_TEMPERATURE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		0x00, // assert event mask
		IPMI_SDR_CMP_RETURN_LCT | IPMI_SDR_ASSERT_MASK_UCT_HI |
			IPMI_SDR_ASSERT_MASK_LCT_LO, // assert threshold reading mask
		0x00, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT | IPMI_SDR_DEASSERT_MASK_UCT_LO |
			IPMI_SDR_DEASSERT_MASK_LCT_HI, // deassert threshold reading mask
		0x00, // discrete reading mask/ settable
		IPMI_SDR_UCT_SETTABLE | IPMI_SDR_LCT_SETTABLE | IPMI_SDR_UCT_READABLE |
			IPMI_SDR_LCT_READABLE, // threshold mask/ readable threshold mask
		0x80, // sensor unit
		IPMI_SENSOR_UNIT_AMP, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		0x1A, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		0xE0, // Rexp, Bexp
		0x00, // analog characteristic
		0x00, // nominal reading
		0x00, // normal maximum
		0x00, // normal minimum
		0x00, // sensor maximum reading
		0x00, // sensor minimum reading
		0xE7, // UNRT
		0xB1, // UCT
		0xA2, // UNCT
		0x00, // LNRT
		0x00, // LCT
		0x00, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"IOM INA vol",
	},

	/* CURRENT SENSOR */
	{
		// HSCOUT current
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_CUR_HSCOUT, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_TEMPERATURE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		0x00, // assert event mask
		IPMI_SDR_CMP_RETURN_LCT | IPMI_SDR_ASSERT_MASK_UCT_HI |
			IPMI_SDR_ASSERT_MASK_LCT_LO, // assert threshold reading mask
		0x00, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT | IPMI_SDR_DEASSERT_MASK_UCT_LO |
			IPMI_SDR_DEASSERT_MASK_LCT_HI, // deassert threshold reading mask
		0x00, // discrete reading mask/ settable
		IPMI_SDR_UCT_SETTABLE | IPMI_SDR_LCT_SETTABLE | IPMI_SDR_UCT_READABLE |
			IPMI_SDR_LCT_READABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_AMP, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		0x11, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		0xE0, // Rexp, Bexp
		0x00, // analog characteristic
		0x00, // nominal reading
		0x00, // normal maximum
		0x00, // normal minimum
		0x00, // sensor maximum reading
		0x00, // sensor minimum reading
		0xEB, // UNRT
		0xBC, // UCT
		0xA9, // UNCT
		0x00, // LNRT
		0x00, // LCT
		0x00, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"HSC Output Cur",
	},
	{
		// IOM INA230 current
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_CUR_IOM_INA, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_TEMPERATURE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		0x00, // assert event mask
		IPMI_SDR_CMP_RETURN_LCT | IPMI_SDR_ASSERT_MASK_UCT_HI |
			IPMI_SDR_ASSERT_MASK_LCT_LO, // assert threshold reading mask
		0x00, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT | IPMI_SDR_DEASSERT_MASK_UCT_LO |
			IPMI_SDR_DEASSERT_MASK_LCT_HI, // deassert threshold reading mask
		0x00, // discrete reading mask/ settable
		IPMI_SDR_UCT_SETTABLE | IPMI_SDR_LCT_SETTABLE | IPMI_SDR_UCT_READABLE |
			IPMI_SDR_LCT_READABLE, // threshold mask/ readable threshold mask
		0x80, // sensor unit
		IPMI_SENSOR_UNIT_AMP, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		0x1A, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		0xE0, // Rexp, Bexp
		0x00, // analog characteristic
		0x00, // nominal reading
		0x00, // normal maximum
		0x00, // normal minimum
		0x00, // sensor maximum reading
		0x00, // sensor minimum reading
		0xE7, // UNRT
		0xB1, // UCT
		0xA2, // UNCT
		0x00, // LNRT
		0x00, // LCT
		0x00, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"IOM INA Cur",
	},

	/* POWER SENSOR */
	{
		// CPU power
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_PWR_CPU, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_TEMPERATURE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		0x00, // assert event mask
		IPMI_SDR_CMP_RETURN_LCT | IPMI_SDR_ASSERT_MASK_UCT_HI |
			IPMI_SDR_ASSERT_MASK_LCT_LO, // assert threshold reading mask
		0x00, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT | IPMI_SDR_DEASSERT_MASK_UCT_LO |
			IPMI_SDR_DEASSERT_MASK_LCT_HI, // deassert threshold reading mask
		0x00, // discrete reading mask/ settable
		IPMI_SDR_UCT_SETTABLE | IPMI_SDR_LCT_SETTABLE | IPMI_SDR_UCT_READABLE |
			IPMI_SDR_LCT_READABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_WATT, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		0x01, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		0x00, // Rexp, Bexp
		0x00, // analog characteristic
		0x00, // nominal reading
		0x00, // normal maximum
		0x00, // normal minimum
		0x00, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		0x00, // UCT
		0x00, // UNCT
		0x00, // LNRT
		0x00, // LCT
		0x00, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"CPU Pwr",
	},
	{
		// HSCIN power
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_PWR_HSCIN, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_TEMPERATURE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		0x00, // assert event mask
		IPMI_SDR_CMP_RETURN_LCT | IPMI_SDR_ASSERT_MASK_UCT_HI |
			IPMI_SDR_ASSERT_MASK_LCT_LO, // assert threshold reading mask
		0x00, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT | IPMI_SDR_DEASSERT_MASK_UCT_LO |
			IPMI_SDR_DEASSERT_MASK_LCT_HI, // deassert threshold reading mask
		0x00, // discrete reading mask/ settable
		IPMI_SDR_UCT_SETTABLE | IPMI_SDR_LCT_SETTABLE | IPMI_SDR_UCT_READABLE |
			IPMI_SDR_LCT_READABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_WATT, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		0xD9, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		0xE0, // Rexp, Bexp
		0x00, // analog characteristic
		0x00, // nominal reading
		0x00, // normal maximum
		0x00, // normal minimum
		0x00, // sensor maximum reading
		0x00, // sensor minimum reading
		0xE6, // UNRT
		0xB8, // UCT
		0xA6, // UNCT
		0x00, // LNRT
		0x00, // LCT
		0x00, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"HSC Input Pwr",
	},
	{
		// DIMMA power
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_PWR_DIMMA_PMIC, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_TEMPERATURE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		0x00, // assert event mask
		IPMI_SDR_CMP_RETURN_LCT | IPMI_SDR_ASSERT_MASK_UCT_HI |
			IPMI_SDR_ASSERT_MASK_LCT_LO, // assert threshold reading mask
		0x00, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT | IPMI_SDR_DEASSERT_MASK_UCT_LO |
			IPMI_SDR_DEASSERT_MASK_LCT_HI, // deassert threshold reading mask
		0x00, // discrete reading mask/ settable
		IPMI_SDR_UCT_SETTABLE | IPMI_SDR_LCT_SETTABLE | IPMI_SDR_UCT_READABLE |
			IPMI_SDR_LCT_READABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_WATT, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		0x0E, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		0xE0, // Rexp, Bexp
		0x00, // analog characteristic
		0x00, // nominal reading
		0x00, // normal maximum
		0x00, // normal minimum
		0x00, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		0xE6, // UCT
		0xE1, // UNCT
		0x00, // LNRT
		0x00, // LCT
		0x00, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"DIMMA PMIC_Pout",
	},
	{
		// DIMMC power
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_PWR_DIMMC_PMIC, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_TEMPERATURE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		0x00, // assert event mask
		IPMI_SDR_CMP_RETURN_LCT | IPMI_SDR_ASSERT_MASK_UCT_HI |
			IPMI_SDR_ASSERT_MASK_LCT_LO, // assert threshold reading mask
		0x00, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT | IPMI_SDR_DEASSERT_MASK_UCT_LO |
			IPMI_SDR_DEASSERT_MASK_LCT_HI, // deassert threshold reading mask
		0x00, // discrete reading mask/ settable
		IPMI_SDR_UCT_SETTABLE | IPMI_SDR_LCT_SETTABLE | IPMI_SDR_UCT_READABLE |
			IPMI_SDR_LCT_READABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_WATT, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		0x0E, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		0xE0, // Rexp, Bexp
		0x00, // analog characteristic
		0x00, // nominal reading
		0x00, // normal maximum
		0x00, // normal minimum
		0x00, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		0xE6, // UCT
		0xE1, // UNCT
		0x00, // LNRT
		0x00, // LCT
		0x00, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"DIMMC PMIC_Pout",
	},
	{
		// DIMME power
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_PWR_DIMME_PMIC, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_TEMPERATURE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		0x00, // assert event mask
		IPMI_SDR_CMP_RETURN_LCT | IPMI_SDR_ASSERT_MASK_UCT_HI |
			IPMI_SDR_ASSERT_MASK_LCT_LO, // assert threshold reading mask
		0x00, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT | IPMI_SDR_DEASSERT_MASK_UCT_LO |
			IPMI_SDR_DEASSERT_MASK_LCT_HI, // deassert threshold reading mask
		0x00, // discrete reading mask/ settable
		IPMI_SDR_UCT_SETTABLE | IPMI_SDR_LCT_SETTABLE | IPMI_SDR_UCT_READABLE |
			IPMI_SDR_LCT_READABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_WATT, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		0x0E, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		0xE0, // Rexp, Bexp
		0x00, // analog characteristic
		0x00, // nominal reading
		0x00, // normal maximum
		0x00, // normal minimum
		0x00, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		0xE6, // UCT
		0xE1, // UNCT
		0x00, // LNRT
		0x00, // LCT
		0x00, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"DIMME PMIC_Pout",
	},
	{
		// DIMMG power
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_PWR_DIMMG_PMIC, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_TEMPERATURE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		0x00, // assert event mask
		IPMI_SDR_CMP_RETURN_LCT | IPMI_SDR_ASSERT_MASK_UCT_HI |
			IPMI_SDR_ASSERT_MASK_LCT_LO, // assert threshold reading mask
		0x00, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT | IPMI_SDR_DEASSERT_MASK_UCT_LO |
			IPMI_SDR_DEASSERT_MASK_LCT_HI, // deassert threshold reading mask
		0x00, // discrete reading mask/ settable
		IPMI_SDR_UCT_SETTABLE | IPMI_SDR_LCT_SETTABLE | IPMI_SDR_UCT_READABLE |
			IPMI_SDR_LCT_READABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_WATT, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		0x0E, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		0xE0, // Rexp, Bexp
		0x00, // analog characteristic
		0x00, // nominal reading
		0x00, // normal maximum
		0x00, // normal minimum
		0x00, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		0xE6, // UCT
		0xE1, // UNCT
		0x00, // LNRT
		0x00, // LCT
		0x00, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"DIMMG PMIC_Pout",
	},
	{
		// IOM INA230 power
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_PWR_IOM_INA, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_TEMPERATURE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		0x00, // assert event mask
		IPMI_SDR_CMP_RETURN_LCT | IPMI_SDR_ASSERT_MASK_UCT_HI |
			IPMI_SDR_ASSERT_MASK_LCT_LO, // assert threshold reading mask
		0x00, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT | IPMI_SDR_DEASSERT_MASK_UCT_LO |
			IPMI_SDR_DEASSERT_MASK_LCT_HI, // deassert threshold reading mask
		0x00, // discrete reading mask/ settable
		IPMI_SDR_UCT_SETTABLE | IPMI_SDR_LCT_SETTABLE | IPMI_SDR_UCT_READABLE |
			IPMI_SDR_LCT_READABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_WATT, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		0x0E, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		0xE0, // Rexp, Bexp
		0x00, // analog characteristic
		0x00, // nominal reading
		0x00, // normal maximum
		0x00, // normal minimum
		0x00, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		0xE6, // UCT
		0xE1, // UNCT
		0x00, // LNRT
		0x00, // LCT
		0x00, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"IOM INA Pwr",
	},
};

const int SDR_TABLE_SIZE = ARRAY_SIZE(plat_sdr_table);
