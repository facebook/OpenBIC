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

#include "plat_sdr_table.h"

#include <stdio.h>
#include <string.h>

#include "sdr.h"
#include "libutil.h"
#include <logging/log.h>
#include "plat_ipmb.h"
#include "plat_sensor_table.h"

LOG_MODULE_REGISTER(plat_sdr_table);

#define HSC_SENSOR 0

SDR_Full_sensor plat_sdr_table[] = {
	{
		/***********************************/
		/* outlet temperature,             */
		/***********************************/
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_T_MB_OUTLET_TEMP_T, // sensor number

		IPMI_SDR_ENTITY_ID_AIR_INLET, // entity id
		0x01, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_AUTO_RE_ARM | IPMI_SDR_SENSOR_CAP_HYSTERESIS_NO |
			IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_BIT, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_TEMPERATURE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		0x00, // assert event mask
		IPMI_SDR_ASSERT_MASK_UCT_HI, // assert threshold reading mask
		0x00, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT |
			IPMI_SDR_DEASSERT_MASK_UCT_HI, // deassert threshold reading mask
		IPMI_SDR_UCT_READABLE, // discrete reading mask/ settable
		IPMI_SDR_UCT_SETTABLE, // threshold mask/ readable threshold mask
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
		0x23, // nominal reading
		0x00, // normal maximum
		0x00, // normal minimum
		0xFF, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		0x34, // UCT
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
		"VF_E1S_OUTLET_TEMP_C",
	},
#if HSC_SENSOR
	{
		/***********************************/
		/* HSC Temperature,                */
		/***********************************/
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_T_HSC, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x01, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_AUTO_RE_ARM | IPMI_SDR_SENSOR_CAP_HYSTERESIS_NO |
			IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_BIT, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_TEMPERATURE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		0x00, // assert event mask
		IPMI_SDR_ASSERT_MASK_UCT_HI, // assert threshold reading mask
		0x00, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT |
			IPMI_SDR_DEASSERT_MASK_UCT_HI, // deassert threshold reading mask
		IPMI_SDR_UCT_READABLE, // discrete reading mask/ settable
		IPMI_SDR_UCT_SETTABLE, // threshold mask/ readable threshold mask
		0x80, // sensor unit
		IPMI_SENSOR_UNIT_DEGREE_C, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		HSC_T_SEN_FACTOR_M, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		HSC_T_SEN_FACTOR_EXP_RB, // Rexp, Bexp
		0x00, // analog characteristic
		0x23, // nominal reading
		0x00, // normal maximum
		0x00, // normal minimum
		0xFF, // sensor maximum reading
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
		"E1S HSC Temp",
	},
#endif
	{
		/***********************************/
		/* P12V_AUX,                       */
		/***********************************/
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_V_12_AUX, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x01, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_AUTO_RE_ARM | IPMI_SDR_SENSOR_CAP_HYSTERESIS_NO |
			IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_BIT, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_VOLTAGE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		IPMI_SDR_ASSERT_MASK_LCT_LO, // assert event mask
		IPMI_SDR_CMP_RETURN_LCT |
			IPMI_SDR_ASSERT_MASK_UCT_HI, // assert threshold reading mask
		IPMI_SDR_ASSERT_MASK_LCT_LO, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT |
			IPMI_SDR_DEASSERT_MASK_UCT_HI, // deassert threshold reading mask
		IPMI_SDR_LCT_SETTABLE | IPMI_SDR_UCT_READABLE, // discrete reading mask/ settable
		IPMI_SDR_LCT_SETTABLE |
			IPMI_SDR_UCT_READABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_VOL, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		ADC_12V_SEN_FACTOR_M, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		ADC_12V_SEN_FACTOR_RB, // Rexp, Bexp
		0x00, // analog characteristic
		0xBB, // nominal reading
		0xCA, // normal maximum
		0x98, // normal minimum
		0xFF, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		ADC_12V_UCT, // UCT
		0x00, // UNCT
		0x00, // LNRT
		ADC_12V_LCT, // LCT
		0x00, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"VF_E1S_P12V_AUX_VOLT_V",
	},
	{
		/***********************************/
		/* P12V_EDGE,                      */
		/***********************************/
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_V_12_EDGE, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x01, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_AUTO_RE_ARM | IPMI_SDR_SENSOR_CAP_HYSTERESIS_NO |
			IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_BIT, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_VOLTAGE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		IPMI_SDR_ASSERT_MASK_LCT_LO, // assert event mask
		IPMI_SDR_CMP_RETURN_LCT |
			IPMI_SDR_ASSERT_MASK_UCT_HI, // assert threshold reading mask
		IPMI_SDR_ASSERT_MASK_LCT_LO, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT |
			IPMI_SDR_DEASSERT_MASK_UCT_HI, // deassert threshold reading mask
		IPMI_SDR_LCT_SETTABLE | IPMI_SDR_UCT_READABLE, // discrete reading mask/ settable
		IPMI_SDR_LCT_SETTABLE |
			IPMI_SDR_UCT_READABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_VOL, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		ADC_12V_SEN_FACTOR_M, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		ADC_12V_SEN_FACTOR_RB, // Rexp, Bexp
		0x00, // analog characteristic
		0xBB, // nominal reading
		0xCA, // normal maximum
		0x98, // normal minimum
		0xFF, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		ADC_12V_UCT, // UCT
		0x00, // UNCT
		0x00, // LNRT
		ADC_12V_LCT, // LCT
		0x00, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"VF_E1S_P12V_EDGE_VOLT_V",
	},
	{
		/***********************************/
		/* P3V3_AUX,                       */
		/***********************************/
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_V_3_3_AUX, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x01, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_AUTO_RE_ARM | IPMI_SDR_SENSOR_CAP_HYSTERESIS_NO |
			IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_BIT, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_VOLTAGE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		IPMI_SDR_ASSERT_MASK_LCT_LO, // assert event mask
		IPMI_SDR_CMP_RETURN_LCT |
			IPMI_SDR_ASSERT_MASK_UCT_HI, // assert threshold reading mask
		IPMI_SDR_DEASSERT_MASK_LCT_LO, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT |
			IPMI_SDR_DEASSERT_MASK_UCT_HI, // deassert threshold reading mask
		IPMI_SDR_LCT_READABLE | IPMI_SDR_UCT_READABLE, // discrete reading mask/ settable
		IPMI_SDR_LCT_SETTABLE |
			IPMI_SDR_UCT_SETTABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_VOL, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		ADC_3V3_SEN_FACTOR_M, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		ADC_3V3_SEN_FACTOR_RB, // Rexp, Bexp
		0x00, // analog characteristic
		0xA7, // nominal reading
		0xB4, // normal maximum
		0xA5, // normal minimum
		0xFF, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		ADC_3V3_UCT, // UCT
		0x00, // UNCT
		0x00, // LNRT
		ADC_3V3_LCT, // LCT
		0x00, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"VF_E1S_P3V3_AUX_VOLT_V",
	},
	{
		/***********************************/
		/* P1V2_STBY,                      */
		/***********************************/
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_V_1_2_STBY, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x01, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_AUTO_RE_ARM | IPMI_SDR_SENSOR_CAP_HYSTERESIS_NO |
			IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_BIT, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_VOLTAGE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		IPMI_SDR_ASSERT_MASK_LCT_LO, // assert event mask
		IPMI_SDR_CMP_RETURN_LCT |
			IPMI_SDR_ASSERT_MASK_UCT_HI, // assert threshold reading mask
		IPMI_SDR_ASSERT_MASK_LCT_LO, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT |
			IPMI_SDR_DEASSERT_MASK_UCT_HI, // deassert threshold reading mask
		IPMI_SDR_LCT_SETTABLE | IPMI_SDR_UCT_READABLE, // discrete reading mask/ settable
		IPMI_SDR_LCT_SETTABLE |
			IPMI_SDR_UCT_READABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_VOL, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		0x01, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		0xE0, // Rexp, Bexp
		0x00, // analog characteristic
		0xBB, // nominal reading
		0xCA, // normal maximum
		0x98, // normal minimum
		0xFF, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		0x7E, // UCT
		0x00, // UNCT
		0x00, // LNRT
		0x72, // LCT
		0x00, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"VF_E1S_P1V2_STBY_VOLT_V",
	},
#if HSC_SENSOR
	{
		/***********************************/
		/* HSC INPUT VOLTAGE               */
		/***********************************/
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_V_HSC_IN, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x01, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_AUTO_RE_ARM | IPMI_SDR_SENSOR_CAP_HYSTERESIS_NO |
			IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_BIT, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_VOLTAGE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		IPMI_SDR_ASSERT_MASK_LCT_LO, // assert event mask
		IPMI_SDR_CMP_RETURN_LCT |
			IPMI_SDR_ASSERT_MASK_UCT_HI, // assert threshold reading mask
		IPMI_SDR_DEASSERT_MASK_LCT_LO, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT |
			IPMI_SDR_DEASSERT_MASK_UCT_HI, // deassert threshold reading mask
		IPMI_SDR_LCT_READABLE | IPMI_SDR_UCT_READABLE, // discrete reading mask/ settable
		IPMI_SDR_LCT_SETTABLE |
			IPMI_SDR_UCT_SETTABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_VOL, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		HSC_VIN_SEN_FACTOR_M, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		HSC_VIN_SEN_FACTOR_EXP_RB, // Rexp, Bexp
		0x00, // analog characteristic
		0x23, // nominal reading
		0xF0, // normal maximum
		0xBC, // normal minimum
		0xFF, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		0xDC, // UCT
		0x00, // UNCT
		0x00, // LNRT
		0xB4, // LCT
		0x00, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"E1S HSC VIN",
	},
#endif
	{
		/***********************************/
		/* ADC_12V_DEV0,                   */
		/***********************************/
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_ADC_12V_VOL_M2A, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x01, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_AUTO_RE_ARM | IPMI_SDR_SENSOR_CAP_HYSTERESIS_NO |
			IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_BIT, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_VOLTAGE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		IPMI_SDR_ASSERT_MASK_LCT_LO, // assert event mask
		IPMI_SDR_CMP_RETURN_LCT |
			IPMI_SDR_ASSERT_MASK_UCT_HI, // assert threshold reading mask
		IPMI_SDR_DEASSERT_MASK_LCT_LO, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT |
			IPMI_SDR_DEASSERT_MASK_UCT_HI, // deassert threshold reading mask
		IPMI_SDR_LCT_READABLE | IPMI_SDR_UCT_READABLE, // discrete reading mask/ settable
		IPMI_SDR_LCT_SETTABLE |
			IPMI_SDR_UCT_SETTABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_VOL, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		ADC_12V_SEN_FACTOR_M, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		ADC_12V_SEN_FACTOR_RB, // Rexp, Bexp
		0x00, // analog characteristic
		0xBB, // nominal reading
		0xCA, // normal maximum
		0x98, // normal minimum
		0xFF, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		ADC_12V_UCT, // UCT
		0x00, // UNCT
		0x00, // LNRT
		ADC_12V_LCT, // LCT
		0x00, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"VF_E1S_SSD0_12V_ADC_VOLT_V",
	},
	{
		/***********************************/
		/* ADC_12V_DEV1,                   */
		/***********************************/
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_ADC_12V_VOL_M2B, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x01, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_AUTO_RE_ARM | IPMI_SDR_SENSOR_CAP_HYSTERESIS_NO |
			IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_BIT, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_VOLTAGE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		IPMI_SDR_ASSERT_MASK_LCT_LO, // assert event mask
		IPMI_SDR_CMP_RETURN_LCT |
			IPMI_SDR_ASSERT_MASK_UCT_HI, // assert threshold reading mask
		IPMI_SDR_DEASSERT_MASK_LCT_LO, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT |
			IPMI_SDR_DEASSERT_MASK_UCT_HI, // deassert threshold reading mask
		IPMI_SDR_LCT_READABLE | IPMI_SDR_UCT_READABLE, // discrete reading mask/ settable
		IPMI_SDR_LCT_SETTABLE |
			IPMI_SDR_UCT_SETTABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_VOL, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		ADC_12V_SEN_FACTOR_M, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		ADC_12V_SEN_FACTOR_RB, // Rexp, Bexp
		0x00, // analog characteristic
		0xBB, // nominal reading
		0xCA, // normal maximum
		0x98, // normal minimum
		0xFF, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		ADC_12V_UCT, // UCT
		0x00, // UNCT
		0x00, // LNRT
		ADC_12V_LCT, // LCT
		0x00, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"VF_E1S_SSD1_12V_ADC_VOLT_V",
	},
	{
		/***********************************/
		/* ADC_12V_DEV2,                   */
		/***********************************/
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_ADC_12V_VOL_M2C, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x01, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_AUTO_RE_ARM | IPMI_SDR_SENSOR_CAP_HYSTERESIS_NO |
			IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_BIT, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_VOLTAGE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		IPMI_SDR_ASSERT_MASK_LCT_LO, // assert event mask
		IPMI_SDR_CMP_RETURN_LCT |
			IPMI_SDR_ASSERT_MASK_UCT_HI, // assert threshold reading mask
		IPMI_SDR_DEASSERT_MASK_LCT_LO, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT |
			IPMI_SDR_DEASSERT_MASK_UCT_HI, // deassert threshold reading mask
		IPMI_SDR_LCT_READABLE | IPMI_SDR_UCT_READABLE, // discrete reading mask/ settable
		IPMI_SDR_LCT_SETTABLE |
			IPMI_SDR_UCT_SETTABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_VOL, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		ADC_12V_SEN_FACTOR_M, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		ADC_12V_SEN_FACTOR_RB, // Rexp, Bexp
		0x00, // analog characteristic
		0xBB, // nominal reading
		0xCA, // normal maximum
		0x98, // normal minimum
		0xFF, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		ADC_12V_UCT, // UCT
		0x00, // UNCT
		0x00, // LNRT
		ADC_12V_LCT, // LCT
		0x00, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"VF_E1S_SSD2_12V_ADC_VOLT_V",
	},
	{
		/***********************************/
		/* ADC_12V_DEV3,                   */
		/***********************************/
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_ADC_12V_VOL_M2D, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x01, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_AUTO_RE_ARM | IPMI_SDR_SENSOR_CAP_HYSTERESIS_NO |
			IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_BIT, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_VOLTAGE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		IPMI_SDR_ASSERT_MASK_LCT_LO, // assert event mask
		IPMI_SDR_CMP_RETURN_LCT |
			IPMI_SDR_ASSERT_MASK_UCT_HI, // assert threshold reading mask
		IPMI_SDR_DEASSERT_MASK_LCT_LO, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT |
			IPMI_SDR_DEASSERT_MASK_UCT_HI, // deassert threshold reading mask
		IPMI_SDR_LCT_READABLE | IPMI_SDR_UCT_READABLE, // discrete reading mask/ settable
		IPMI_SDR_LCT_SETTABLE |
			IPMI_SDR_UCT_SETTABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_VOL, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		ADC_12V_SEN_FACTOR_M, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		ADC_12V_SEN_FACTOR_RB, // Rexp, Bexp
		0x00, // analog characteristic
		0xBB, // nominal reading
		0xCA, // normal maximum
		0x98, // normal minimum
		0xFF, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		ADC_12V_UCT, // UCT
		0x00, // UNCT
		0x00, // LNRT
		ADC_12V_LCT, // LCT
		0x00, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"VF_E1S_SSD3_12V_ADC_VOLT_V",
	},
	{
		/***********************************/
		/* ADC_3V3_DEV0,                   */
		/***********************************/
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_ADC_3V3_VOL_M2A, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x01, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_AUTO_RE_ARM | IPMI_SDR_SENSOR_CAP_HYSTERESIS_NO |
			IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_BIT, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_VOLTAGE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		IPMI_SDR_ASSERT_MASK_LCT_LO, // assert event mask
		IPMI_SDR_CMP_RETURN_LCT |
			IPMI_SDR_ASSERT_MASK_UCT_HI, // assert threshold reading mask
		IPMI_SDR_DEASSERT_MASK_LCT_LO, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT |
			IPMI_SDR_DEASSERT_MASK_UCT_HI, // deassert threshold reading mask
		IPMI_SDR_LCT_READABLE | IPMI_SDR_UCT_READABLE, // discrete reading mask/ settable
		IPMI_SDR_LCT_SETTABLE |
			IPMI_SDR_UCT_SETTABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_VOL, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		ADC_3V3_SEN_FACTOR_M, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		ADC_3V3_SEN_FACTOR_RB, // Rexp, Bexp
		0x00, // analog characteristic
		0xBB, // nominal reading
		0xCA, // normal maximum
		0x98, // normal minimum
		0xFF, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		ADC_3V3_UCT, // UCT
		0x00, // UNCT
		0x00, // LNRT
		ADC_3V3_LCT, // LCT
		0x00, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"VF_E1S_SSD0_3V3_ADC_VOLT_V",
	},
	{
		/***********************************/
		/* ADC_3V3_DEV1,                   */
		/***********************************/
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_ADC_3V3_VOL_M2B, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x01, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_AUTO_RE_ARM | IPMI_SDR_SENSOR_CAP_HYSTERESIS_NO |
			IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_BIT, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_VOLTAGE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		IPMI_SDR_ASSERT_MASK_LCT_LO, // assert event mask
		IPMI_SDR_CMP_RETURN_LCT |
			IPMI_SDR_ASSERT_MASK_UCT_HI, // assert threshold reading mask
		IPMI_SDR_DEASSERT_MASK_LCT_LO, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT |
			IPMI_SDR_DEASSERT_MASK_UCT_HI, // deassert threshold reading mask
		IPMI_SDR_LCT_READABLE | IPMI_SDR_UCT_READABLE, // discrete reading mask/ settable
		IPMI_SDR_LCT_SETTABLE |
			IPMI_SDR_UCT_SETTABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_VOL, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		ADC_3V3_SEN_FACTOR_M, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		ADC_3V3_SEN_FACTOR_RB, // Rexp, Bexp
		0x00, // analog characteristic
		0xBB, // nominal reading
		0xCA, // normal maximum
		0x98, // normal minimum
		0xFF, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		ADC_3V3_UCT, // UCT
		0x00, // UNCT
		0x00, // LNRT
		ADC_3V3_LCT, // LCT
		0x00, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"VF_E1S_SSD1_3V3_ADC_VOLT_V",
	},
	{
		/***********************************/
		/* ADC_3V3_DEV2,                   */
		/***********************************/
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_ADC_3V3_VOL_M2C, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x01, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_AUTO_RE_ARM | IPMI_SDR_SENSOR_CAP_HYSTERESIS_NO |
			IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_BIT, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_VOLTAGE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		IPMI_SDR_ASSERT_MASK_LCT_LO, // assert event mask
		IPMI_SDR_CMP_RETURN_LCT |
			IPMI_SDR_ASSERT_MASK_UCT_HI, // assert threshold reading mask
		IPMI_SDR_DEASSERT_MASK_LCT_LO, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT |
			IPMI_SDR_DEASSERT_MASK_UCT_HI, // deassert threshold reading mask
		IPMI_SDR_LCT_READABLE | IPMI_SDR_UCT_READABLE, // discrete reading mask/ settable
		IPMI_SDR_LCT_SETTABLE |
			IPMI_SDR_UCT_SETTABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_VOL, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		ADC_3V3_SEN_FACTOR_M, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		ADC_3V3_SEN_FACTOR_RB, // Rexp, Bexp
		0x00, // analog characteristic
		0xBB, // nominal reading
		0xCA, // normal maximum
		0x98, // normal minimum
		0xFF, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		ADC_3V3_UCT, // UCT
		0x00, // UNCT
		0x00, // LNRT
		ADC_3V3_LCT, // LCT
		0x00, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"VF_E1S_SSD2_3V3_ADC_VOLT_V",
	},
	{
		/***********************************/
		/* ADC_3V3_DEV3,                   */
		/***********************************/
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_ADC_3V3_VOL_M2D, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x01, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_AUTO_RE_ARM | IPMI_SDR_SENSOR_CAP_HYSTERESIS_NO |
			IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_BIT, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_VOLTAGE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		IPMI_SDR_ASSERT_MASK_LCT_LO, // assert event mask
		IPMI_SDR_CMP_RETURN_LCT |
			IPMI_SDR_ASSERT_MASK_UCT_HI, // assert threshold reading mask
		IPMI_SDR_DEASSERT_MASK_LCT_LO, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT |
			IPMI_SDR_DEASSERT_MASK_UCT_HI, // deassert threshold reading mask
		IPMI_SDR_LCT_READABLE | IPMI_SDR_UCT_READABLE, // discrete reading mask/ settable
		IPMI_SDR_LCT_SETTABLE |
			IPMI_SDR_UCT_SETTABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_VOL, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		ADC_3V3_SEN_FACTOR_M, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		ADC_3V3_SEN_FACTOR_RB, // Rexp, Bexp
		0x00, // analog characteristic
		0xBB, // nominal reading
		0xCA, // normal maximum
		0x98, // normal minimum
		0xFF, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		ADC_3V3_UCT, // UCT
		0x00, // UNCT
		0x00, // LNRT
		ADC_3V3_LCT, // LCT
		0x00, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"VF_E1S_SSD3_3V3_ADC_VOLT_V",
	},
#if HSC_SENSOR
	{
		/**********************/
		/* HSC OUTPUT CURRENT */
		/**********************/
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_I_HSC_OUT, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x01, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_AUTO_RE_ARM | IPMI_SDR_SENSOR_CAP_HYSTERESIS_NO |
			IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_BIT, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_CURRENT, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		0x00, // assert event mask
		IPMI_SDR_ASSERT_MASK_UCT_HI, // assert threshold reading mask
		0x00, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT |
			IPMI_SDR_DEASSERT_MASK_UCT_HI, // deassert threshold reading mask
		IPMI_SDR_UCT_READABLE, // discrete reading mask/ settable
		IPMI_SDR_UCT_SETTABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_AMP, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		HSC_IOUT_SEN_FACTOR_M, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		HSC_IOUT_SEN_FACTOR_EXP_RB, // Rexp, Bexp
		0x00, // analog characteristic
		0x23, // nominal reading
		0xCC, // normal maximum
		0x00, // normal minimum
		0xFF, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		0xFA, // UCT
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
		"E1S HSC IOUT",
	},
	{
		/*********************/
		/* HSC INPUT POWER   */
		/*********************/
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_P_HSC_IN, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x01, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_AUTO_RE_ARM | IPMI_SDR_SENSOR_CAP_HYSTERESIS_NO |
			IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_BIT, // sensor capabilities
		0x0B, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		0x00, // assert event mask
		IPMI_SDR_ASSERT_MASK_UCT_HI, // assert threshold reading mask
		0x00, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT |
			IPMI_SDR_DEASSERT_MASK_UCT_HI, // deassert threshold reading mask
		IPMI_SDR_UCT_READABLE, // discrete reading mask/ settable
		IPMI_SDR_UCT_SETTABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_WATT, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		HSC_PIN_SEN_FACTOR_M, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		HSC_PIN_SEN_FACTOR_EXP_RB, // Rexp, Bexp
		0x00, // analog characteristic
		0x23, // nominal reading
		0xA0, // normal maximum
		0x00, // normal minimum
		0xFF, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		0xFA, // UCT
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
		"E1S HSC PIN",
	},
#endif
	{
		/*********************/
		/* INA231 POWER M.2A */
		/*********************/
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_INA231_PWR_M2A, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x01, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_AUTO_RE_ARM | IPMI_SDR_SENSOR_CAP_HYSTERESIS_NO |
			IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_BIT, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_OTHER_UNIT_BASE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		0x00, // assert event mask
		IPMI_SDR_ASSERT_MASK_UCT_HI, // assert threshold reading mask
		0x00, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT |
			IPMI_SDR_DEASSERT_MASK_UCT_HI, // deassert threshold reading mask
		IPMI_SDR_UCT_READABLE, // discrete reading mask/ settable
		IPMI_SDR_UCT_SETTABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_WATT, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		0x01, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		0xF0, // Rexp, Bexp
		0x00, // analog characteristic
		0x20, // nominal reading
		0x5C, // normal maximum
		0x01, // normal minimum
		0xFF, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		0x96, // UCT
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
		"VF_E1S_SSD0_PWR_WATT_W",
	},
	{
		/*********************/
		/* INA231 POWER M.2B */
		/*********************/
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_INA231_PWR_M2B, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x01, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_AUTO_RE_ARM | IPMI_SDR_SENSOR_CAP_HYSTERESIS_NO |
			IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_BIT, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_OTHER_UNIT_BASE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		0x00, // assert event mask
		IPMI_SDR_ASSERT_MASK_UCT_HI, // assert threshold reading mask
		0x00, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT |
			IPMI_SDR_DEASSERT_MASK_UCT_HI, // deassert threshold reading mask
		IPMI_SDR_UCT_READABLE, // discrete reading mask/ settable
		IPMI_SDR_UCT_SETTABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_WATT, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		0x01, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		0xF0, // Rexp, Bexp
		0x00, // analog characteristic
		0x20, // nominal reading
		0x5C, // normal maximum
		0x01, // normal minimum
		0xFF, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		0x96, // UCT
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
		"VF_E1S_SSD1_PWR_WATT_W",
	},
	{
		/*********************/
		/* INA231 POWER M.2C */
		/*********************/
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_INA231_PWR_M2C, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x01, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_AUTO_RE_ARM | IPMI_SDR_SENSOR_CAP_HYSTERESIS_NO |
			IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_BIT, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_OTHER_UNIT_BASE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		0x00, // assert event mask
		IPMI_SDR_ASSERT_MASK_UCT_HI, // assert threshold reading mask
		0x00, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT |
			IPMI_SDR_DEASSERT_MASK_UCT_HI, // deassert threshold reading mask
		IPMI_SDR_UCT_READABLE, // discrete reading mask/ settable
		IPMI_SDR_UCT_SETTABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_WATT, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		0x01, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		0xF0, // Rexp, Bexp
		0x00, // analog characteristic
		0x20, // nominal reading
		0x5C, // normal maximum
		0x01, // normal minimum
		0xFF, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		0x96, // UCT
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
		"VF_E1S_SSD2_PWR_WATT_W",
	},
	{
		/*********************/
		/* INA231 POWER M.2D */
		/*********************/
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_INA231_PWR_M2D, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x01, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_AUTO_RE_ARM | IPMI_SDR_SENSOR_CAP_HYSTERESIS_NO |
			IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_BIT, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_OTHER_UNIT_BASE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		0x00, // assert event mask
		IPMI_SDR_ASSERT_MASK_UCT_HI, // assert threshold reading mask
		0x00, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT |
			IPMI_SDR_DEASSERT_MASK_UCT_HI, // deassert threshold reading mask
		IPMI_SDR_UCT_READABLE, // discrete reading mask/ settable
		IPMI_SDR_UCT_SETTABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_WATT, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		0x01, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		0xF0, // Rexp, Bexp
		0x00, // analog characteristic
		0x20, // nominal reading
		0x5C, // normal maximum
		0x01, // normal minimum
		0xFF, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		0x96, // UCT
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
		"VF_E1S_SSD3_PWR_WATT_W",
	},
	{
		/***********************************/
		/* INA231 VOLTAGE M.2A             */
		/***********************************/
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_INA231_VOL_M2A, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x01, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_AUTO_RE_ARM | IPMI_SDR_SENSOR_CAP_HYSTERESIS_NO |
			IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_BIT, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_VOLTAGE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		IPMI_SDR_ASSERT_MASK_LCT_LO, // assert event mask
		IPMI_SDR_ASSERT_MASK_UCT_HI |
			IPMI_SDR_CMP_RETURN_LCT, // assert threshold reading mask
		IPMI_SDR_DEASSERT_MASK_LCT_LO, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT |
			IPMI_SDR_DEASSERT_MASK_UCT_HI, // deassert threshold reading mask
		IPMI_SDR_LCT_READABLE | IPMI_SDR_UCT_READABLE, // discrete reading mask/ settable
		IPMI_SDR_LCT_SETTABLE |
			IPMI_SDR_UCT_SETTABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_VOL, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		INA231_VOL_SEN_FACTOR_M, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		INA231_VOL_SEN_FACTOR_RB, // Rexp, Bexp
		0x00, // analog characteristic
		0xA7, // nominal reading
		0xB4, // normal maximum
		0xA5, // normal minimum
		0xFF, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		INA231_VOL_UCT, // UCT
		0x00, // UNCT
		0x00, // LNRT
		INA231_VOL_LCT, // LCT
		0x00, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"VF_E1S_SSD0_VOLT_V",
	},
	{
		/***********************************/
		/* INA231 VOLTAGE M.2B             */
		/***********************************/
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_INA231_VOL_M2B, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x01, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_AUTO_RE_ARM | IPMI_SDR_SENSOR_CAP_HYSTERESIS_NO |
			IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_BIT, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_VOLTAGE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		IPMI_SDR_ASSERT_MASK_LCT_LO, // assert event mask
		IPMI_SDR_ASSERT_MASK_UCT_HI |
			IPMI_SDR_CMP_RETURN_LCT, // assert threshold reading mask
		IPMI_SDR_DEASSERT_MASK_LCT_LO, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT |
			IPMI_SDR_DEASSERT_MASK_UCT_HI, // deassert threshold reading mask
		IPMI_SDR_LCT_READABLE | IPMI_SDR_UCT_READABLE, // discrete reading mask/ settable
		IPMI_SDR_LCT_SETTABLE |
			IPMI_SDR_UCT_SETTABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_VOL, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		INA231_VOL_SEN_FACTOR_M, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		INA231_VOL_SEN_FACTOR_RB, // Rexp, Bexp
		0x00, // analog characteristic
		0xA7, // nominal reading
		0xB4, // normal maximum
		0xA5, // normal minimum
		0xFF, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		INA231_VOL_UCT, // UCT
		0x00, // UNCT
		0x00, // LNRT
		INA231_VOL_LCT, // LCT
		0x00, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"VF_E1S_SSD1_VOLT_V",
	},
	{
		/***********************************/
		/* INA231 VOLTAGE M.2C             */
		/***********************************/
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_INA231_VOL_M2C, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x01, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_AUTO_RE_ARM | IPMI_SDR_SENSOR_CAP_HYSTERESIS_NO |
			IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_BIT, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_VOLTAGE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		IPMI_SDR_ASSERT_MASK_LCT_LO, // assert event mask
		IPMI_SDR_ASSERT_MASK_UCT_HI |
			IPMI_SDR_CMP_RETURN_LCT, // assert threshold reading mask
		IPMI_SDR_DEASSERT_MASK_LCT_LO, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT |
			IPMI_SDR_DEASSERT_MASK_UCT_HI, // deassert threshold reading mask
		IPMI_SDR_LCT_READABLE | IPMI_SDR_UCT_READABLE, // discrete reading mask/ settable
		IPMI_SDR_LCT_SETTABLE |
			IPMI_SDR_UCT_SETTABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_VOL, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		INA231_VOL_SEN_FACTOR_M, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		INA231_VOL_SEN_FACTOR_RB, // Rexp, Bexp
		0x00, // analog characteristic
		0xA7, // nominal reading
		0xB4, // normal maximum
		0xA5, // normal minimum
		0xFF, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		INA231_VOL_UCT, // UCT
		0x00, // UNCT
		0x00, // LNRT
		INA231_VOL_LCT, // LCT
		0x00, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"VF_E1S_SSD2_VOLT_V",
	},
	{
		/***********************************/
		/* INA231 VOLTAGE M.2D             */
		/***********************************/
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_INA231_VOL_M2D, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x01, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_AUTO_RE_ARM | IPMI_SDR_SENSOR_CAP_HYSTERESIS_NO |
			IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_BIT, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_VOLTAGE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		IPMI_SDR_ASSERT_MASK_LCT_LO, // assert event mask
		IPMI_SDR_ASSERT_MASK_UCT_HI |
			IPMI_SDR_CMP_RETURN_LCT, // assert threshold reading mask
		IPMI_SDR_DEASSERT_MASK_LCT_LO, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT |
			IPMI_SDR_DEASSERT_MASK_UCT_HI, // deassert threshold reading mask
		IPMI_SDR_LCT_READABLE | IPMI_SDR_UCT_READABLE, // discrete reading mask/ settable
		IPMI_SDR_LCT_SETTABLE |
			IPMI_SDR_UCT_SETTABLE, // threshold mask/ readable threshold mask
		0x00, // sensor unit
		IPMI_SENSOR_UNIT_VOL, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		INA231_VOL_SEN_FACTOR_M, // [7:0] M bits
		0x00, // [9:8] M bits, tolerance
		0x00, // [7:0] B bits
		0x00, // [9:8] B bits, tolerance
		0x00, // [7:4] accuracy , [3:2] accuracy exp, [1:0] sensor direction
		INA231_VOL_SEN_FACTOR_RB, // Rexp, Bexp
		0x00, // analog characteristic
		0xA7, // nominal reading
		0xB4, // normal maximum
		0xA5, // normal minimum
		0xFF, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		INA231_VOL_UCT, // UCT
		0x00, // UNCT
		0x00, // LNRT
		INA231_VOL_LCT, // LCT
		0x00, // LNCT
		0x00, // positive-going threshold
		0x00, // negative-going threshold
		0x00, // reserved
		0x00, // reserved
		0x00, // OEM
		IPMI_SDR_STRING_TYPE_ASCII_8, // ID len, should be same as "size of struct"
		"VF_E1S_SSD3_VOLT_V",
	},
	{
		/***********************************/
		/* NVME SSD TEMPERATURE M.2A       */
		/***********************************/
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_NVME_TEMP_M2A, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x01, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_AUTO_RE_ARM | IPMI_SDR_SENSOR_CAP_HYSTERESIS_NO |
			IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_BIT, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_TEMPERATURE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		0x00, // assert event mask
		IPMI_SDR_ASSERT_MASK_UCT_HI, // assert threshold reading mask
		0x00, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT |
			IPMI_SDR_DEASSERT_MASK_UCT_HI, // deassert threshold reading mask
		IPMI_SDR_UCT_READABLE, // discrete reading mask/ settable
		IPMI_SDR_UCT_SETTABLE, // threshold mask/ readable threshold mask
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
		0x23, // nominal reading
		0x00, // normal maximum
		0x00, // normal minimum
		0xFF, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		0x4C, // UCT
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
		"VF_E1S_SSD0_TEMP_C",
	},
	{
		/***********************************/
		/* NVME SSD TEMPERATURE M.2B       */
		/***********************************/
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_NVME_TEMP_M2B, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x01, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_AUTO_RE_ARM | IPMI_SDR_SENSOR_CAP_HYSTERESIS_NO |
			IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_BIT, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_TEMPERATURE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		0x00, // assert event mask
		IPMI_SDR_ASSERT_MASK_UCT_HI, // assert threshold reading mask
		0x00, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT |
			IPMI_SDR_DEASSERT_MASK_UCT_HI, // deassert threshold reading mask
		IPMI_SDR_UCT_READABLE, // discrete reading mask/ settable
		IPMI_SDR_UCT_SETTABLE, // threshold mask/ readable threshold mask
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
		0x23, // nominal reading
		0x00, // normal maximum
		0x00, // normal minimum
		0xFF, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		0x4C, // UCT
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
		"VF_E1S_SSD1_TEMP_C",
	},
	{
		/***********************************/
		/* NVME SSD TEMPERATURE M.2C       */
		/***********************************/
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_NVME_TEMP_M2C, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x01, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_AUTO_RE_ARM | IPMI_SDR_SENSOR_CAP_HYSTERESIS_NO |
			IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_BIT, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_TEMPERATURE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		0x00, // assert event mask
		IPMI_SDR_ASSERT_MASK_UCT_HI, // assert threshold reading mask
		0x00, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT |
			IPMI_SDR_DEASSERT_MASK_UCT_HI, // deassert threshold reading mask
		IPMI_SDR_UCT_READABLE, // discrete reading mask/ settable
		IPMI_SDR_UCT_SETTABLE, // threshold mask/ readable threshold mask
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
		0x23, // nominal reading
		0x00, // normal maximum
		0x00, // normal minimum
		0xFF, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		0x4C, // UCT
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
		"VF_E1S_SSD2_TEMP_C",
	},
	{
		/***********************************/
		/* NVME SSD TEMPERATURE M.2D       */
		/***********************************/
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_NVME_TEMP_M2D, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x01, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_AUTO_RE_ARM | IPMI_SDR_SENSOR_CAP_HYSTERESIS_NO |
			IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_BIT, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_TEMPERATURE, // sensor type
		IPMI_SDR_EVENT_TYPE_THRESHOLD, // event/reading type
		0x00, // assert event mask
		IPMI_SDR_ASSERT_MASK_UCT_HI, // assert threshold reading mask
		0x00, // deassert event mask
		IPMI_SDR_CMP_RETURN_UCT |
			IPMI_SDR_DEASSERT_MASK_UCT_HI, // deassert threshold reading mask
		IPMI_SDR_UCT_READABLE, // discrete reading mask/ settable
		IPMI_SDR_UCT_SETTABLE, // threshold mask/ readable threshold mask
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
		0x23, // nominal reading
		0x00, // normal maximum
		0x00, // normal minimum
		0xFF, // sensor maximum reading
		0x00, // sensor minimum reading
		0x00, // UNRT
		0x4C, // UCT
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
		"VF_E1S_SSD3_TEMP_C",
	},
};

typedef struct {
	uint8_t sensor_num;
	uint8_t ID_str[MAX_SDR_SENSOR_NAME_LEN];
} yv3_change_sdr;

// yv3 platform re-set sensor name in standard get sensor reading
yv3_change_sdr sdr_table_in_yv3[] = {
	{ SENSOR_NUM_T_MB_OUTLET_TEMP_T, "E1S Outlet Temp" },
	{ SENSOR_NUM_V_12_AUX, "E1S P12V_AUX" },
	{ SENSOR_NUM_V_12_EDGE, "E1S P12V_EDGE" },
	{ SENSOR_NUM_V_3_3_AUX, "E1S P3V3_AUX" },
	{ SENSOR_NUM_ADC_12V_VOL_M2A, "E1S 12V_ADC_DEV0" },
	{ SENSOR_NUM_ADC_12V_VOL_M2B, "E1S 12V_ADC_DEV1" },
	{ SENSOR_NUM_ADC_12V_VOL_M2C, "E1S 12V_ADC_DEV2" },
	{ SENSOR_NUM_ADC_12V_VOL_M2D, "E1S 12V_ADC_DEV3" },
	{ SENSOR_NUM_ADC_3V3_VOL_M2A, "E1S 3V3_ADC_DEV0" },
	{ SENSOR_NUM_ADC_3V3_VOL_M2B, "E1S 3V3_ADC_DEV1" },
	{ SENSOR_NUM_ADC_3V3_VOL_M2C, "E1S 3V3_ADC_DEV2" },
	{ SENSOR_NUM_ADC_3V3_VOL_M2D, "E1S 3V3_ADC_DEV3" },
	{ SENSOR_NUM_INA231_PWR_M2A, "E1S DEV0 Power" },
	{ SENSOR_NUM_INA231_PWR_M2B, "E1S DEV1 Power" },
	{ SENSOR_NUM_INA231_PWR_M2C, "E1S DEV2 Power" },
	{ SENSOR_NUM_INA231_PWR_M2D, "E1S DEV3 Power" },
	{ SENSOR_NUM_INA231_VOL_M2A, "E1S DEV0 Voltage" },
	{ SENSOR_NUM_INA231_VOL_M2B, "E1S DEV1 Voltage" },
	{ SENSOR_NUM_INA231_VOL_M2C, "E1S DEV2 Voltage" },
	{ SENSOR_NUM_INA231_VOL_M2D, "E1S DEV3 Voltage" },
	{ SENSOR_NUM_NVME_TEMP_M2A, "E1S DEV0 Temp" },
	{ SENSOR_NUM_NVME_TEMP_M2B, "E1S DEV1 Temp" },
	{ SENSOR_NUM_NVME_TEMP_M2C, "E1S DEV2 Temp" },
	{ SENSOR_NUM_NVME_TEMP_M2D, "E1S DEV3 Temp" },
};

void pal_set_SDR(uint8_t *table_ptr)
{
	CHECK_NULL_ARG(table_ptr);
	SDR_Full_sensor *sdr_ptr = (SDR_Full_sensor *)table_ptr;
	yv3_change_sdr *change_sdr_ptr = sdr_table_in_yv3;

	uint8_t i;
	for (i = 0; i < ARRAY_SIZE(sdr_table_in_yv3); i++) {
		if (sdr_ptr->sensor_num == (change_sdr_ptr + i)->sensor_num)
			snprintf(sdr_ptr->ID_str, sizeof(sdr_ptr->ID_str), "%s",
				 (change_sdr_ptr + i)->ID_str);
	}
}

const int SDR_TABLE_SIZE = ARRAY_SIZE(plat_sdr_table);