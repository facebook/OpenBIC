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
#include "sensor.h"
#include "plat_def.h"
#include "plat_class.h"
#include "plat_ipmb.h"
#include "plat_sensor_table.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(plat_sdr_table);

SDR_Full_sensor plat_sdr_table[] = {
	/* =============================== TEMPARUTURE SENSOR =============================== */
	{
		// TMP451_IN temperature
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_TEMP_TMP451_IN, // sensor number

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
		0x00, // [7:0] M bits
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
		"MB_INLET_TEMP_C",
	},
	{
		// TMP451_OUT temperature
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_TEMP_TMP451_OUT, // sensor number

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
		0x00, // [7:0] M bits
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
		"MB_OUTLET_TEMP_C",
	},
	{
		// TMP75_FIO temperature
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_TEMP_TMP75_FIO, // sensor number

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
		0x00, // [7:0] M bits
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
		"MB_FIO_FRONT_TEMP_C",
	},
#ifdef ENABLE_NVIDIA
	{
		// CPU temperature
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
		0x00, // [7:0] M bits
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
		"MB_SOC_CPU_TEMP_C",
	},
#endif
	{
		// FPGA temperature
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_TEMP_FPGA, // sensor number

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
		0x00, // [7:0] M bits
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
		"MB_FPGA_TEMP_C",
	},
	{
		// E1S_SSD temperature
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_TEMP_E1S_SSD, // sensor number

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
		0x00, // [7:0] M bits
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
		"MB_E1S_SSD_TEMP_C",
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
		0x50, // UCT
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
		"MB_HSC_TEMP_C",
	},

	/* =============================== VOLTAGE SENSOR =============================== */
	{
		// HSC Input Voltage
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
		IPMI_SDR_SENSOR_TYPE_VOLTAGE, // sensor type
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
		0x00, // [7:0] M bits
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
		"MB_HSC_INPUT_VOLT_V",
	},
	{
		// ADC0 - P12V_STBY Voltage
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_VOL_ADC0_P12V_STBY, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_VOLTAGE, // sensor type
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
		0x00, // [7:0] M bits
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
		"MB_ADC_P12V_STBY_VOLT_V",
	},
	{
		// ADC1 - VDD_1V8 Voltage
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_VOL_ADC1_VDD_1V8, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_VOLTAGE, // sensor type
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
		0x00, // [7:0] M bits
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
		"MB_ADC_VDD_1V8_VOLT_V",
	},
	{
		// ADC2 - P3V3_STBY Voltage
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_VOL_ADC2_P3V3_STBY, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_VOLTAGE, // sensor type
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
		0x00, // [7:0] M bits
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
		"MB_ADC_P3V3_STBY_VOLT_V",
	},
	{
		// ADC3 - SOCVDD Voltage
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_VOL_ADC3_SOCVDD, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_VOLTAGE, // sensor type
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
		0x00, // [7:0] M bits
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
		"MB_ADC_SOCVDD_VOLT_V",
	},
	{
		// ADC4 - P3V_BAT Voltage
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_VOL_ADC4_P3V_BAT, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_VOLTAGE, // sensor type
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
		0x00, // [7:0] M bits
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
		"MB_ADC_P3V_BAT_VOLT_V",
	},
	{
		// ADC5 - CPUVDD Voltage
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_VOL_ADC5_CPUVDD, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_VOLTAGE, // sensor type
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
		0x00, // [7:0] M bits
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
		"MB_ADC_CPUVDD_VOLT_V",
	},
	{
		// ADC6 - FPGA_VCC_AO Voltage
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_VOL_ADC6_FPGA_VCC_AO, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_VOLTAGE, // sensor type
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
		0x00, // [7:0] M bits
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
		"MB_ADC_FPGA_VCC_AO_VOLT_V",
	},
	{
		// ADC7 - 1V2 Voltage
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_VOL_ADC7_1V2, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_VOLTAGE, // sensor type
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
		0x00, // [7:0] M bits
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
		"MB_ADC_1V2_VOLT_V",
	},
	{
		// ADC9 - VDD_M2 Voltage
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_VOL_ADC9_VDD_M2, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_VOLTAGE, // sensor type
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
		0x00, // [7:0] M bits
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
		"MB_ADC_VDD_M2_VOLT_V",
	},
	{
		// ADC10 - P1V2_STBY Voltage
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_VOL_ADC10_P1V2_STBY, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_VOLTAGE, // sensor type
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
		0x00, // [7:0] M bits
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
		"MB_ADC_P1V2_STBY_VOLT_V",
	},
	{
		// ADC11 - FBVDDQ Voltage
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_VOL_ADC11_FBVDDQ, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_VOLTAGE, // sensor type
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
		0x00, // [7:0] M bits
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
		"MB_ADC_FBVDDQ_VOLT_V",
	},
	{
		// ADC12 - FBVDDP2 Voltage
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_VOL_ADC12_FBVDDP2, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_VOLTAGE, // sensor type
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
		0x00, // [7:0] M bits
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
		"MB_ADC_FBVDDP2_VOLT_V",
	},
	{
		// ADC13 - FBVDD1 Voltage
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_VOL_ADC13_FBVDD1, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_VOLTAGE, // sensor type
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
		0x00, // [7:0] M bits
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
		"MB_ADC_FBVDD1_VOLT_V",
	},
	{
		// ADC14 - P5V_STBY Voltage
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_VOL_ADC14_P5V_STBY, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_VOLTAGE, // sensor type
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
		0x00, // [7:0] M bits
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
		"MB_ADC_P5V_STBY_VOLT_V",
	},
	{
		// ADC15 - CPU_DVDD Voltage
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_VOL_ADC15_CPU_DVDD, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_VOLTAGE, // sensor type
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
		0x00, // [7:0] M bits
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
		"MB_ADC_CPU_DVDD_VOLT_V",
	},
	{
		// INA230 Voltage
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_VOL_E1S, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_VOLTAGE, // sensor type
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
		0x00, // [7:0] M bits
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
		"MB_E1S_VOLT_V",
	},
#ifdef ENABLE_NVIDIA
	{
		// CPU_VDD voltage
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_VOL_CPUVDD, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_VOLTAGE, // sensor type
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
		0x00, // [7:0] M bits
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
		"MB_CPUVDD_VOLT_V",
	},
	{
		// SOC_VDD voltage
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_VOL_SOCVDD, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_VOLTAGE, // sensor type
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
		0x00, // [7:0] M bits
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
		"MB_SOCVDD_VOLT_V",
	},
#endif

	/* =============================== CURRENT SENSOR =============================== */
	{
		// HSC Output Current
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
		IPMI_SDR_SENSOR_TYPE_CURRENT, // sensor type
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
		0x00, // [7:0] M bits
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
		"MB_HSC_OUTPUT_CURR_A",
	},
	{
		// INA230 E1.S Current
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_CUR_E1S, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_CURRENT, // sensor type
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
		0x00, // [7:0] M bits
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
		"MB_E1S_CURR_A",
	},

	/* =============================== POWER SENSOR =============================== */
	{
		// HSC Input Power
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
		IPMI_SDR_SENSOR_TYPE_OTHER_UNIT_BASE, // sensor type
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
		0x00, // [7:0] M bits
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
		"MB_HSC_INPUT_PWR_W",
	},
	{
		// INA230 E1.S Power
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_PWR_E1S, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_OTHER_UNIT_BASE, // sensor type
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
		0x00, // [7:0] M bits
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
		"MB_E1S_PWR_W",
	},
#ifdef ENABLE_NVIDIA
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
		IPMI_SDR_SENSOR_TYPE_OTHER_UNIT_BASE, // sensor type
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
		0x00, // [7:0] M bits
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
		"MB_CPU_PWR_W",
	},
	{
		// CPU_VDD power
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_PWR_CPUVDD, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_OTHER_UNIT_BASE, // sensor type
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
		0x00, // [7:0] M bits
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
		"MB_CPUVDD_PWR_W",
	},
	{
		// SOC_VDD power
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_PWR_SOCVDD, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_OTHER_UNIT_BASE, // sensor type
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
		0x00, // [7:0] M bits
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
		"MB_SOCVDD_PWR_W",
	},
#endif

/* =============================== STATE SENSOR =============================== */
#ifdef ENABLE_NVIDIA
	{
		// CPU THROTTLE state
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_OTH_CPU_THROTTLE, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_OTHER_UNIT_BASE, // sensor type
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
		IPMI_SENSOR_UNIT_UNSPECIFIED, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		0x00, // [7:0] M bits
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
		"MB_CPU_THROTTLE_STA",
	},
	{
		// POWER BREAK state
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_OTH_POWER_BREAK, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_OTHER_UNIT_BASE, // sensor type
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
		IPMI_SENSOR_UNIT_UNSPECIFIED, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		0x00, // [7:0] M bits
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
		"MB_PWR_BREAK_STA",
	},
	{
		// CPU THROTTLE state
		0x00,
		0x00, // record ID
		IPMI_SDR_VER_15, // SDR ver
		IPMI_SDR_FULL_SENSOR, // record type
		IPMI_SDR_FULL_SENSOR_MIN_LEN, // size of struct

		SELF_I2C_ADDRESS << 1, // owner id
		0x00, // owner lun
		SENSOR_NUM_OTH_SPARE_CHANNEL, // sensor number

		IPMI_SDR_ENTITY_ID_SYS_BOARD, // entity id
		0x00, // entity instance
		IPMI_SDR_SENSOR_INIT_SCAN | IPMI_SDR_SENSOR_INIT_EVENT |
			IPMI_SDR_SENSOR_INIT_THRESHOLD | IPMI_SDR_SENSOR_INIT_TYPE |
			IPMI_SDR_SENSOR_INIT_DEF_EVENT |
			IPMI_SDR_SENSOR_INIT_DEF_SCAN, // sensor init
		IPMI_SDR_SENSOR_CAP_THRESHOLD_RW |
			IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO, // sensor capabilities
		IPMI_SDR_SENSOR_TYPE_OTHER_UNIT_BASE, // sensor type
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
		IPMI_SENSOR_UNIT_UNSPECIFIED, // base unit
		0x00, // modifier unit
		IPMI_SDR_LINEAR_LINEAR, // linearization
		0x00, // [7:0] M bits
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
		"MB_SPARE_CH_STA",
	},
#endif
};

const int SDR_TABLE_SIZE = ARRAY_SIZE(plat_sdr_table);

void load_sdr_table(void)
{
	memcpy(full_sdr_table, plat_sdr_table, sizeof(plat_sdr_table));
	sdr_count = ARRAY_SIZE(plat_sdr_table);
}
