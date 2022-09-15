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

#include <stdint.h>
#include <pmbus.h>

/*  define config for sensors  */
#define TMP75_ADDR (0x9A >> 1)
#define HSC_ADDR (0x26 >> 1)
#define NVME_ADDR (0xD4 >> 1)

#define I2C_ADDR_M2_INA231 (0x8A >> 1)
#define I2C_ADDR_M2_ISL28022 (0x8A >> 1)
#define I2C_ADDR_M2_FPGA (0x80 >> 1)
#define I2C_ADDR_M2_DDR (0x88 >> 1)
#define I2C_ADDR_M2_TMP75 (0x9C >> 1)
#define I2C_ADDR_M2_EEPROM (0xA0 >> 1)

#define INA231_POWER_REG 0x03
#define INA231_BUS_VOLTAGE_REG 0x02
#define NVME_TEMP_REG 0x00

/* ISL28022 ReadType define region start */
#define ISL28022_CONFIG_REG 0
#define ISL28022_BUS_VOLTAGE_REG 2
#define ISL28022_POWER_REG 3
#define ISL28022_CURRENT_REG 4
#define ISL28022_CALIBRATION_REG 5

/*  define config for sensors  */
#define ASIC_OFFSET 0x00 // TBU
#define TMP75_TEMP_OFFSET 0x00
#define INA233_CALIBRATION_OFFSET 0xD4
#define VR_PAGE_OFFSET 0x00
#define SMBUS_VOL_CMD 0x8B
#define SMBUS_CUR_CMD 0x8C
#define SMBUS_TEMP_CMD 0x8D
#define SMBUS_PWR_CMD 0x96

// #TMP

////////////////////////////////////////////////////////////////////////////////////
//  ADM1278 ReadType define region Start

#define ADM1278_VIN_REG PMBUS_READ_VIN
#define ADM1278_IOUT_REG PMBUS_READ_IOUT
#define ADM1278_TEMP_REG PMBUS_READ_TEMPERATURE_1
#define ADM1278_PIN_REG PMBUS_READ_PIN
#define ADM1278_PMON_CONF_REG 0xD4

#define ADM1278_PMON_CONF_REG_TEMP1_EN 0x08
#define ADM1278_PMON_CONF_REG_PWR_AVG 0x38

/* sensor number */
/*************************************************************************************************/
/************************************** temperature **********************************************/
/*************************************************************************************************/
#define SENSOR_NUM_T_MB_OUTLET_TEMP_T 0x50

/*************************************************************************************************/
/************************************* adc voltage ***********************************************/
/*************************************************************************************************/
#define SENSOR_NUM_V_12_AUX 0x51
#define SENSOR_NUM_V_12_EDGE 0x52
#define SENSOR_NUM_V_3_3_AUX 0x53
#define SENSOR_NUM_V_1_2_STBY 0x58

/*************************************************************************************************/
/*************************************      HSC    ***********************************************/
/*************************************************************************************************/
#define SENSOR_NUM_V_HSC_IN 0x54
#define SENSOR_NUM_I_HSC_OUT 0x55
#define SENSOR_NUM_P_HSC_IN 0x56
#define SENSOR_NUM_T_HSC 0x57
/*************************************************************************************************/
/***************************************** M.2 ***************************************************/
/*************************************************************************************************/
/* sensor number prefix of M.2 devices */
#define PREFIX_MASK 0xF8

#define PREFIX_M2A 0x60
#define PREFIX_M2B 0x68
#define PREFIX_M2C 0x70
#define PREFIX_M2D 0x78
#define PREFIX_M2E 0xFF
#define PREFIX_M2F 0xFF

/* sensor number suffix of M.2 devices */
#define SUFFIX_INA231_PWR 0x00
#define SUFFIX_INA231_VOL 0x01
#define SUFFIX_NVME_TEMP 0x02
#define SUFFIX_ADC_12V_VOL 0x03
#define SUFFIX_ADC_3V3_VOL 0x04

#define SENSOR_NUM_INA231_PWR_M2A (PREFIX_M2A | SUFFIX_INA231_PWR)
#define SENSOR_NUM_INA231_VOL_M2A (PREFIX_M2A | SUFFIX_INA231_VOL)
#define SENSOR_NUM_NVME_TEMP_M2A (PREFIX_M2A | SUFFIX_NVME_TEMP)
#define SENSOR_NUM_ADC_3V3_VOL_M2A (PREFIX_M2A | SUFFIX_ADC_3V3_VOL)
#define SENSOR_NUM_ADC_12V_VOL_M2A (PREFIX_M2A | SUFFIX_ADC_12V_VOL)

#define SENSOR_NUM_INA231_PWR_M2B (PREFIX_M2B | SUFFIX_INA231_PWR)
#define SENSOR_NUM_INA231_VOL_M2B (PREFIX_M2B | SUFFIX_INA231_VOL)
#define SENSOR_NUM_NVME_TEMP_M2B (PREFIX_M2B | SUFFIX_NVME_TEMP)
#define SENSOR_NUM_ADC_3V3_VOL_M2B (PREFIX_M2B | SUFFIX_ADC_3V3_VOL)
#define SENSOR_NUM_ADC_12V_VOL_M2B (PREFIX_M2B | SUFFIX_ADC_12V_VOL)

#define SENSOR_NUM_INA231_PWR_M2C (PREFIX_M2C | SUFFIX_INA231_PWR)
#define SENSOR_NUM_INA231_VOL_M2C (PREFIX_M2C | SUFFIX_INA231_VOL)
#define SENSOR_NUM_NVME_TEMP_M2C (PREFIX_M2C | SUFFIX_NVME_TEMP)
#define SENSOR_NUM_ADC_3V3_VOL_M2C (PREFIX_M2C | SUFFIX_ADC_3V3_VOL)
#define SENSOR_NUM_ADC_12V_VOL_M2C (PREFIX_M2C | SUFFIX_ADC_12V_VOL)

#define SENSOR_NUM_INA231_PWR_M2D (PREFIX_M2D | SUFFIX_INA231_PWR)
#define SENSOR_NUM_INA231_VOL_M2D (PREFIX_M2D | SUFFIX_INA231_VOL)
#define SENSOR_NUM_NVME_TEMP_M2D (PREFIX_M2D | SUFFIX_NVME_TEMP)
#define SENSOR_NUM_ADC_3V3_VOL_M2D (PREFIX_M2D | SUFFIX_ADC_3V3_VOL)
#define SENSOR_NUM_ADC_12V_VOL_M2D (PREFIX_M2D | SUFFIX_ADC_12V_VOL)

#define SENSOR_NUM_INA231_PWR_M2E (PREFIX_M2E | SUFFIX_INA231_PWR)
#define SENSOR_NUM_INA231_VOL_M2E (PREFIX_M2E | SUFFIX_INA231_VOL)
#define SENSOR_NUM_NVME_TEMP_M2E (PREFIX_M2E | SUFFIX_NVME_TEMP)

#define SENSOR_NUM_INA231_PWR_M2F (PREFIX_M2F | SUFFIX_INA231_PWR)
#define SENSOR_NUM_INA231_VOL_M2F (PREFIX_M2F | SUFFIX_INA231_VOL)
#define SENSOR_NUM_NVME_TEMP_M2F (PREFIX_M2F | SUFFIX_NVME_TEMP)

uint8_t plat_get_config_size();
void load_sensor_config(void);
#endif
