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

#include "sensor.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <logging/log.h>
#include "power_status.h"
#include "sdr.h"
#include "pdr.h"
#include "hal_i2c.h"
#include "plat_sensor_table.h"
#include "plat_sdr_table.h"
#include "ast_adc.h"
#include "intel_peci.h"
#include "util_sys.h"
#include "plat_def.h"
#include "libutil.h"

#include <logging/log.h>

LOG_MODULE_REGISTER(sensor);

#define SENSOR_DRIVE_INIT_DECLARE(name) uint8_t name##_init(sensor_cfg *cfg)

#define SENSOR_DRIVE_TYPE_INIT_MAP(name)                                                           \
	{                                                                                          \
		sensor_dev_##name, name##_init                                                     \
	}

#define SENSOR_DRIVE_TYPE_UNUSE(name)                                                              \
	{                                                                                          \
		sensor_dev_##name, NULL                                                            \
	}

#define SENSOR_READ_RETRY_MAX 3

extern sensor_cfg plat_sensor_config[];
extern const int SENSOR_CONFIG_SIZE;

struct k_thread sensor_poll;
K_KERNEL_STACK_MEMBER(sensor_poll_stack, SENSOR_POLL_STACK_SIZE);

uint8_t sensor_config_index_map[SENSOR_NUM_MAX];
uint8_t sdr_index_map[SENSOR_NUM_MAX];

bool enable_sensor_poll_thread = true;
static bool sensor_poll_enable_flag = true;
static bool is_sensor_initial_done = false;
static bool is_sensor_ready_flag = false;

const int negative_ten_power[16] = { 1,	    1,		1,	   1,	     1,	      1,
				     1,	    1000000000, 100000000, 10000000, 1000000, 100000,
				     10000, 1000,	100,	   10 };

sensor_cfg *sensor_config = NULL;
uint8_t sensor_config_count = 0;

sensor_monitor_table_info *sensor_monitor_table;
uint16_t sensor_monitor_count = 0;
char common_sensor_table_name[] = "common sensor table";

// clang-format off
const char *const sensor_type_name[] = {
	sensor_name_to_num(tmp75)
	sensor_name_to_num(adc)
	sensor_name_to_num(peci)
	sensor_name_to_num(isl69259)
	sensor_name_to_num(hsc)
	sensor_name_to_num(nvme)
	sensor_name_to_num(pch)
	sensor_name_to_num(mp5990)
	sensor_name_to_num(isl28022)
	sensor_name_to_num(pex89000)
	sensor_name_to_num(tps53689)
	sensor_name_to_num(xdpe15284)
	sensor_name_to_num(ltc4282)
	sensor_name_to_num(fan)
	sensor_name_to_num(tmp431)
	sensor_name_to_num(pmic)
	sensor_name_to_num(ina233)
	sensor_name_to_num(isl69254)
	sensor_name_to_num(max16550a)
	sensor_name_to_num(ina230)
	sensor_name_to_num(xdpe12284c)
	sensor_name_to_num(raa229621)
	sensor_name_to_num(nct7718w)
	sensor_name_to_num(ltc4286)
	sensor_name_to_num(amd_tsi)
	sensor_name_to_num(apml_mailbox)
	sensor_name_to_num(xdpe19283b)
	sensor_name_to_num(g788p81u)
	sensor_name_to_num(mp2856gut)
	sensor_name_to_num(ddr5_power)
	sensor_name_to_num(ddr5_temp)
	sensor_name_to_num(adm1272)
	sensor_name_to_num(q50sn120a1)
	sensor_name_to_num(mp2971)
	sensor_name_to_num(pm8702)
	sensor_name_to_num(ltc2991)
	sensor_name_to_num(sq52205)
	sensor_name_to_num(emc1412)
	sensor_name_to_num(i3c_dimm)
	sensor_name_to_num(pt5161l)
	sensor_name_to_num(lm75bd118)
	sensor_name_to_num(tmp461)
	sensor_name_to_num(mp2985)
	sensor_name_to_num(m88rt51632)
	sensor_name_to_num(mpro)
	sensor_name_to_num(bmr351)
	sensor_name_to_num(cx7)
	sensor_name_to_num(vistara)
	sensor_name_to_num(max11617)
	sensor_name_to_num(nv_satmc)
	sensor_name_to_num(nct7363)
	sensor_name_to_num(ads112c)
	sensor_name_to_num(hdc1080)
	sensor_name_to_num(ina238)
	sensor_name_to_num(nct214)
	sensor_name_to_num(ast_tach)
	sensor_name_to_num(xdp710)
	sensor_name_to_num(ds160pt801)
	sensor_name_to_num(rs31380r)
	sensor_name_to_num(adc128d818)
	sensor_name_to_num(ads1015)
	sensor_name_to_num(plat_def_sensor)
	sensor_name_to_num(e50sn12051)
};
// clang-format on

#ifndef DISABLE_TMP75
SENSOR_DRIVE_INIT_DECLARE(tmp75);
#endif
#ifndef DISABLE_AST_ADC
SENSOR_DRIVE_INIT_DECLARE(ast_adc);
#endif
#ifndef DISABLE_ISL69259
SENSOR_DRIVE_INIT_DECLARE(isl69259);
#endif
#ifndef DISABLE_NVME
SENSOR_DRIVE_INIT_DECLARE(nvme);
#endif
#ifndef DISABLE_MP5990
SENSOR_DRIVE_INIT_DECLARE(mp5990);
#endif
#ifndef DISABLE_ISL28022
SENSOR_DRIVE_INIT_DECLARE(isl28022);
#endif
#ifndef DISABLE_PEX89000
SENSOR_DRIVE_INIT_DECLARE(pex89000);
#endif
SENSOR_DRIVE_INIT_DECLARE(intel_peci);
#ifndef DISABLE_PCH
SENSOR_DRIVE_INIT_DECLARE(pch);
#endif
#ifndef DISABLE_ADM1278
SENSOR_DRIVE_INIT_DECLARE(adm1278);
#endif
#ifndef DISABLE_TPS53689
SENSOR_DRIVE_INIT_DECLARE(tps53689);
#endif
#ifndef DISABLE_XDPE15284
SENSOR_DRIVE_INIT_DECLARE(xdpe15284);
#endif
#ifndef DISABLE_LTC4282
SENSOR_DRIVE_INIT_DECLARE(ltc4282);
#endif
#ifndef DISABLE_TMP431
SENSOR_DRIVE_INIT_DECLARE(tmp431);
#endif
#ifndef DISABLE_PMIC
SENSOR_DRIVE_INIT_DECLARE(pmic);
#endif
#ifndef DISABLE_INA233
SENSOR_DRIVE_INIT_DECLARE(ina233);
#endif
#ifndef DISABLE_ISL69254IRAZ_T
SENSOR_DRIVE_INIT_DECLARE(isl69254iraz_t);
#endif
#ifndef DISABLE_MAX16550A
SENSOR_DRIVE_INIT_DECLARE(max16550a);
#endif
#ifndef DISABLE_INA230
SENSOR_DRIVE_INIT_DECLARE(ina230);
#endif
#ifndef DISABLE_XDP12284C
SENSOR_DRIVE_INIT_DECLARE(xdpe12284c);
#endif
#ifndef DISABLE_RAA229621
SENSOR_DRIVE_INIT_DECLARE(raa229621);
#endif
#ifndef DISABLE_NCT7718W
SENSOR_DRIVE_INIT_DECLARE(nct7718w);
#endif
#ifndef DISABLE_LTC4286
SENSOR_DRIVE_INIT_DECLARE(ltc4286);
#endif
#ifdef ENABLE_APML
SENSOR_DRIVE_INIT_DECLARE(amd_tsi);
SENSOR_DRIVE_INIT_DECLARE(apml_mailbox);
#endif
#ifndef DISABLE_XDPE19283B
SENSOR_DRIVE_INIT_DECLARE(xdpe19283b);
#endif
#ifndef DISABLE_G788P81U
SENSOR_DRIVE_INIT_DECLARE(g788p81u);
#endif
#ifndef DISABLE_MP2856GUT
SENSOR_DRIVE_INIT_DECLARE(mp2856gut);
#endif
#ifndef DISABLE_DDR5_POWER
SENSOR_DRIVE_INIT_DECLARE(ddr5_power);
#endif
#ifndef DISABLE_DDR5_TEMP
SENSOR_DRIVE_INIT_DECLARE(ddr5_temp);
#endif
#ifndef DISABLE_ADM1272
SENSOR_DRIVE_INIT_DECLARE(adm1272);
#endif
#ifndef DISABLE_Q50SN120A1
SENSOR_DRIVE_INIT_DECLARE(q50sn120a1);
#endif
#ifndef DISABLE_MP2971
SENSOR_DRIVE_INIT_DECLARE(mp2971);
#endif
#ifdef ENABLE_PM8702
SENSOR_DRIVE_INIT_DECLARE(pm8702);
#endif
#ifndef DISABLE_LTC2991
SENSOR_DRIVE_INIT_DECLARE(ltc2991);
#endif
#ifndef DISABLE_SQ52205
SENSOR_DRIVE_INIT_DECLARE(sq52205);
#endif
#ifndef DISABLE_EMC1412
SENSOR_DRIVE_INIT_DECLARE(emc1412);
#endif
#ifndef DISABLE_I3C_DIMM
SENSOR_DRIVE_INIT_DECLARE(i3c_dimm);
#endif
#ifndef DISABLE_PT5161L
SENSOR_DRIVE_INIT_DECLARE(pt5161l);
#endif
#ifndef DISABLE_LM75BD118
SENSOR_DRIVE_INIT_DECLARE(lm75bd118);
#endif
#ifndef DISABLE_TMP461
SENSOR_DRIVE_INIT_DECLARE(tmp461);
#endif
#ifndef DISABLE_MP2985
SENSOR_DRIVE_INIT_DECLARE(mp2985);
#endif
#ifndef DISABLE_M88RT51632
SENSOR_DRIVE_INIT_DECLARE(m88rt51632);
#endif
#ifdef ENABLE_MPRO
SENSOR_DRIVE_INIT_DECLARE(mpro);
#endif
#ifndef DISABLE_BMR351
SENSOR_DRIVE_INIT_DECLARE(bmr351);
#endif
#ifndef DISABLE_CX7
SENSOR_DRIVE_INIT_DECLARE(cx7);
#endif
#ifdef ENABLE_VISTARA
SENSOR_DRIVE_INIT_DECLARE(vistara);
#endif
#ifndef DISABLE_MAX11617
SENSOR_DRIVE_INIT_DECLARE(max11617);
#endif
#ifdef ENABLE_NVIDIA
SENSOR_DRIVE_INIT_DECLARE(nv_satmc);
#endif
#ifndef DISABLE_NCT7363
SENSOR_DRIVE_INIT_DECLARE(nct7363);
#endif
#ifndef DISABLE_ADS112C
SENSOR_DRIVE_INIT_DECLARE(ads112c);
#endif
#ifndef DISABLE_HDC1080
SENSOR_DRIVE_INIT_DECLARE(hdc1080);
#endif
#ifndef DISABLE_INA238
SENSOR_DRIVE_INIT_DECLARE(ina238);
#endif
#ifndef DISABLE_NCT214
SENSOR_DRIVE_INIT_DECLARE(nct214);
#endif
#ifndef DISABLE_AST_TACH
SENSOR_DRIVE_INIT_DECLARE(ast_tach);
#endif
#ifndef DISABLE_XDP710
SENSOR_DRIVE_INIT_DECLARE(xdp710);
#endif
#ifdef ENABLE_DS160PT801
SENSOR_DRIVE_INIT_DECLARE(ds160pt801);
#endif
#ifdef ENABLE_RS31380R
SENSOR_DRIVE_INIT_DECLARE(rs31380r);
#endif
#ifndef DISABLE_ADC128D818
SENSOR_DRIVE_INIT_DECLARE(adc128d818);
#endif
#ifdef ENABLE_ADS1015
SENSOR_DRIVE_INIT_DECLARE(ads1015);
#endif
#ifdef ENABLE_PLAT_DEF_SENSOR
SENSOR_DRIVE_INIT_DECLARE(plat_def_sensor);
#endif
#ifdef ENABLE_PLAT_DEF_SENSOR
SENSOR_DRIVE_INIT_DECLARE(e50sn12051);
#endif

// The sequence needs to same with SENSOR_DEV ID
sensor_drive_api sensor_drive_tbl[] = {
#ifndef DISABLE_TMP75
	SENSOR_DRIVE_TYPE_INIT_MAP(tmp75),
#else
	SENSOR_DRIVE_TYPE_UNUSE(tmp75),
#endif
#ifndef DISABLE_AST_ADC
	SENSOR_DRIVE_TYPE_INIT_MAP(ast_adc),
#else
	SENSOR_DRIVE_TYPE_UNUSE(ast_adc),
#endif
	SENSOR_DRIVE_TYPE_INIT_MAP(intel_peci),
#ifndef DISABLE_ISL69259
	SENSOR_DRIVE_TYPE_INIT_MAP(isl69259),
#else
	SENSOR_DRIVE_TYPE_UNUSE(isl69259),
#endif
#ifndef DISABLE_ADM1278
	SENSOR_DRIVE_TYPE_INIT_MAP(adm1278),
#else
	SENSOR_DRIVE_TYPE_UNUSE(adm1278),
#endif
#ifndef DISABLE_NVME
	SENSOR_DRIVE_TYPE_INIT_MAP(nvme),
#else
	SENSOR_DRIVE_TYPE_UNUSE(nvme),
#endif
#ifndef DISABLE_PCH
	SENSOR_DRIVE_TYPE_INIT_MAP(pch),
#else
	SENSOR_DRIVE_TYPE_UNUSE(pch),
#endif
#ifndef DISABLE_MP5990
	SENSOR_DRIVE_TYPE_INIT_MAP(mp5990),
#else
	SENSOR_DRIVE_TYPE_UNUSE(mp5990),
#endif
#ifndef DISABLE_ISL28022
	SENSOR_DRIVE_TYPE_INIT_MAP(isl28022),
#else
	SENSOR_DRIVE_TYPE_UNUSE(isl28022),
#endif
#ifndef DISABLE_PEX89000
	SENSOR_DRIVE_TYPE_INIT_MAP(pex89000),
#else
	SENSOR_DRIVE_TYPE_UNUSE(pex89000),
#endif
#ifndef DISABLE_TPS53689
	SENSOR_DRIVE_TYPE_INIT_MAP(tps53689),
#else
	SENSOR_DRIVE_TYPE_UNUSE(tps53689),
#endif
#ifndef DISABLE_XDPE15284
	SENSOR_DRIVE_TYPE_INIT_MAP(xdpe15284),
#else
	SENSOR_DRIVE_TYPE_UNUSE(xdpe15284),
#endif
#ifndef DISABLE_LTC4282
	SENSOR_DRIVE_TYPE_INIT_MAP(ltc4282),
#else
	SENSOR_DRIVE_TYPE_UNUSE(ltc4282),
#endif
#ifdef ENABLE_FAN
	SENSOR_DRIVE_TYPE_INIT_MAP(ast_fan),
#else
	SENSOR_DRIVE_TYPE_UNUSE(ast_fan),
#endif
#ifndef DISABLE_TMP431
	SENSOR_DRIVE_TYPE_INIT_MAP(tmp431),
#else
	SENSOR_DRIVE_TYPE_UNUSE(tmp431),
#endif
#ifndef DISABLE_PMIC
	SENSOR_DRIVE_TYPE_INIT_MAP(pmic),
#else
	SENSOR_DRIVE_TYPE_UNUSE(pmic),
#endif
#ifndef DISABLE_INA233
	SENSOR_DRIVE_TYPE_INIT_MAP(ina233),
#else
	SENSOR_DRIVE_TYPE_UNUSE(ina233),
#endif
#ifndef DISABLE_ISL69254IRAZ_T
	SENSOR_DRIVE_TYPE_INIT_MAP(isl69254iraz_t),
#else
	SENSOR_DRIVE_TYPE_UNUSE(isl69254iraz_t),
#endif
#ifndef DISABLE_MAX16550A
	SENSOR_DRIVE_TYPE_INIT_MAP(max16550a),
#else
	SENSOR_DRIVE_TYPE_UNUSE(max16550a),
#endif
#ifndef DISABLE_INA230
	SENSOR_DRIVE_TYPE_INIT_MAP(ina230),
#else
	SENSOR_DRIVE_TYPE_UNUSE(ina230),
#endif
#ifndef DISABLE_XDP12284C
	SENSOR_DRIVE_TYPE_INIT_MAP(xdpe12284c),
#else
	SENSOR_DRIVE_TYPE_UNUSE(xdpe12284c),
#endif
#ifndef DISABLE_RAA229621
	SENSOR_DRIVE_TYPE_INIT_MAP(raa229621),
#else
	SENSOR_DRIVE_TYPE_UNUSE(raa229621),
#endif
#ifndef DISABLE_NCT7718W
	SENSOR_DRIVE_TYPE_INIT_MAP(nct7718w),
#else
	SENSOR_DRIVE_TYPE_UNUSE(nct7718w),
#endif
#ifndef DISABLE_LTC4286
	SENSOR_DRIVE_TYPE_INIT_MAP(ltc4286),
#else
	SENSOR_DRIVE_TYPE_UNUSE(ltc4286),
#endif
#ifdef ENABLE_APML
	SENSOR_DRIVE_TYPE_INIT_MAP(amd_tsi),
	SENSOR_DRIVE_TYPE_INIT_MAP(apml_mailbox),
#else
	SENSOR_DRIVE_TYPE_UNUSE(amd_tsi),
	SENSOR_DRIVE_TYPE_UNUSE(apml_mailbox),
#endif
#ifndef DISABLE_XDPE19283B
	SENSOR_DRIVE_TYPE_INIT_MAP(xdpe19283b),
#else
	SENSOR_DRIVE_TYPE_UNUSE(xdpe19283b),
#endif
#ifndef DISABLE_G788P81U
	SENSOR_DRIVE_TYPE_INIT_MAP(g788p81u),
#else
	SENSOR_DRIVE_TYPE_UNUSE(g788p81u),
#endif
#ifndef DISABLE_MP2856GUT
	SENSOR_DRIVE_TYPE_INIT_MAP(mp2856gut),
#else
	SENSOR_DRIVE_TYPE_UNUSE(mp2856gut),
#endif
#ifndef DISABLE_DDR5_POWER
	SENSOR_DRIVE_TYPE_INIT_MAP(ddr5_power),
#else
	SENSOR_DRIVE_TYPE_UNUSE(ddr5_power),
#endif
#ifndef DISABLE_DDR5_TEMP
	SENSOR_DRIVE_TYPE_INIT_MAP(ddr5_temp),
#else
	SENSOR_DRIVE_TYPE_UNUSE(ddr5_temp),
#endif
#ifndef DISABLE_ADM1272
	SENSOR_DRIVE_TYPE_INIT_MAP(adm1272),
#else
	SENSOR_DRIVE_TYPE_UNUSE(adm1272),
#endif
#ifndef DISABLE_Q50SN120A1
	SENSOR_DRIVE_TYPE_INIT_MAP(q50sn120a1),
#else
	SENSOR_DRIVE_TYPE_UNUSE(q50sn120a1),
#endif
#ifndef DISABLE_MP2971
	SENSOR_DRIVE_TYPE_INIT_MAP(mp2971),
#else
	SENSOR_DRIVE_TYPE_UNUSE(mp2971),
#endif
#ifdef ENABLE_PM8702
	SENSOR_DRIVE_TYPE_INIT_MAP(pm8702),
#else
	SENSOR_DRIVE_TYPE_UNUSE(pm8702),
#endif
#ifndef DISABLE_LTC2991
	SENSOR_DRIVE_TYPE_INIT_MAP(ltc2991),
#else
	SENSOR_DRIVE_TYPE_UNUSE(ltc2991),
#endif
#ifndef DISABLE_SQ52205
	SENSOR_DRIVE_TYPE_INIT_MAP(sq52205),
#else
	SENSOR_DRIVE_TYPE_UNUSE(sq52205),
#endif
#ifndef DISABLE_EMC1412
	SENSOR_DRIVE_TYPE_INIT_MAP(emc1412),
#else
	SENSOR_DRIVE_TYPE_UNUSE(emc1412),
#endif
#ifndef DISABLE_I3C_DIMM
	SENSOR_DRIVE_TYPE_INIT_MAP(i3c_dimm),
#else
	SENSOR_DRIVE_TYPE_UNUSE(i3c_dimm),
#endif
#ifndef DISABLE_PT5161L
	SENSOR_DRIVE_TYPE_INIT_MAP(pt5161l),
#else
	SENSOR_DRIVE_TYPE_UNUSE(pt5161l),
#endif
#ifndef DISABLE_LM75BD118
	SENSOR_DRIVE_TYPE_INIT_MAP(lm75bd118),
#else
	SENSOR_DRIVE_TYPE_UNUSE(lm75bd118),
#endif
#ifndef DISABLE_TMP461
	SENSOR_DRIVE_TYPE_INIT_MAP(tmp461),
#else
	SENSOR_DRIVE_TYPE_UNUSE(tmp461),
#endif
#ifndef DISABLE_MP2985
	SENSOR_DRIVE_TYPE_INIT_MAP(mp2985),
#else
	SENSOR_DRIVE_TYPE_UNUSE(mp2985),
#endif
#ifndef DISABLE_M88RT51632
	SENSOR_DRIVE_TYPE_INIT_MAP(m88rt51632),
#else
	SENSOR_DRIVE_TYPE_UNUSE(m88rt51632),
#endif
#ifdef ENABLE_MPRO
	SENSOR_DRIVE_TYPE_INIT_MAP(mpro),
#else
	SENSOR_DRIVE_TYPE_UNUSE(mpro),
#endif
#ifndef DISABLE_BMR351
	SENSOR_DRIVE_TYPE_INIT_MAP(bmr351),
#else
	SENSOR_DRIVE_TYPE_UNUSE(bmr351),
#endif
#ifndef DISABLE_CX7
	SENSOR_DRIVE_TYPE_INIT_MAP(cx7),
#else
	SENSOR_DRIVE_TYPE_UNUSE(cx7),
#endif
#ifdef ENABLE_VISTARA
	SENSOR_DRIVE_TYPE_INIT_MAP(vistara),
#else
	SENSOR_DRIVE_TYPE_UNUSE(vistara),
#endif
#ifndef DISABLE_MAX11617
	SENSOR_DRIVE_TYPE_INIT_MAP(max11617),
#else
	SENSOR_DRIVE_TYPE_UNUSE(max11617),
#endif
#ifdef ENABLE_NVIDIA
	SENSOR_DRIVE_TYPE_INIT_MAP(nv_satmc),
#else
	SENSOR_DRIVE_TYPE_UNUSE(nv_satmc),
#endif
#ifndef DISABLE_NCT7363
	SENSOR_DRIVE_TYPE_INIT_MAP(nct7363),
#else
	SENSOR_DRIVE_TYPE_UNUSE(nct7363),
#endif
#ifndef DISABLE_ADS112C
	SENSOR_DRIVE_TYPE_INIT_MAP(ads112c),
#else
	SENSOR_DRIVE_TYPE_UNUSE(ads112c),
#endif
#ifndef DISABLE_HDC1080
	SENSOR_DRIVE_TYPE_INIT_MAP(hdc1080),
#else
	SENSOR_DRIVE_TYPE_UNUSE(hdc1080),
#endif
#ifndef DISABLE_INA238
	SENSOR_DRIVE_TYPE_INIT_MAP(ina238),
#else
	SENSOR_DRIVE_TYPE_UNUSE(ina238),
#endif
#ifndef DISABLE_NCT214
	SENSOR_DRIVE_TYPE_INIT_MAP(nct214),
#else
	SENSOR_DRIVE_TYPE_UNUSE(nct214),
#endif
#ifndef DISABLE_AST_TACH
	SENSOR_DRIVE_TYPE_INIT_MAP(ast_tach),
#else
	SENSOR_DRIVE_TYPE_UNUSE(ast_tach),
#endif
#ifndef DISABLE_XDP710
	SENSOR_DRIVE_TYPE_INIT_MAP(xdp710),
#else
	SENSOR_DRIVE_TYPE_UNUSE(xdp710),
#endif
#ifdef ENABLE_DS160PT801
	SENSOR_DRIVE_TYPE_INIT_MAP(ds160pt801),
#else
	SENSOR_DRIVE_TYPE_UNUSE(ds160pt801),
#endif
#ifdef ENABLE_RS31380R
	SENSOR_DRIVE_TYPE_INIT_MAP(rs31380r),
#else
	SENSOR_DRIVE_TYPE_UNUSE(rs31380r),
#endif
#ifndef DISABLE_ADC128D818
	SENSOR_DRIVE_TYPE_INIT_MAP(adc128d818),
#else
	SENSOR_DRIVE_TYPE_UNUSE(adc128d818),
#endif
#ifdef ENABLE_ADS1015
	SENSOR_DRIVE_TYPE_INIT_MAP(ads1015),
#else
	SENSOR_DRIVE_TYPE_UNUSE(ads1015),
#endif
#ifdef ENABLE_PLAT_DEF_SENSOR
	SENSOR_DRIVE_TYPE_INIT_MAP(plat_def_sensor),
#else
	SENSOR_DRIVE_TYPE_UNUSE(plat_def_sensor),
#endif
#ifdef ENABLE_PLAT_DEF_SENSOR
	SENSOR_DRIVE_TYPE_INIT_MAP(e50sn12051),
#else
	SENSOR_DRIVE_TYPE_UNUSE(e50sn12051),
#endif
};

static void init_sensor_num(void)
{
	for (int i = 0; i < SENSOR_NUM_MAX; i++) {
		sdr_index_map[i] = 0xFF;
		sensor_config_index_map[i] = 0xFF;
	}
}

void map_sensor_num_to_sdr_cfg(void)
{
	uint8_t i, j;

	for (i = 0; i < SENSOR_NUM_MAX; i++) {
		for (j = 0; j < sdr_count; j++) {
			if (i == full_sdr_table[j].sensor_num) {
				sdr_index_map[i] = j;
				break;
			} else if (i == sdr_count) {
				sdr_index_map[i] = SENSOR_NULL;
			} else {
				continue;
			}
		}
		for (j = 0; j < sdr_count; j++) {
			if (i == sensor_config[j].num) {
				sensor_config_index_map[i] = j;
				break;
			} else if (i == sdr_count) {
				sensor_config_index_map[i] = SENSOR_NULL;
			} else {
				continue;
			}
		}
	}
	return;
}

__weak sensor_cfg *get_common_sensor_cfg_info(uint8_t sensor_num)
{
	if (!sensor_monitor_table)
		return NULL;

	uint8_t cfg_count = sensor_monitor_table[0].cfg_count;
	sensor_cfg *cfg_table = sensor_monitor_table[0].monitor_sensor_cfg;

	if (cfg_table != NULL) {
		return find_sensor_cfg_via_sensor_num(cfg_table, cfg_count, sensor_num);
	}

	return NULL;
}

sensor_cfg *find_sensor_cfg_via_sensor_num(sensor_cfg *cfg_table, uint8_t cfg_count,
					   uint8_t sensor_num)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg_table, false);

	uint8_t index = 0;

	for (index = 0; index < cfg_count; ++index) {
		if (cfg_table[index].num == sensor_num) {
			return &cfg_table[index];
		}
	}

	return NULL;
}

bool access_check(uint8_t sensor_num)
{
	bool (*access_checker)(uint8_t);

	access_checker = sensor_config[sensor_config_index_map[sensor_num]].access_checker;
	return (access_checker)(sensor_config[sensor_config_index_map[sensor_num]].num);
}

void clear_unaccessible_sensor_cache(sensor_cfg *cfg)
{
	CHECK_NULL_ARG(cfg);

	if (cfg->cache_status != SENSOR_INIT_STATUS) {
		cfg->cache = SENSOR_FAIL;
		cfg->cache_status = SENSOR_INIT_STATUS;
	}
}

uint8_t get_sensor_reading(sensor_cfg *cfg_table, uint8_t cfg_count, uint8_t sensor_num,
			   int *reading, uint8_t read_mode)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg_table, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	// Check sensor information in sensor config table
	// Block BMC send invalid sensor number by OEM accurate read command
	sensor_cfg *cfg = NULL;
	cfg = find_sensor_cfg_via_sensor_num(cfg_table, cfg_count, sensor_num);
	if (cfg == NULL) {
		LOG_ERR("Fail to find sensor info in config table, sensor_num: 0x%x, cfg count: 0x%x",
			sensor_num, cfg_count);
		return SENSOR_NOT_FOUND;
	}

	*reading = 0; // Initial return reading value
	uint8_t current_status = SENSOR_UNSPECIFIED_ERROR;
	bool post_ret = false;

	if (cfg->cache_status == SENSOR_NOT_PRESENT) {
		return cfg->cache_status;
	}

	if (cfg->access_checker(sensor_num) != true) { // sensor not accessable
		clear_unaccessible_sensor_cache(cfg);
		cfg->cache_status = SENSOR_NOT_ACCESSIBLE;
		return cfg->cache_status;
	}

	switch (read_mode) {
	case GET_FROM_SENSOR:
		if (cfg->pre_sensor_read_hook) {
			if (cfg->pre_sensor_read_hook(cfg, cfg->pre_sensor_read_args) == false) {
				LOG_ERR("Failed to do pre sensor read function, sensor number: 0x%x",
					sensor_num);
				cfg->cache_status = SENSOR_PRE_READ_ERROR;
				return cfg->cache_status;
			}
			if ((cfg->cache_status == SENSOR_NOT_PRESENT) ||
			    (cfg->cache_status == SENSOR_POLLING_DISABLE)) {
				return cfg->cache_status;
			}
		}

		if (cfg->read) {
			current_status = cfg->read(cfg, reading);
		}

		if (current_status == SENSOR_READ_SUCCESS ||
		    current_status == SENSOR_READ_ACUR_SUCCESS) {
			cfg->retry = 0;
			if (cfg->post_sensor_read_hook) { // makesure post hook function be called
				post_ret = cfg->post_sensor_read_hook(
					cfg, cfg->post_sensor_read_args, reading);
			}

			if (cfg->access_checker(sensor_num) !=
			    true) { // double check access to avoid not accessible read at same moment status change
				clear_unaccessible_sensor_cache(cfg);
				cfg->cache_status = SENSOR_NOT_ACCESSIBLE;
				return cfg->cache_status;
			}

			if (cfg->post_sensor_read_hook && post_ret == false) {
				LOG_ERR("Failed to do post sensor read function, sensor number: 0x%x",
					sensor_num);
				cfg->cache_status = SENSOR_POST_READ_ERROR;
				return cfg->cache_status;
			}
			memcpy(&cfg->cache, reading, sizeof(*reading));
			cfg->cache_status = SENSOR_READ_4BYTE_ACUR_SUCCESS;
			return cfg->cache_status;
		} else {
			/* Return current status if retry reach max retry count, otherwise return cache status instead of current status */
			if (cfg->retry >= SENSOR_READ_RETRY_MAX) {
				cfg->cache_status = current_status;
			} else {
				cfg->retry++;
			}

			/* If sensor read fails, let the reading argument in the
       * post_sensor_read_hook function to NULL.
       * All post_sensor_read_hook function define in each platform should check
       * reading whether is NULL to do the corresponding thing. (Ex: mutex_unlock)
       */
			if (cfg->post_sensor_read_hook) {
				if (cfg->post_sensor_read_hook(cfg, cfg->post_sensor_read_args,
							       NULL) == false) {
					LOG_ERR("Sensor number 0x%x reading and post_read fail",
						sensor_num);
				}
			}

			return cfg->cache_status;
		}
		break;
	case GET_FROM_CACHE:
		switch (cfg->cache_status) {
		case SENSOR_READ_SUCCESS:
		case SENSOR_READ_ACUR_SUCCESS:
		case SENSOR_READ_4BYTE_ACUR_SUCCESS:
			*reading = cfg->cache;
			if (cfg->access_checker(sensor_num) !=
			    true) { // double check access to avoid not accessible read at same moment status change
				cfg->cache_status = SENSOR_NOT_ACCESSIBLE;
			}
			return cfg->cache_status;
		case SENSOR_INIT_STATUS:
		case SENSOR_NOT_PRESENT:
		case SENSOR_NOT_ACCESSIBLE:
		case SENSOR_POLLING_DISABLE:
			cfg->cache = SENSOR_FAIL;
			return cfg->cache_status;
		case SENSOR_UNAVAILABLE:
			return cfg->cache_status;
		default:
			cfg->cache = SENSOR_FAIL;
			LOG_ERR("Failed to read sensor value from cache, sensor number: 0x%x, cache status: 0x%x",
				sensor_num, cfg->cache_status);
			return cfg->cache_status;
		}
		break;
	default:
		LOG_ERR("Invalid mbr type during changing sensor mbr");
		break;
	}

	cfg->cache_status = current_status;
	return cfg->cache_status;
}

void disable_sensor_poll()
{
	sensor_poll_enable_flag = false;
}

void enable_sensor_poll()
{
	sensor_poll_enable_flag = true;
}

bool get_sensor_poll_enable_flag()
{
	return sensor_poll_enable_flag;
}

void sensor_poll_handler(void *arug0, void *arug1, void *arug2)
{
	uint16_t table_index = 0;
	uint8_t sensor_index = 0;
	uint8_t sensor_num = 0;
	int sensor_poll_interval_ms = 0;
	int reading = 0;
	bool ret = false;

	k_msleep(1000); // delay 1 second to wait for drivers ready before start sensor polling

	pal_set_sensor_poll_interval(&sensor_poll_interval_ms);

	while (1) {
		for (table_index = 0; table_index < sensor_monitor_count; ++table_index) {
			sensor_monitor_table_info *table_info = &sensor_monitor_table[table_index];

			if (table_info->access_checker != NULL) {
				if (table_info->access_checker(table_info->access_checker_arg) !=
				    true) {
					continue;
				}
			}

			sensor_cfg *cfg_table = table_info->monitor_sensor_cfg;
			if (cfg_table == NULL) {
				LOG_ERR("Table index: 0x%x is NULL, skip to monitor sensor table",
					table_index);
				continue;
			}

			uint8_t sensor_count = table_info->cfg_count;
			for (sensor_index = 0; sensor_index < sensor_count; ++sensor_index) {
				if (sensor_poll_enable_flag ==
				    false) { /* skip if disable sensor poll */
					break;
				}
				sensor_cfg *cfg = &cfg_table[sensor_index];
				sensor_num = cfg->num;

				if (cfg->cache_status == SENSOR_NOT_PRESENT) {
					continue;
				}

				// Check whether monitoring sensor is enabled
				if (cfg->is_enable_polling == DISABLE_SENSOR_POLLING) {
					cfg->cache = SENSOR_FAIL;
					cfg->cache_status = SENSOR_POLLING_DISABLE;
					continue;
				}

				if (cfg->poll_time != POLL_TIME_DEFAULT) {
					if (pal_is_time_to_poll(sensor_num, cfg->poll_time) ==
					    false) {
						continue;
					}
				}

				if (table_info->pre_monitor != NULL) {
					ret = table_info->pre_monitor(
						sensor_num, table_info->pre_post_monitor_arg);
					if (ret != true) {
						LOG_ERR("Pre-monitor fail, table index: 0x%x, sensor num: 0x%x",
							table_index, sensor_num);
						continue;
					}
				}

				get_sensor_reading(cfg_table, sensor_count, sensor_num, &reading,
						   GET_FROM_SENSOR);

				if (table_info->post_monitor != NULL) {
					ret = table_info->post_monitor(
						sensor_num, table_info->pre_post_monitor_arg);
					if (ret != true) {
						LOG_ERR("Post-monitor fail, table index: 0x%x, sensor num: 0x%x",
							table_index, sensor_num);
						continue;
					}
				}
			}

			k_yield();
		}

		is_sensor_ready_flag = true;
		k_msleep(sensor_poll_interval_ms);
	}
}

__weak bool pal_is_time_to_poll(uint8_t sensor_num, int poll_time)
{
	return true;
}

__weak void pal_set_sensor_poll_interval(int *interval_ms)
{
	*interval_ms = 1000;
	return;
}

__weak void pal_extend_sensor_config(void)
{
	return;
}

__weak uint8_t pal_get_extend_sdr()
{
	return 0;
}

__weak uint8_t pal_get_extend_sensor_config()
{
	return 0;
}

__weak uint8_t pal_get_monitor_sensor_count()
{
	return 0;
}

__weak void plat_fill_monitor_sensor_table()
{
	return;
}

void check_init_sensor_size()
{
	uint8_t init_sdr_size = plat_get_sdr_size();
	uint8_t init_sensor_config_size = plat_get_config_size();
	uint8_t extend_sdr_size = pal_get_extend_sdr();
	uint8_t extend_sensor_config_size = pal_get_extend_sensor_config();

	init_sdr_size += extend_sdr_size;
	init_sensor_config_size += extend_sensor_config_size;

	if (init_sdr_size != init_sensor_config_size) {
		enable_sensor_poll_thread = false;
		LOG_ERR("Init sdr size is not equal to config size, sdr size: 0x%x, config size: 0x%x",
			init_sdr_size, init_sensor_config_size);
		LOG_ERR("BIC should not monitor sensors if SDR size and sensor config size is not match, BIC would not start sensor thread");
		return;
	}
	sensor_config_size = init_sdr_size;
}

bool stby_access(uint8_t sensor_num)
{
	return true;
}

bool dc_access(uint8_t sensor_num)
{
	return get_DC_on_delayed_status();
}

bool post_access(uint8_t sensor_num)
{
	return get_post_status();
}

bool me_access(uint8_t sensor_num)
{
	if (get_me_mode() == ME_NORMAL_MODE) {
		return get_post_status();
	} else {
		return false;
	}
}

bool vr_access(uint8_t sensor_num)
{
	if (get_DC_on_delayed_status() == false) {
		return false;
	}
	return get_vr_monitor_status();
}

bool vr_stby_access(uint8_t sensor_num)
{
	return get_vr_monitor_status();
}

void sensor_poll_init()
{
	k_thread_create(&sensor_poll, sensor_poll_stack, K_THREAD_STACK_SIZEOF(sensor_poll_stack),
			sensor_poll_handler, NULL, NULL, NULL, CONFIG_MAIN_THREAD_PRIORITY, 0,
			K_NO_WAIT);
	k_thread_name_set(&sensor_poll, "sensor_poll");
	return;
}

uint8_t get_sensor_config_index(uint8_t sensor_num)
{
	uint8_t i = 0;
	for (i = 0; i < sensor_config_count; ++i) {
		if (sensor_num == sensor_config[i].num) {
			return i;
		}
	}
	return SENSOR_NUM_MAX;
}

void add_sensor_config(sensor_cfg config)
{
	uint8_t index = get_sensor_config_index(config.num);
	if (index != SENSOR_NUM_MAX) {
		memcpy(&sensor_config[index], &config, sizeof(sensor_cfg));
		LOG_INF("Change the sensor[0x%02x] configuration", config.num);
		return;
	}
	// Check config table size before adding sensor config
	if (sensor_config_count + 1 <= sdr_count) {
		sensor_config[sensor_config_count++] = config;
	} else {
		LOG_ERR("Add config would over config max size");
	}
}

void init_sensor_monitor_table()
{
	sensor_monitor_count = 1;

	uint8_t plat_monitor_sensor_count = pal_get_monitor_sensor_count();
	sensor_monitor_count += plat_monitor_sensor_count;

	sensor_monitor_table = (sensor_monitor_table_info *)malloc(
		sensor_monitor_count * sizeof(sensor_monitor_table_info));
	if (sensor_monitor_table == NULL) {
		LOG_ERR("Fail to allocate memory to store sensor monitor table");
		return;
	}

	if (sensor_config != NULL) {
		sensor_monitor_table[0].monitor_sensor_cfg = sensor_config;
		sensor_monitor_table[0].cfg_count = sensor_config_count;
		sensor_monitor_table[0].access_checker = NULL;
		sensor_monitor_table[0].pre_monitor = NULL;
		sensor_monitor_table[0].post_monitor = NULL;
		snprintf(sensor_monitor_table[0].table_name,
			 sizeof(sensor_monitor_table[0].table_name), "%s",
			 common_sensor_table_name);
	}

	plat_fill_monitor_sensor_table();
}

static inline bool init_drive_type(sensor_cfg *p, uint16_t current_drive)
{
	if (p == NULL) {
		LOG_ERR("p is NULL, current drive: 0x%x", current_drive);
		return false;
	}

	int ret = -1;
	if (p->type != sensor_drive_tbl[current_drive].dev) {
		return false;
	}

	if (p->pre_sensor_read_hook) {
		if (p->pre_sensor_read_hook(p, p->pre_sensor_read_args) == false) {
			LOG_ERR("Sensor 0x%x pre sensor read failed!", p->num);
			return false;
		}
	}

	ret = sensor_drive_tbl[current_drive].init(p);
	if (ret != SENSOR_INIT_SUCCESS) {
		LOG_ERR("Sensor num %d initial fail, ret %d", p->num, ret);
	}

	if (p->post_sensor_read_hook) {
		if (p->post_sensor_read_hook(p, p->post_sensor_read_args, NULL) == false) {
			LOG_ERR("Sensor 0x%x post sensor read failed!", p->num);
		}
	}

	return true;
}

uint8_t common_tbl_sen_reinit(uint8_t sen_num)
{
	sensor_cfg *cfg = get_common_sensor_cfg_info(sen_num);
	if (!cfg) {
		LOG_ERR("Fail to get sensor config info, sensor number: 0x%x", sen_num);
		return SENSOR_NOT_FOUND;
	}

	for (uint8_t i = 0; i < ARRAY_SIZE(sensor_drive_tbl); i++) {
		if (cfg->type != sensor_drive_tbl[i].dev)
			continue;

		if (init_drive_type(cfg, i) == false) {
			LOG_ERR("reinit drive type fail, sensor num: 0x%x, type: 0x%x", cfg->num,
				cfg->type);
			return SENSOR_NOT_FOUND;
		}

		LOG_DBG("Reinit sensor num: 0x%x, type: 0x%x success", cfg->num, cfg->type);

		break;
	}

	return 0;
}

static void drive_init(void)
{
	const uint16_t max_drive_num = ARRAY_SIZE(sensor_drive_tbl);

	bool ret = false;
	uint16_t table_index = 0;
	uint8_t sensor_index = 0;
	uint16_t current_drive = 0;

	for (table_index = 0; table_index < sensor_monitor_count; ++table_index) {
		sensor_monitor_table_info *table_info = &sensor_monitor_table[table_index];
		if (table_info->access_checker != NULL) {
			if (table_info->access_checker(table_info->access_checker_arg) != true) {
				LOG_WRN("[%s] table: 0x%x can't access, skip init drive", __func__,
					table_index);
				continue;
			}
		}

		sensor_cfg *cfg_table = sensor_monitor_table[table_index].monitor_sensor_cfg;
		if (cfg_table == NULL) {
			LOG_ERR("Table index: 0x%x is NULL, skip to initialize drive", table_index);
			continue;
		}
		for (sensor_index = 0; sensor_index < sensor_monitor_table[table_index].cfg_count;
		     ++sensor_index) {
			sensor_cfg *cfg = &cfg_table[sensor_index];
			for (current_drive = 0; current_drive < max_drive_num; ++current_drive) {
				if (cfg->type == sensor_drive_tbl[current_drive].dev) {
					if (table_info->pre_monitor != NULL) {
						ret = table_info->pre_monitor(
							cfg->num, table_info->pre_post_monitor_arg);
						if (ret != true) {
							LOG_ERR("Pre-monitor fail cause drive init fail, table index: 0x%x, sensor num: 0x%x, type: 0x%x",
								table_index, cfg->num, cfg->type);
							break;
						}
					}

					ret = init_drive_type(cfg, current_drive);
					if (ret != true) {
						LOG_ERR("init drive type fail, table index: 0x%x, sensor num: 0x%x, type: 0x%x",
							table_index, cfg->num, cfg->type);
					}

					if (table_info->post_monitor != NULL) {
						ret = table_info->post_monitor(
							cfg->num, table_info->pre_post_monitor_arg);
						if (ret != true) {
							LOG_ERR("Post-monitor fail cause drive init fail, table index: 0x%x, sensor num: 0x%x, type: 0x%x",
								table_index, cfg->num, cfg->type);
						}
					}

					break;
				}
			}

			if (current_drive == max_drive_num) {
				LOG_ERR("Table index: 0x%x, sensor number: 0x%x, type = 0x%x is not supported!",
					table_index, cfg->num, cfg->type);
				cfg->read = NULL;
			}
		}
	}
}

bool sensor_init(void)
{
	init_sensor_num();
	// Check init SDR size is equal to sensor config size
	check_init_sensor_size();

	if (sensor_config_size != 0) {
		full_sdr_table =
			(SDR_Full_sensor *)malloc(sensor_config_size * sizeof(SDR_Full_sensor));
		if (full_sdr_table != NULL) {
			sdr_init();
		} else {
			LOG_ERR("Fail to allocate memory to SDR table");
			return false;
		}
	} else {
		LOG_ERR("Init sensor size is zero");
		return false;
	}

	if (sdr_count != 0) {
		sensor_config = (sensor_cfg *)malloc(sdr_count * sizeof(sensor_cfg));
		if (sensor_config != NULL) {
			load_sensor_config();
		} else {
			SAFE_FREE(full_sdr_table);
			LOG_ERR("Fail to allocate memory to config table");
			return false;
		}
	} else {
		LOG_ERR("SDR number is zero");
		return false;
	}

	map_sensor_num_to_sdr_cfg();
	init_sensor_monitor_table();

	/* register read api of sensor_config */
	drive_init();

	if (DEBUG_SENSOR) {
		LOG_ERR("Sensor name: %s", log_strdup(full_sdr_table[sdr_index_map[1]].ID_str));
	}

	if (enable_sensor_poll_thread) {
		sensor_poll_init();
	}

	is_sensor_initial_done = true;
	return true;
}

bool check_is_sensor_ready()
{
	return is_sensor_ready_flag;
}

uint8_t plat_get_config_size()
{
	return SENSOR_CONFIG_SIZE;
}

__weak void load_sensor_config(void)
{
	memcpy(sensor_config, plat_sensor_config, sizeof(sensor_cfg) * SENSOR_CONFIG_SIZE);
	sensor_config_count = SENSOR_CONFIG_SIZE;

	pal_extend_sensor_config();
}

void control_sensor_polling(uint8_t sensor_num, uint8_t optional, uint8_t cache_status)
{
	if ((sensor_num == SENSOR_NOT_SUPPORT) ||
	    (sensor_config_index_map[sensor_num] == SENSOR_FAIL)) {
		return;
	}

	if ((optional != DISABLE_SENSOR_POLLING) && (optional != ENABLE_SENSOR_POLLING)) {
		LOG_ERR("Input optional is not support, optional: %d", optional);
		return;
	}

	sensor_cfg *config = &sensor_config[sensor_config_index_map[sensor_num]];
	config->is_enable_polling = optional;
	config->cache_status = cache_status;
}

bool check_reading_pointer_null_is_allowed(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);

	uint8_t sensor_retry_count = cfg->retry;

	/* Reading pointer NULL is allowed in sensor initial stage and sensor reading fail */
	/* Sensor retry count will be set to zero when the sensor read success */
	if (is_sensor_initial_done && (sensor_retry_count == 0)) {
		return false;
	} else {
		return true;
	}
}

bool init_drive_type_delayed(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);

	int ret = -1;
	uint8_t index = 0;
	const uint16_t max_drive_num = ARRAY_SIZE(sensor_drive_tbl);

	for (index = 0; index < max_drive_num; index++) {
		if (cfg->type == sensor_drive_tbl[index].dev) {
			ret = sensor_drive_tbl[index].init(cfg);
			if (ret != SENSOR_INIT_SUCCESS) {
				return false;
			}

			return true;
		}
	}

	return false;
}

bool get_sensor_init_done_flag()
{
	return is_sensor_initial_done;
}
