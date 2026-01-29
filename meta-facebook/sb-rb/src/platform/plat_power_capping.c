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

#include <kernel.h>
#include <logging/log.h>
#include "raa228249.h"
#include "mp29816a.h"
#include "pldm_sensor.h"
#include "plat_adc.h"
#include "plat_class.h"
#include "plat_cpld.h"
#include "plat_hook.h"
#include "plat_i2c_target.h"
#include "plat_power_capping.h"
#include "plat_pldm_sensor.h"

LOG_MODULE_REGISTER(plat_power_capping);

#define POWER_CAPPING_STACK_SIZE 1024
K_THREAD_STACK_DEFINE(power_capping_thread_stack, POWER_CAPPING_STACK_SIZE);
struct k_thread power_capping_thread;

typedef struct {
	uint8_t method;
	uint8_t source;
	uint16_t current_threshold[CAPPING_VR_IDX_MAX];
	uint16_t time_w[CAPPING_VR_IDX_MAX][CAPPING_LV_IDX_MAX];
	uint16_t threshold[CAPPING_VR_IDX_MAX][CAPPING_LV_IDX_MAX];
} power_capping_info_t;

static power_capping_info_t power_capping_info = { 0 };
static const uint16_t cpld_lv1_time_window_list[CPLD_LV1_TIME_WINDOW_NUM] = { 0,  1,  3,  5,
									      10, 15, 20, 50 };
static uint8_t prev_set_ucr_status = 0;
const static uint8_t volt_sensor_id_list[2] = { SENSOR_NUM_ASIC_P0V85_MEDHA0_VDD_VOLT_V,
						SENSOR_NUM_ASIC_P0V85_MEDHA1_VDD_VOLT_V };
const static uint8_t powr_sensor_id_list[2] = { SENSOR_NUM_ASIC_P0V85_MEDHA0_VDD_PWR_W,
						SENSOR_NUM_ASIC_P0V85_MEDHA1_VDD_PWR_W };

K_WORK_DELAYABLE_DEFINE(sync_vr_oc_work, power_capping_syn_vr_oc_warn_limit);

void power_capping_syn_vr_oc_warn_limit()
{
	if (!is_mb_dc_on()) {
		LOG_WRN("need to DC on!");
		return;
	}

	for (uint8_t i = 0; i < ARRAY_SIZE(volt_sensor_id_list); i++) {
		uint8_t sensor_id = volt_sensor_id_list[i];
		sensor_cfg *cfg = get_sensor_cfg_by_sensor_id(sensor_id);

		if (cfg == NULL) {
			LOG_ERR("Fail to get sensor config: 0x%x", sensor_id);
			continue;
		}

		if (!cfg->pre_sensor_read_hook) {
			LOG_ERR("NULL pre_sensor_read_hook: 0x%x", sensor_id);
			continue;
		}

		vr_pre_proc_arg *pre_proc_args = cfg->pre_sensor_read_args;
		if (pre_proc_args == NULL) {
			LOG_ERR("NULL pre_sensor_read_args: 0x%x", sensor_id);
			continue;
		}

		if (!cfg->pre_sensor_read_hook(cfg, cfg->pre_sensor_read_args)) {
			LOG_ERR("sensor id: 0x%x pre-read fail", sensor_id);
			/* mutex unlock */
			if (pre_proc_args->mutex) {
				k_mutex_unlock(pre_proc_args->mutex);
			}
			continue;
		}

		uint16_t value = 0;
		uint16_t voltage_value = 0;
		float float_value = 0;
		if (get_vr_module() == VR_MODULE_MPS) {
			if (mp29816a_get_iout_oc_warn_limit(cfg, &value)) {
				if (mp29816a_get_vout_command(cfg, 0, &voltage_value)) {
					float_value = voltage_value / 1000.0;
					power_capping_info.current_threshold[i] = value;
					power_capping_info.threshold[i][CAPPING_LV_IDX_LV1] =
						value * float_value;
				} else {
					LOG_ERR("Can't get VOUT_COMMAND: 0x%x", sensor_id);
				}
			} else {
				LOG_ERR("Can't get IOUT_OC_WARN: 0x%x", sensor_id);
			}
		} else if (get_vr_module() == VR_MODULE_RNS) {
			if (raa228249_get_iout_oc_warn_limit(cfg, &value)) {
				if (raa228249_get_vout_command(cfg, 0, &voltage_value)) {
					float_value = voltage_value / 1000.0;
					power_capping_info.current_threshold[i] = value;
					power_capping_info.threshold[i][CAPPING_LV_IDX_LV1] =
						value * float_value;
				} else {
					LOG_ERR("Can't get VOUT_COMMAND: 0x%x", sensor_id);
				}
			} else {
				LOG_ERR("Can't get IOUT_OC_WARN: 0x%x", sensor_id);
			}
		} else {
			LOG_ERR("Unknown VR module: %d", get_vr_module());
		}

		/* mutex unlock */
		if (pre_proc_args->mutex) {
			k_mutex_unlock(pre_proc_args->mutex);
		}
	}
}

void add_sync_oc_warn_to_work()
{
	k_work_schedule(&sync_vr_oc_work, K_MSEC(1000));
}

bool set_power_capping_vr_oc_warn_limit(uint8_t vr_idx, uint16_t value)
{
	/* input value unit: 1W */

	if (!is_mb_dc_on()) {
		LOG_WRN("need to DC on!");
		return false;
	}

	uint8_t ret = false;
	uint8_t sensor_id = volt_sensor_id_list[vr_idx];
	sensor_cfg *cfg = get_sensor_cfg_by_sensor_id(sensor_id);

	if (cfg == NULL) {
		LOG_ERR("Fail to get sensor config: 0x%x", sensor_id);
		return false;
	}

	if (!cfg->pre_sensor_read_hook) {
		LOG_ERR("NULL pre_sensor_read_hook: 0x%x", sensor_id);
		return false;
	}

	vr_pre_proc_arg *pre_proc_args = cfg->pre_sensor_read_args;
	if (pre_proc_args == NULL) {
		LOG_ERR("NULL pre_sensor_read_args: 0x%x", sensor_id);
		return false;
	}

	if (!cfg->pre_sensor_read_hook(cfg, cfg->pre_sensor_read_args)) {
		LOG_ERR("sensor id: 0x%x pre-read fail", sensor_id);
		/* mutex unlock */
		if (pre_proc_args->mutex) {
			k_mutex_unlock(pre_proc_args->mutex);
		}
		return false;
	}

	if (get_vr_module() == VR_MODULE_MPS) {
		uint16_t voltage_value = 0;
		float float_value = 0;
		ret = mp29816a_get_vout_command(cfg, 0, &voltage_value);
		if (ret) {
			float_value = voltage_value / 1000.0;
			uint16_t current_val = value / float_value;
			ret = mp29816a_set_iout_oc_warn_limit(cfg, current_val);
			if (ret) {
				mp29816a_get_iout_oc_warn_limit(cfg, &current_val);
				power_capping_info.current_threshold[vr_idx] = current_val;
			} else {
				LOG_ERR("Can't set IOUT_OC_WARN 0x%x", sensor_id);
			}
		} else {
			LOG_ERR("Can't get VOUT_COMMAND: 0x%x", sensor_id);
		}
	} else if (get_vr_module() == VR_MODULE_RNS) {
		uint16_t voltage_value = 0;
		float float_value = 0;
		ret = raa228249_get_vout_command(cfg, 0, &voltage_value);
		if (ret) {
			float_value = voltage_value / 1000.0;
			uint16_t current_val = value / float_value;
			ret = raa228249_set_iout_oc_warn_limit(cfg, current_val);
			if (ret) {
				raa228249_get_iout_oc_warn_limit(cfg, &current_val);
				power_capping_info.current_threshold[vr_idx] = current_val;
			} else {
				LOG_ERR("Can't set IOUT_OC_WARN 0x%x", sensor_id);
			}
		} else {
			LOG_ERR("Can't get VOUT_COMMAND: 0x%x", sensor_id);
		}
	} else {
		LOG_ERR("Unknown VR module: %d", get_vr_module());
	}

	/* mutex unlock */
	if (pre_proc_args->mutex) {
		k_mutex_unlock(pre_proc_args->mutex);
	}

	return ret;
}

bool find_cpld_lv1_time_window_idx_by_value(uint8_t *idx, uint16_t value)
{
	for (uint8_t i = 0; i < CPLD_LV1_TIME_WINDOW_NUM; i++) {
		if (cpld_lv1_time_window_list[i] == value) {
			*idx = i;
			return true;
		}
	}
	return false;
}

uint16_t get_power_capping_avg_power(uint8_t vr_idx, uint8_t lv)
{
	if (vr_idx >= CAPPING_VR_IDX_MAX) {
		LOG_ERR("Wrong vr idx %d", vr_idx);
		return 0;
	}

	if (lv >= CAPPING_LV_IDX_MAX) {
		LOG_ERR("Wrong vr lv %d", lv);
		return 0;
	}

	uint8_t tmp_idx = 0;
	switch (lv) {
	case CAPPING_LV_IDX_LV1:
		break;
	case CAPPING_LV_IDX_LV2:
		if (vr_idx == CAPPING_VR_IDX_MEDHA0) {
			tmp_idx = ADC_IDX_MEDHA0_1;
		} else if (vr_idx == CAPPING_VR_IDX_MEDHA1) {
			tmp_idx = ADC_IDX_MEDHA1_1;
		}
		break;
	case CAPPING_LV_IDX_LV3:
		if (vr_idx == CAPPING_VR_IDX_MEDHA0) {
			tmp_idx = ADC_IDX_MEDHA0_2;
		} else if (vr_idx == CAPPING_VR_IDX_MEDHA1) {
			tmp_idx = ADC_IDX_MEDHA1_2;
		}
		break;
	default:
		break;
	}

	if (power_capping_info.source == CAPPING_SOURCE_VR) {
		return get_adc_averge_val(tmp_idx);
	} else if (power_capping_info.source == CAPPING_SOURCE_ADC) {
		return (uint16_t)get_adc_vr_pwr(tmp_idx);
	}

	return 0;
}

uint8_t get_power_capping_method()
{
	return power_capping_info.method;
}

void set_power_capping_method(uint8_t value)
{
	if (value >= CAPPING_M_MAX) {
		LOG_ERR("Wrong method %d", value);
		return;
	}

	power_capping_info.method = value;

	// for credit base, we don't assert LV2 and LV3
	uint8_t data = 0;
	if (!plat_read_cpld(CPLD_OFFSET_POWER_CLAMP, &data, 1)) {
		LOG_ERR("Can't r cpld offset 0x%02x", CPLD_OFFSET_POWER_CLAMP);
	}

	if (value == CAPPING_M_LOOK_UP_TABLE) {
		data = (data & 0x09) | 0x06;
		if (!plat_write_cpld(CPLD_OFFSET_POWER_CLAMP, &data)) {
			LOG_ERR("Can't w cpld offset 0x%02x", CPLD_OFFSET_POWER_CLAMP);
		}
	} else if (value == CAPPING_M_CREDIT_BASE) {
		data = (data & 0x09);
		if (!plat_write_cpld(CPLD_OFFSET_POWER_CLAMP, &data)) {
			LOG_ERR("Can't w cpld offset 0x%02x", CPLD_OFFSET_POWER_CLAMP);
		} else {
			prev_set_ucr_status = 0;
		}
	}
}

uint8_t get_power_capping_source()
{
	return power_capping_info.source;
}

void set_power_capping_source(uint8_t value)
{
	if (value >= CAPPING_SOURCE_MAX) {
		LOG_ERR("Wrong source %d", value);
		return;
	}
	power_capping_info.source = value;

	// reset value
	adc_poll_init();

	pldm_sensor_info *table = NULL;
	uint8_t i = 0;
	uint8_t j = 0;
	int count = 0;
	uint8_t volt_rate = 0;
	uint8_t powr_rate = 0;

	// set polling rate.  VR: PWR is quick.  ADC: VOLT is quick.
	if (value == CAPPING_SOURCE_VR) {
		volt_rate = 1;
		powr_rate = 0;
	} else if (value == CAPPING_SOURCE_ADC) {
		volt_rate = 0;
		powr_rate = 1;
	}

	table = plat_pldm_sensor_load(QUICK_VR_SENSOR_THREAD_ID);
	if (table == NULL) {
		LOG_ERR("No table: %d", QUICK_VR_SENSOR_THREAD_ID);
		return;
	}
	count = plat_pldm_sensor_get_sensor_count(QUICK_VR_SENSOR_THREAD_ID);
	if (count < 0) {
		LOG_ERR("0 sensor count: %d", QUICK_VR_SENSOR_THREAD_ID);
		return;
	}

	for (i = 0; i < ARRAY_SIZE(volt_sensor_id_list); i++) {
		uint8_t sensor_id = volt_sensor_id_list[i];
		for (j = 0; j < count; j++) {
			if (table[j].pldm_sensor_cfg.num == sensor_id) {
				table[j].pdr_numeric_sensor.update_interval = (real32_t)volt_rate;
				break;
			}
		}
		if (j == count) {
			LOG_ERR("Can't find sensor: 0x%02x", sensor_id);
		}
	}

	for (i = 0; i < ARRAY_SIZE(powr_sensor_id_list); i++) {
		uint8_t sensor_id = powr_sensor_id_list[i];
		for (j = 0; j < count; j++) {
			if (table[j].pldm_sensor_cfg.num == sensor_id) {
				table[j].pdr_numeric_sensor.update_interval = (real32_t)powr_rate;
				break;
			}
		}
		if (j == count) {
			LOG_ERR("Can't find sensor: 0x%02x", sensor_id);
		}
	}
}

uint16_t get_power_capping_current_threshold(uint8_t vr_idx)
{
	if (vr_idx >= CAPPING_VR_IDX_MAX) {
		LOG_ERR("Wrong vr idx %d", vr_idx);
		return 0;
	}

	return power_capping_info.current_threshold[vr_idx];
}

uint16_t get_power_capping_time_w(uint8_t vr_idx, uint8_t lv)
{
	if (vr_idx >= CAPPING_VR_IDX_MAX) {
		LOG_ERR("Wrong vr idx %d", vr_idx);
		return 0;
	}

	if (lv >= CAPPING_LV_IDX_MAX) {
		LOG_ERR("Wrong vr lv %d", lv);
		return 0;
	}

	return power_capping_info.time_w[vr_idx][lv];
}

void set_power_capping_time_w(uint8_t vr_idx, uint8_t lv, uint16_t value)
{
	if (vr_idx >= CAPPING_VR_IDX_MAX) {
		LOG_ERR("Wrong vr idx %d", vr_idx);
		return;
	}

	if (lv >= CAPPING_LV_IDX_MAX) {
		LOG_ERR("Wrong vr lv %d", lv);
		return;
	}

	if (value < ADC_AVERGE_TIMES_MIN && value > ADC_AVERGE_TIMES_MAX) {
		LOG_ERR("Wrong value %d", value);
		return;
	}

	uint8_t tmp_idx = 0;
	uint8_t data = 0;
	switch (lv) {
	case CAPPING_LV_IDX_LV1:
		if (find_cpld_lv1_time_window_idx_by_value(&tmp_idx, value)) {
			data = (tmp_idx << 5);
			if (!plat_write_cpld(CPLD_OFFSET_POWER_CAPPING_LV1_TIME, &data)) {
				LOG_ERR("can't w cpld offset %d",
					CPLD_OFFSET_POWER_CAPPING_LV1_TIME);
				return;
			}
		} else {
			LOG_ERR("can't find lv1 time by %d", value);
			return;
		}
		break;
	case CAPPING_LV_IDX_LV2:
		if (vr_idx == CAPPING_VR_IDX_MEDHA0) {
			adc_set_averge_times(ADC_IDX_MEDHA0_1, value);
		} else if (vr_idx == CAPPING_VR_IDX_MEDHA1) {
			adc_set_averge_times(ADC_IDX_MEDHA1_1, value);
		}
		break;
	case CAPPING_LV_IDX_LV3:
		if (vr_idx == CAPPING_VR_IDX_MEDHA0) {
			adc_set_averge_times(ADC_IDX_MEDHA0_2, value);
		} else if (vr_idx == CAPPING_VR_IDX_MEDHA1) {
			adc_set_averge_times(ADC_IDX_MEDHA1_2, value);
		}
		break;
	default:
		break;
	}

	if (lv == CAPPING_LV_IDX_LV1) {
		power_capping_info.time_w[CAPPING_VR_IDX_MEDHA0][lv] = value;
		power_capping_info.time_w[CAPPING_VR_IDX_MEDHA1][lv] = value;
	} else {
		power_capping_info.time_w[vr_idx][lv] = value;
	}
}

uint16_t get_power_capping_threshold(uint8_t vr_idx, uint8_t lv)
{
	if (vr_idx >= CAPPING_VR_IDX_MAX) {
		LOG_ERR("Wrong vr idx %d", vr_idx);
		return 0;
	}

	if (lv >= CAPPING_LV_IDX_MAX) {
		LOG_ERR("Wrong vr lv %d", lv);
		return 0;
	}

	return power_capping_info.threshold[vr_idx][lv];
}

void set_power_capping_threshold(uint8_t vr_idx, uint8_t lv, uint16_t value)
{
	if (vr_idx >= CAPPING_VR_IDX_MAX) {
		LOG_ERR("Wrong vr idx %d", vr_idx);
		return;
	}

	if (lv >= CAPPING_LV_IDX_MAX) {
		LOG_ERR("Wrong vr lv %d", lv);
		return;
	}

	switch (lv) {
	case CAPPING_LV_IDX_LV1:
		if (vr_idx == CAPPING_VR_IDX_MEDHA0) {
			if (!set_power_capping_vr_oc_warn_limit(ADC_IDX_MEDHA0_1, value)) {
				return;
			}
		} else if (vr_idx == CAPPING_VR_IDX_MEDHA1) {
			if (!set_power_capping_vr_oc_warn_limit(ADC_IDX_MEDHA1_1, value)) {
				return;
			}
		}
		break;
	case CAPPING_LV_IDX_LV2:
		if (vr_idx == CAPPING_VR_IDX_MEDHA0) {
			set_adc_ucr(ADC_IDX_MEDHA0_1, value);
		} else if (vr_idx == CAPPING_VR_IDX_MEDHA1) {
			set_adc_ucr(ADC_IDX_MEDHA1_1, value);
		}
		break;
	case CAPPING_LV_IDX_LV3:
		if (vr_idx == CAPPING_VR_IDX_MEDHA0) {
			set_adc_ucr(ADC_IDX_MEDHA0_2, value);
		} else if (vr_idx == CAPPING_VR_IDX_MEDHA1) {
			set_adc_ucr(ADC_IDX_MEDHA1_2, value);
		}
		break;
	default:
		break;
	}

	power_capping_info.threshold[vr_idx][lv] = value;
}

void power_capping_handler(void *p1, void *p2, void *p3)
{
	while (1) {
		k_msleep(1);

		if (get_power_capping_method() == CAPPING_M_LOOK_UP_TABLE) {
			uint8_t final_ucr_status = get_final_ucr_status();
			if (prev_set_ucr_status != final_ucr_status) {
				uint8_t data = 0;
				if (!plat_read_cpld(CPLD_OFFSET_POWER_CLAMP, &data, 1)) {
					continue;
				}

				data = (data & 0x0F) | final_ucr_status;
				if (plat_write_cpld(CPLD_OFFSET_POWER_CLAMP, &data)) {
					prev_set_ucr_status = final_ucr_status;
				}
			}
		}
	}
}

void plat_power_capping_init()
{
	// sync avg_times
	uint8_t data = 0;
	if (plat_read_cpld(CPLD_OFFSET_POWER_CAPPING_LV1_TIME, &data, 1)) {
		uint8_t idx = data >> 5;
		if (idx < CPLD_LV1_TIME_WINDOW_NUM) {
			power_capping_info.time_w[CAPPING_VR_IDX_MEDHA0][CAPPING_LV_IDX_LV1] =
				cpld_lv1_time_window_list[idx];
			power_capping_info.time_w[CAPPING_VR_IDX_MEDHA1][CAPPING_LV_IDX_LV1] =
				cpld_lv1_time_window_list[idx];
		} else {
			LOG_ERR("invalid cpld lv1 time data 0x%02x", data);
		}
	} else {
		LOG_ERR("can't r cpld offset %d", CPLD_OFFSET_POWER_CAPPING_LV1_TIME);
	}

	power_capping_info.time_w[CAPPING_VR_IDX_MEDHA0][CAPPING_LV_IDX_LV2] =
		get_adc_averge_times(ADC_IDX_MEDHA0_1);
	power_capping_info.time_w[CAPPING_VR_IDX_MEDHA1][CAPPING_LV_IDX_LV2] =
		get_adc_averge_times(ADC_IDX_MEDHA1_1);

	power_capping_info.time_w[CAPPING_VR_IDX_MEDHA0][CAPPING_LV_IDX_LV3] =
		get_adc_averge_times(ADC_IDX_MEDHA0_2);
	power_capping_info.time_w[CAPPING_VR_IDX_MEDHA1][CAPPING_LV_IDX_LV3] =
		get_adc_averge_times(ADC_IDX_MEDHA1_2);

	// sync threshold
	add_sync_oc_warn_to_work();

	power_capping_info.threshold[CAPPING_VR_IDX_MEDHA0][CAPPING_LV_IDX_LV2] =
		get_adc_ucr(ADC_IDX_MEDHA0_1);
	power_capping_info.threshold[CAPPING_VR_IDX_MEDHA1][CAPPING_LV_IDX_LV2] =
		get_adc_ucr(ADC_IDX_MEDHA1_1);

	power_capping_info.threshold[CAPPING_VR_IDX_MEDHA0][CAPPING_LV_IDX_LV3] =
		get_adc_ucr(ADC_IDX_MEDHA0_2);
	power_capping_info.threshold[CAPPING_VR_IDX_MEDHA1][CAPPING_LV_IDX_LV3] =
		get_adc_ucr(ADC_IDX_MEDHA1_2);

	// init set capping to 0
	if (plat_read_cpld(CPLD_OFFSET_POWER_CLAMP, &data, 1)) {
		data = (data & 0x0F);
		plat_write_cpld(CPLD_OFFSET_POWER_CLAMP, &data);
	}

	set_power_capping_source(CAPPING_SOURCE_VR);

	k_thread_create(&power_capping_thread, power_capping_thread_stack, POWER_CAPPING_STACK_SIZE,
			power_capping_handler, NULL, NULL, NULL, CONFIG_MAIN_THREAD_PRIORITY, 0,
			K_MSEC(3000));

	k_thread_name_set(&power_capping_thread, "power_capping");
}
