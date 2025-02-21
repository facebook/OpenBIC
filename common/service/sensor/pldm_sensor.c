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

#include <zephyr.h>
#include <logging/log.h>
#include "libutil.h"
#include "pldm_monitor.h"
#include "plat_def.h"
#include "sensor.h"

#ifdef ENABLE_PLDM_SENSOR
#include "plat_pldm_sensor.h"
#include "pldm_sensor.h"

LOG_MODULE_REGISTER(pldm_sensor);

static struct k_thread pldm_sensor_polling_thread[MAX_SENSOR_THREAD_ID];
static k_tid_t pldm_sensor_polling_tid[MAX_SENSOR_THREAD_ID];
K_THREAD_STACK_EXTERN(pldm_sensor_poll_stack);
K_THREAD_STACK_ARRAY_DEFINE(pldm_sensor_poll_stacks, MAX_SENSOR_THREAD_ID,
			    PLDM_SENSOR_POLL_STACK_SIZE);

pldm_sensor_thread *pldm_sensor_thread_list;
pldm_sensor_info *pldm_sensor_list[MAX_SENSOR_THREAD_ID];

__weak pldm_sensor_thread *plat_pldm_sensor_load_thread()
{
	return NULL;
}

__weak pldm_sensor_info *plat_pldm_sensor_load(int thread_id)
{
	return NULL;
}

__weak int plat_pldm_sensor_get_sensor_count(int thread_id)
{
	return -1;
}

bool pldm_sensor_is_interval_ready(pldm_sensor_info *pldm_sensor_list)
{
	CHECK_NULL_ARG_WITH_RETURN(pldm_sensor_list, false);

	if (pldm_sensor_list->update_time == 0) { // First time to read sensor
		return true;
	}

	uint32_t current_time = 0, diff_time = 0;

	current_time = k_uptime_get_32() / 1000;
	diff_time = current_time - pldm_sensor_list->update_time;

	if (pldm_sensor_list->pdr_numeric_sensor.update_interval > diff_time) {
		return false;
	}

	return true;
}

int pldm_sensor_get_info_via_sensor_thread_and_sensor_pdr_index(
	int thread_id, int sensor_pdr_index, uint16_t *sensor_id, real32_t *resolution,
	real32_t *offset, int8_t *unit_modifier, real32_t *poll_time, uint32_t *update_time,
	uint8_t *type, int *cache, uint8_t *cache_status, char *check_access)
{
	CHECK_NULL_ARG_WITH_RETURN(sensor_id, -1);
	CHECK_NULL_ARG_WITH_RETURN(resolution, -1);
	CHECK_NULL_ARG_WITH_RETURN(offset, -1);
	CHECK_NULL_ARG_WITH_RETURN(unit_modifier, -1);
	CHECK_NULL_ARG_WITH_RETURN(poll_time, -1);
	CHECK_NULL_ARG_WITH_RETURN(type, -1);
	CHECK_NULL_ARG_WITH_RETURN(cache, -1);
	CHECK_NULL_ARG_WITH_RETURN(cache_status, -1);
	CHECK_NULL_ARG_WITH_RETURN(check_access, -1);

	int pldm_sensor_count = plat_pldm_sensor_get_sensor_count(thread_id);
	if (sensor_pdr_index >= pldm_sensor_count) {
		return -1;
	}

	// Get from numeric sensor PDR
	*sensor_id = pldm_sensor_list[thread_id][sensor_pdr_index].pdr_numeric_sensor.sensor_id;
	*resolution = pldm_sensor_list[thread_id][sensor_pdr_index].pdr_numeric_sensor.resolution;
	*offset = pldm_sensor_list[thread_id][sensor_pdr_index].pdr_numeric_sensor.offset;
	*unit_modifier =
		pldm_sensor_list[thread_id][sensor_pdr_index].pdr_numeric_sensor.unit_modifier;
	*poll_time =
		pldm_sensor_list[thread_id][sensor_pdr_index].pdr_numeric_sensor.update_interval;

	// Get from update time
	*update_time = pldm_sensor_list[thread_id][sensor_pdr_index].update_time;

	// Get from sensor config
	*type = pldm_sensor_list[thread_id][sensor_pdr_index].pldm_sensor_cfg.type;
	*cache = pldm_sensor_list[thread_id][sensor_pdr_index].pldm_sensor_cfg.cache;
	*cache_status = pldm_sensor_list[thread_id][sensor_pdr_index].pldm_sensor_cfg.cache_status;
	*check_access =
		(pldm_sensor_list[thread_id][sensor_pdr_index].pldm_sensor_cfg.access_checker(
			 *sensor_id) ?
			 'O' :
			 'X');

	return 0;
}

int pldm_sensor_get_info_via_sensor_id(uint16_t sensor_id, float *resolution, float *offset,
				       int8_t *unit_modifier, int *cache,
				       uint8_t *sensor_operational_state)
{
	CHECK_NULL_ARG_WITH_RETURN(resolution, -1);
	CHECK_NULL_ARG_WITH_RETURN(offset, -1);
	CHECK_NULL_ARG_WITH_RETURN(unit_modifier, -1);
	CHECK_NULL_ARG_WITH_RETURN(cache, -1);
	CHECK_NULL_ARG_WITH_RETURN(sensor_operational_state, -1);

	int t_id = 0, s_id = 0, pldm_sensor_count = 0;

	for (t_id = 0; t_id < MAX_SENSOR_THREAD_ID; t_id++) {
		pldm_sensor_count = plat_pldm_sensor_get_sensor_count(t_id);
		for (s_id = 0; s_id < pldm_sensor_count; s_id++) {
			if (sensor_id ==
			    pldm_sensor_list[t_id][s_id].pdr_numeric_sensor.sensor_id) {
				// Get from numeric sensor PDR
				*resolution =
					pldm_sensor_list[t_id][s_id].pdr_numeric_sensor.resolution;
				*offset = pldm_sensor_list[t_id][s_id].pdr_numeric_sensor.offset;
				*unit_modifier = pldm_sensor_list[t_id][s_id]
							 .pdr_numeric_sensor.unit_modifier;
				// Get from sensor config
				*cache = pldm_sensor_list[t_id][s_id].pldm_sensor_cfg.cache;
				*sensor_operational_state =
					pldm_sensor_list[t_id][s_id].pldm_sensor_cfg.cache_status;
				return 0;
			}
		}
	}

	return -1;
}

uint8_t pldm_sensor_get_reading_from_cache(uint16_t sensor_id, int *reading,
					   uint8_t *sensor_operational_state)
{
	CHECK_NULL_ARG_WITH_RETURN(reading, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(sensor_operational_state, PLDM_ERROR);

	float sensor_reading = 0, decimal = 0, resolution = 0, offset = 0;
	int cache_reading = 0;
	int8_t unit_modifier = 0;
	int16_t integer = 0;

	if (pldm_sensor_get_info_via_sensor_id(sensor_id, &resolution, &offset, &unit_modifier,
					       &cache_reading, sensor_operational_state) != 0) {
		// Couldn't find sensor id in pldm_sensor_list
		return PLDM_PLATFORM_INVALID_SENSOR_ID;
	}

	if (resolution == 0) {
		// The value of resolution couldn't be 0
		return PLDM_ERROR_INVALID_DATA;
	}

	// Convert two byte integer, two byte decimal sensor format to float
	integer = cache_reading & 0xffff;
	decimal = (float)(cache_reading >> 16) / 1000.0;

	if (integer >= 0) {
		sensor_reading = (float)integer + decimal;
	} else {
		sensor_reading = (float)integer - decimal;
	}

	// Y = converted reading in Units in BMC
	// X = sensor reading report to BMC
	// Y = (X * resolution + offset ) * power (10, unit_modifier)
	// X = (Y * power (10, -1 * unit_modifier) - offset ) / resolution
	*reading = (int)((sensor_reading * power(10, -1 * unit_modifier) - offset) / resolution);

	return PLDM_SUCCESS;
}

void pldm_sensor_get_reading(sensor_cfg *pldm_sensor_cfg, uint32_t *update_time,
			     int pldm_sensor_count, int thread_id, int sensor_num)
{
	CHECK_NULL_ARG(pldm_sensor_list);
	CHECK_NULL_ARG(update_time);

	int reading = 0;
	uint8_t status = SENSOR_UNSPECIFIED_ERROR;

	if (pldm_sensor_cfg->access_checker(sensor_num) != true) {
		pldm_sensor_cfg->cache_status = PLDM_SENSOR_UNAVAILABLE;
		return;
	}

	if (pldm_sensor_cfg->pre_sensor_read_hook) {
		if (!pldm_sensor_cfg->pre_sensor_read_hook(pldm_sensor_cfg,
							   pldm_sensor_cfg->pre_sensor_read_args)) {
			pldm_sensor_cfg->cache_status = PLDM_SENSOR_FAILED;
			*update_time = (k_uptime_get_32() / 1000);
			LOG_DBG("Failed to pre read sensor_num 0x%x of thread %d", sensor_num,
				thread_id);
			return;
		}
	}

	// TODO: Check device ready

	if (pldm_sensor_cfg->read) {
		status = pldm_sensor_cfg->read(pldm_sensor_cfg, &reading);

		if (pldm_sensor_cfg->type == sensor_dev_apml_mailbox) {
			*update_time = (k_uptime_get_32() / 1000);

			if (pldm_sensor_cfg->post_sensor_read_hook) {
				if (!pldm_sensor_cfg->post_sensor_read_hook(
					    pldm_sensor_cfg, pldm_sensor_cfg->post_sensor_read_args,
					    NULL)) {
					LOG_DBG("Failed to pose read sensor_num 0x%x of thread %d",
						sensor_num, thread_id);
				}
			}
			return;
		}

		if ((status != SENSOR_READ_SUCCESS) && (status != SENSOR_READ_ACUR_SUCCESS)) {
			pldm_sensor_cfg->cache_status = PLDM_SENSOR_FAILED;
			*update_time = (k_uptime_get_32() / 1000);
			LOG_ERR("Failed to read sensor_num 0x%x of thread %d, status0x%x",
				sensor_num, thread_id, status);

			// If sensor read fails, let the reading argument in the post_sensor_read_hook function to NULL.
			// All post_sensor_read_hook function define in each platform should check reading
			// whether is NULL to do the corresponding thing. (Ex: mutex_unlock)
			if (pldm_sensor_cfg->post_sensor_read_hook) {
				if (!pldm_sensor_cfg->post_sensor_read_hook(
					    pldm_sensor_cfg, pldm_sensor_cfg->post_sensor_read_args,
					    NULL)) {
					LOG_DBG("Failed to pose read sensor_num 0x%x of thread %d",
						sensor_num, thread_id);
				}
			}
			return;
		}
	}

	if (pldm_sensor_cfg->post_sensor_read_hook) {
		if (!pldm_sensor_cfg->post_sensor_read_hook(
			    pldm_sensor_cfg, pldm_sensor_cfg->post_sensor_read_args, &reading)) {
			pldm_sensor_cfg->cache_status = PLDM_SENSOR_FAILED;
			*update_time = (k_uptime_get_32() / 1000);
			LOG_DBG("Failed to pose read sensor_num 0x%x of thread %d", sensor_num,
				thread_id);
			return;
		}
	}

	*update_time = (k_uptime_get_32() / 1000);
	pldm_sensor_cfg->cache = reading;
	pldm_sensor_cfg->cache_status = PLDM_SENSOR_ENABLED;
}

int pldm_sensor_polling_pre_check(pldm_sensor_info *pldm_snr_list, int sensor_num)
{
	CHECK_NULL_ARG_WITH_RETURN(pldm_snr_list, -1);

	int ret = 0;

	if (pldm_snr_list->pldm_sensor_cfg.access_checker(sensor_num) == false) {
		return -1;
	}

	if (pldm_snr_list->pldm_sensor_cfg.pre_sensor_read_hook) {
		if (!pldm_snr_list->pldm_sensor_cfg.pre_sensor_read_hook(
			    &pldm_snr_list->pldm_sensor_cfg,
			    pldm_snr_list->pldm_sensor_cfg.pre_sensor_read_args)) {
			pldm_snr_list->pldm_sensor_cfg.cache_status = PLDM_SENSOR_FAILED;
			return -1;
		}
	}

	ret = sensor_drive_tbl[pldm_snr_list->pldm_sensor_cfg.type].init(
		&pldm_snr_list->pldm_sensor_cfg);
	if (ret != SENSOR_INIT_SUCCESS) {
		pldm_snr_list->pldm_sensor_cfg.cache_status = PLDM_SENSOR_FAILED;
		LOG_ERR("Failed to init sensor 0x%x", pldm_snr_list->pdr_numeric_sensor.sensor_id);
	} else {
		pldm_snr_list->pdr_numeric_sensor.sensor_init = PDR_SENSOR_ENABLE;
	}

	if (pldm_snr_list->pldm_sensor_cfg.post_sensor_read_hook) {
		if (!pldm_snr_list->pldm_sensor_cfg.post_sensor_read_hook(
			    &pldm_snr_list->pldm_sensor_cfg,
			    pldm_snr_list->pldm_sensor_cfg.post_sensor_read_args, NULL)) {
			pldm_snr_list->pldm_sensor_cfg.cache_status = PLDM_SENSOR_FAILED;
			return -1;
		}
	}

	return 0;
}

int pldm_polling_sensor_reading(pldm_sensor_info *pldm_snr_list, int pldm_sensor_count,
				int thread_id, int sensor_num)
{
	CHECK_NULL_ARG_WITH_RETURN(pldm_snr_list, -1);

	if (pldm_snr_list->pldm_sensor_cfg.cache_status == PLDM_SENSOR_INITIALIZING) {
		if (pldm_sensor_polling_pre_check(&pldm_sensor_list[thread_id][sensor_num],
						  sensor_num) != 0) {
			return -1;
		}
	}

	if (pldm_snr_list->pdr_numeric_sensor.sensor_init != PDR_SENSOR_ENABLE) {
		pldm_snr_list->pldm_sensor_cfg.cache_status = PLDM_SENSOR_DISABLED;
		return -1;
	}

	if (!pldm_sensor_is_interval_ready(pldm_snr_list)) {
		return -1;
	}

	pldm_sensor_get_reading(&pldm_snr_list->pldm_sensor_cfg, &pldm_snr_list->update_time,
				pldm_sensor_count, thread_id, sensor_num);

	LOG_DBG("sensor0x%x, value0x%x, status 0x%x", pldm_snr_list->pdr_numeric_sensor.sensor_id,
		pldm_snr_list->pldm_sensor_cfg.cache, pldm_snr_list->pldm_sensor_cfg.cache_status);

	return 0;
}

void pldm_sensor_polling_handler(void *arug0, void *arug1, void *arug2)
{
	ARG_UNUSED(arug1);
	ARG_UNUSED(arug2);

	int ret = 0, sensor_num = 0;
	int thread_id = (int)arug0;
	int pldm_sensor_count = 0;

	pldm_sensor_count = plat_pldm_sensor_get_sensor_count(thread_id);
	if (pldm_sensor_count <= 0) {
		LOG_ERR("Failed to get PLDM sensor count(%d) of thread%d", pldm_sensor_count,
			thread_id);
		return;
	}

	pldm_sensor_list[thread_id] = plat_pldm_sensor_load(thread_id);
	if (pldm_sensor_list[thread_id] == NULL) {
		LOG_ERR("Failed to load PLDM sensor list of thread%d, ret%d", thread_id, ret);
		return;
	}

	while (1) {
		// Check sensor poll enable
		if (get_sensor_poll_enable_flag() == false) {
			k_msleep(PLDM_SENSOR_POLL_TIME_DEFAULT_MS);
			continue;
		}

		for (sensor_num = 0; sensor_num < pldm_sensor_count; sensor_num++) {
			if (pldm_polling_sensor_reading(&pldm_sensor_list[thread_id][sensor_num],
							pldm_sensor_count, thread_id,
							sensor_num) != 0) {
				continue;
			}
		}

		k_msleep(PLDM_SENSOR_POLL_TIME_DEFAULT_MS);
	}
}

void pldm_sensor_poll_thread_init()
{
	int i = 0;

	LOG_INF("Init PLDM sensor monitor thread");

	for (i = 0; i < MAX_SENSOR_THREAD_ID; i++) {
		pldm_sensor_polling_tid[i] =
			k_thread_create(&pldm_sensor_polling_thread[i], pldm_sensor_poll_stacks[i],
					K_THREAD_STACK_SIZEOF(pldm_sensor_poll_stacks[i]),
					pldm_sensor_polling_handler, (void *)i, NULL, NULL,
					K_PRIO_PREEMPT(1), 0, K_NO_WAIT);
		k_thread_name_set(&pldm_sensor_polling_thread[i],
				  pldm_sensor_thread_list[i].thread_name);

		if (pldm_sensor_polling_tid[i] == NULL) {
			LOG_ERR("Failed to create %s to monitor sensor",
				pldm_sensor_thread_list[i].thread_name);
		}
	}

	return;
}

void pldm_sensor_monitor_init()
{
	LOG_INF("Init PDR table");
	uint16_t pdr_size = plat_get_pdr_size(PLDM_NUMERIC_SENSOR_PDR);
	if (pdr_size != 0) {
		if (pdr_init() != 0) {
			LOG_ERR("Falied to initialize Platform PDR table");
			return;
		}
	} else {
		LOG_ERR("Platform PDR table not configured");
	}

	pldm_sensor_thread_list = plat_pldm_sensor_load_thread(pldm_sensor_thread_list);
	if (!pldm_sensor_thread_list) {
		LOG_ERR("Failed to load PLDM sensor thread list");
		return;
	}

	pldm_sensor_poll_thread_init();

	return;
}

#endif
