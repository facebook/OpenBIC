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

#include <logutil.h>

bool set_single_log_level(char *log_device_name, uint16_t data)
{
	if (log_device_name == NULL)
		return false;

	if (data > LOG_LEVEL_DBG)
		return false;

	int level = data;
	int backend_cnt = log_backend_count_get();
	for (int i = 0; i < backend_cnt; i++) {
		const struct log_backend *backend = log_backend_get(i);
		printk("Backend %d: %s \n", i, backend->name);

		for (int j = 0; j < log_sources_count(); j++) {
			const char *name = log_name_get(j);
			if (strcmp(name, log_device_name) != 0)
				continue;

			int dynamic_lvl = log_filter_get(backend, CONFIG_LOG_DOMAIN_ID, j, true);
			int compiled_lvl = log_filter_get(backend, CONFIG_LOG_DOMAIN_ID, j, false);
			printk("log name %s: dynamic %d, compiled %d\n", name, dynamic_lvl,
			       compiled_lvl);

			log_filter_set(backend, CONFIG_LOG_DOMAIN_ID, j, level);
		}
	}

	return true;
}
bool set_all_log_level(uint16_t data)
{
	/*	LOG_LEVEL_NONE 0U
		LOG_LEVEL_ERR  1U
		LOG_LEVEL_WRN  2U
		LOG_LEVEL_INF  3U
		LOG_LEVEL_DBG  4U
	*/
	if (data > LOG_LEVEL_DBG)
		return false;

	int level = data;
	int backend_cnt = log_backend_count_get();
	for (int i = 0; i < backend_cnt; i++) {
		const struct log_backend *backend = log_backend_get(i);
		printk("Backend %d: %s \n", i, backend->name);

		for (int j = 0; j < log_sources_count(); j++) {
			const char *name = log_name_get(j);
			int dynamic_lvl = log_filter_get(backend, CONFIG_LOG_DOMAIN_ID, j, true);
			int compiled_lvl = log_filter_get(backend, CONFIG_LOG_DOMAIN_ID, j, false);
			printk("log name %s: dynamic %d, compiled %d\n", name, dynamic_lvl,
			       compiled_lvl);

			log_filter_set(backend, CONFIG_LOG_DOMAIN_ID, j, level);
		}
	}

	return true;
}