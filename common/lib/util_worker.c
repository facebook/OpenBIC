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
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "util_worker.h"
#include "cmsis_os2.h"
#include "libutil.h"

#include <logging/log.h>

LOG_MODULE_REGISTER(util_worker);

#define WORKER_STACK_SIZE 10000
#define WORKER_PRIORITY CONFIG_MAIN_THREAD_PRIORITY

#define MAX_WORK_COUNT 32
#define WARN_WORK_PROC_TIME_MS 1000

K_THREAD_STACK_DEFINE(worker_stack_area, WORKER_STACK_SIZE);
K_THREAD_STACK_DEFINE(plat_worker_stack_area, WORKER_STACK_SIZE);
struct k_work_q plat_work_q;
static struct k_work_q worker_work_q;
static struct k_mutex mutex_use_count;
static uint8_t work_count;

typedef struct {
	union {
		struct k_work normal_work;
		struct k_work_delayable delay_work;
	} work;
	void (*fn)(void *, uint32_t);
	void *ptr_arg;
	uint32_t ui32_arg;
	char name[MAX_WORK_NAME_LEN];
} work_info;

static void work_handler(struct k_work *item)
{
	work_info *work_job = CONTAINER_OF(item, work_info, work);
	uint64_t fn_start_time, fn_finish_time;

	if (work_job->fn == NULL) {
		LOG_ERR("work_handler function is null");
	} else {
		fn_start_time = k_uptime_get();
		work_job->fn(work_job->ptr_arg, work_job->ui32_arg);
		fn_finish_time = k_uptime_get();

		/* Processing time too long, print warning message */
		if ((fn_finish_time - fn_start_time) > WARN_WORK_PROC_TIME_MS) {
			LOG_ERR("WARN: work %s Processing time too long, %llu ms",
				log_strdup(work_job->name), (fn_finish_time - fn_start_time));
		}
	}
	if (k_mutex_lock(&mutex_use_count, K_MSEC(1000))) {
		LOG_ERR("work_handler mutex lock fail");
		SAFE_FREE(work_job);
		return;
	}
	work_count--;
	k_mutex_unlock(&mutex_use_count);

	SAFE_FREE(work_job);
}

/* Get number of works in worker now.
 *
 * @retval number of works
 */
uint8_t get_work_count()
{
	return work_count;
}

/* Attempt to add new work to worker.
 *
 * @param job pointer to the worker_job to be added
 *
 * @retval 1 if successfully queued.
 * @retval -1 if work queue is full.
 * @retval -2 if memory allocation fail.
 * @retval -3 if mutex lock fail.
 */
int add_work(worker_job *job)
{
	if (work_count >= MAX_WORK_COUNT) {
		LOG_ERR("add_work work queue full");
		return -1;
	}

	int ret;
	work_info *new_job;
	new_job = malloc(sizeof(work_info));
	if (new_job == NULL) {
		LOG_ERR("add_work malloc fail");
		return -2;
	}
	memset(new_job, 0, sizeof(work_info));

	new_job->fn = job->fn;
	new_job->ptr_arg = job->ptr_arg;
	new_job->ui32_arg = job->ui32_arg;
	if (job->name != NULL) {
		snprintf(new_job->name, sizeof(new_job->name), "%s", job->name);
	}

	if (k_mutex_lock(&mutex_use_count, K_MSEC(1000))) {
		LOG_ERR("add_work mutex lock fail");
		SAFE_FREE(new_job);
		return -3;
	}

	if (job->delay_ms == 0) { /* no need to be delayed */
		k_work_init(&(new_job->work.normal_work), work_handler);
		ret = k_work_submit_to_queue(&worker_work_q, &(new_job->work.normal_work));
		if (ret != 1) { /* queued fail */
			LOG_ERR("add_work add work to queue fail");
			goto error;
		}
	} else { /* need to be delayed */
		k_work_init_delayable(&(new_job->work.delay_work), work_handler);
		ret = k_work_schedule_for_queue(&worker_work_q, &(new_job->work.delay_work),
						K_MSEC(job->delay_ms));
		if (ret != 1) { /* queued fail */
			LOG_ERR("add_work add work to queue fail");
			goto error;
		}
	}
	work_count++;
	k_mutex_unlock(&mutex_use_count);
	// free new_job in work_handler()
	return ret;

error:
	SAFE_FREE(new_job);
	k_mutex_unlock(&mutex_use_count);
	return ret;
}

/* Initialize worker
 *
 * Should call this function to initialize worker before use other APIs.
 * This function initialize a workqueue and mutex.
 */
void init_worker()
{
	k_work_queue_start(&worker_work_q, worker_stack_area,
			   K_THREAD_STACK_SIZEOF(worker_stack_area), WORKER_PRIORITY, NULL);
	k_thread_name_set(&worker_work_q.thread, "util_worker");
	k_mutex_init(&mutex_use_count);
}

/* Initialize platform work queue
 *
 * Should call this function to initialize worker before use other APIs.
 * It export the work queue for queue operation
 */
void init_plat_worker(int priority)
{
	k_work_queue_start(&plat_work_q, plat_worker_stack_area,
			   K_THREAD_STACK_SIZEOF(plat_worker_stack_area), priority, NULL);
	k_thread_name_set(&plat_work_q.thread, "plat_worker");
}
