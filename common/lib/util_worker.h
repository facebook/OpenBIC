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

#ifndef SMC_WORKER_H
#define SMC_WORKER_H

#include <stdint.h>

#define MAX_WORK_NAME_LEN 32

/* A structure used to submit work. */
typedef struct {
	/* The function to be added to worker.
   * Which must with two parameters,
   * first one is void*, second one is uint32_t.
   */
	void (*fn)(void *, uint32_t);

	/* First parameter of the function. */
	void *ptr_arg;

	/* Second parameter of the function. */
	uint32_t ui32_arg;

	/* Time to waits before executing function,
   * set to 0 if no delay is required.
   */
	uint32_t delay_ms;

	/* Work name. */
	char name[MAX_WORK_NAME_LEN];
} worker_job;

uint8_t get_work_count();
int add_work(worker_job *);
void init_worker();

#endif
