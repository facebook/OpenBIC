/*
 * Copyright (c) Facebook, Inc. and its affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include "cmsis_os2.h"
#include "board_device.h"
#include "objects.h"
#include "worker.h"

osThreadId_t tid_BelowNormalWorker;
osThreadAttr_t tattr_BelowNormalWorker;

osMessageQueueId_t BelowNormal_worker_Queue = NULL;

uint8_t func_work_add(void *fptr, osPriority_t func_priority, uint32_t arg0, uint32_t arg1) {
  uint32_t no_wait = 0;
  work_cfg_t work;

  work.Handler = fptr;
  work.Priority = func_priority;
  work.u32_arg0 = arg0;
  work.u32_arg1 = arg1;

  Work_add(work, no_wait);
}

osStatus_t Work_add(work_cfg_t work, uint32_t timeout)
{
  osStatus_t status;

	if(1){ //if(Priority == osPriorityBelowNormal)
    status = osMessageQueuePut(BelowNormal_worker_Queue, &(work), 0, timeout);
    if(status != osOK ) {
      printf("work: add fail with os status: %#x", status);
      return false;
    }
    return true;
	}
	return false;
}

static void BelowNormal_Worker0(void *argv)
{
	work_cfg_t work;

	while(1)
	{
    if(osMessageQueueGetSpace(tid_BelowNormalWorker) == 0) {
      printf("BelowNormal_Worker0 is full\n");
    }

		osMessageQueueGet( BelowNormal_worker_Queue, &( work ), 0,osWaitForever );
		work.Handler(work.u32_arg0, work.u32_arg1);
    osThreadYield();
	}
}

void util_init_worker(void)
{

	BelowNormal_worker_Queue = osMessageQueueNew(
			/* The number of items the queue can hold. */
			10,
			/* Size of each item is big enough to hold the
			   whole structure. */
			sizeof( work_cfg_t ),
			/*message queue attributes; NULL: default values.*/
			NULL );	

	tattr_BelowNormalWorker.name = "BelowNormal_Worker0";
	tattr_BelowNormalWorker.priority = osPriorityBelowNormal;
	tattr_BelowNormalWorker.stack_size = 0x1000;

	tid_BelowNormalWorker = osThreadNew(BelowNormal_Worker0, NULL, &tattr_BelowNormalWorker);
}

