#include <zephyr.h>
#include <stdio.h>
#include "cmsis_os2.h"
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
      printf("work: %x add fail with os status: %x", status);
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
		osMessageQueueGet( BelowNormal_worker_Queue, &( work ), 0, osWaitForever);
		work.Handler(work.u32_arg0, work.u32_arg1);
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

