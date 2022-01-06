#include <zephyr.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "util_worker.h"
#include "cmsis_os2.h"

#define WORKER_STACK_SIZE 10000
#define WORKER_PRIORITY osPriorityBelowNormal

#define MAX_WORK_COUNT 32
#define WARN_WORK_PROC_TIME_MS 1000

K_THREAD_STACK_DEFINE(worker_stack_area, WORKER_STACK_SIZE);
static struct k_work_q worker_work_q;
static struct k_mutex mutex_use_count;
static uint8_t work_count;

typedef struct {
  union
  {
    struct k_work normal_work;
    struct k_work_delayable delay_work;
  } work;
  void (*fn)(void*, uint32_t);
  void* ptr_arg;
  uint32_t ui32_arg;
  char name[MAX_WORK_NAME_LEN];
} work_info;


static void work_handler(struct k_work *item) {
  work_info *work_job = CONTAINER_OF(item, work_info, work);
  uint64_t fn_start_time, fn_finish_time;

  if (work_job->fn == NULL) {
    printk("work_handler function is null\n");
  } else {
    fn_start_time = k_uptime_get();
    work_job->fn(work_job->ptr_arg, work_job->ui32_arg);
    fn_finish_time = k_uptime_get();

    /* Processing time too long, print warning message */
    if ((fn_finish_time - fn_start_time) > WARN_WORK_PROC_TIME_MS) {
      printk("WARN: work %s Processing time too long, %lld ms\n", work_job->name, (fn_finish_time - fn_start_time));
    }
  }
  if (k_mutex_lock(&mutex_use_count, K_MSEC(1000))) {
    printk("work_handler mutex lock fail\n");
    free(work_job);
    return;
  }
  work_count--;
  k_mutex_unlock(&mutex_use_count);

  free(work_job);
}

uint8_t get_work_count() {
  return work_count;
}

/* Attempt to add new work to worker.
 * @retval 1 if successfully queued.
 * @retval -1 if work queue is full.
 * @retval -2 if memory allocate fail.
 * @retval -3 if mutex lock fail.
 */
int add_work(worker_job job) {
  if (work_count >= MAX_WORK_COUNT) {
    printk("add_work work queue full\n");
    return -1;
  }

  int ret;
  work_info *new_job;
  new_job = malloc(sizeof(work_info));
  if (new_job == NULL) {
    printk("add_work malloc fail\n");
    return -2;
  }
  memset(new_job, 0, sizeof(work_info));

  new_job->fn = job.fn;
  new_job->ptr_arg = job.ptr_arg;
  new_job->ui32_arg = job.ui32_arg;
  if (job.name != NULL) {
    snprintf(new_job->name, sizeof(new_job->name), "%s", job.name);
  }

  if (k_mutex_lock(&mutex_use_count, K_MSEC(1000))) {
    printk("add_work mutex lock fail\n");
    free(new_job);
    return -3;
  }

  if (job.delay_ms == 0) {
    k_work_init(&(new_job->work.normal_work), work_handler);
    ret = k_work_submit_to_queue(&worker_work_q, &(new_job->work.normal_work));
    if (ret != 1) {  /* queued fail */
      printk("add_work add work to queue fail\n");
      goto error;
    }
  } else {
    k_work_init_delayable(&(new_job->work.delay_work), work_handler);
    ret = k_work_schedule_for_queue(&worker_work_q, &(new_job->work.delay_work), K_MSEC(job.delay_ms));
    if (ret != 1) {  /* queued fail */
      printk("add_work add work to queue fail\n");
      goto error;
    }
  }
  work_count++;
  k_mutex_unlock(&mutex_use_count);
  return ret;

error:
  free(new_job);
  k_mutex_unlock(&mutex_use_count);
  return ret;
}

void init_worker() {
  k_work_queue_start(&worker_work_q, worker_stack_area, K_THREAD_STACK_SIZEOF(worker_stack_area), WORKER_PRIORITY, NULL);
  k_thread_name_set(&worker_work_q.thread, "util_worker");
  k_mutex_init(&mutex_use_count);
}
