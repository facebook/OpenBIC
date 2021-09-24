#ifndef WORKER_H
#define WORKER_H

#include "cmsis_os2.h"


typedef void (*PlatForm_Worker)(uint32_t, uint32_t);

typedef struct 
{
        PlatForm_Worker Handler;
        osPriority_t Priority;
        uint32_t u32_arg0;
        uint32_t u32_arg1;
} work_cfg_t;

void util_init_worker(void);
osStatus_t Work_add(work_cfg_t,uint32_t);

#endif
