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
