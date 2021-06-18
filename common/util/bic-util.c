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

#include "common.h"
#include "objects.h"
#include <stdio.h>
#include <stdlib.h>
#include "cmsis_os.h"
#include "wdt_aspeed.h"
#include "bic-util.h"
#include "timer.h"

void bic_set_WDT_rst(void) {
  const uint8_t WDT_RESET_CPU = 1;

  wdt_set_timeout(WDT_RESET_CPU);
  wdt_reload();
  wdt_enable();
}

void bic_warm_reset(void) {
  osTimerId_t bic_reset_timer_id;
  osStatus_t  status;

  bic_reset_timer_id = osTimerNew(bic_set_WDT_rst, osTimerOnce, NULL, NULL);
  if( bic_reset_timer_id == NULL ) {
    printf("bic_warm_reset timer create fail\n");
  } else {
    status = osTimerStart(bic_reset_timer_id, time_500_ms);
    if (status != osOK) {
      printf("bic_warm_reset timer start fail\n");
    }
  }
  return;
}

