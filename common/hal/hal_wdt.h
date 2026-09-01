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

#ifndef HAL_WDT_H
#define HAL_WDT_H

#if defined(CONFIG_WDT_ASPEED)
#define WDT_DEVICE_NAME "wdt2"
#elif (CONFIG_WDT_NPCM4XX)
#define WDT_DEVICE_NAME "TWD_0"
#elif defined(CONFIG_WDT_MCUX_WWDT)
/* NXP MCX-N9XX-EVK: no string-name device binding is used for this
 * board (see hal_wdt.c) - device_get_binding() by name doesn't apply
 * cleanly to devicetree-only boards, so wdt_init() uses
 * DEVICE_DT_GET(DT_ALIAS(watchdog0)) directly instead. WDT_DEVICE_NAME
 * is kept only as a human-readable string for log messages.
 */
#define WDT_DEVICE_NAME "wwdt0"
#else /* defined(CONFIG_WDT_ASPEED) */
#endif /* defined(CONFIG_WDT_ASPEED) */

#define WDT_TIMEOUT (15 * 1000) // 15s
#define WDT_FEED_DELAY_MS (10 * 1000) // 10s

#define WDT_THREAD_STACK_SIZE 256

void wdt_init();
void wdt_handler(void *arug0, void *arug1, void *arug2);
void set_wdt_continue_feed(bool value);
#endif
