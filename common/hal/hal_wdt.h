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

#define WDT_DEVICE_NAME "wdt2"
#define WDT_TIMEOUT (15 * 1000) // 15s
#define WDT_FEED_DELAY_MS (10 * 1000) // 10s

#define WDT_THREAD_STACK_SIZE 256

void wdt_init();
void wdt_handler(void *arug0, void *arug1, void *arug2);
void set_wdt_continue_feed(bool value);
#endif
