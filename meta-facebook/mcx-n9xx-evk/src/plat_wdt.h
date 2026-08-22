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

#ifndef PLAT_WDT_H
#define PLAT_WDT_H

/* Installs a hardware watchdog timeout on wwdt0 and starts it. Call
 * plat_wdt_feed() periodically (e.g. from the heartbeat loop) or the
 * SoC resets. Mirrors the role of common/service/main.c's wdt_init().
 */
void plat_wdt_init(void);

/* Services the watchdog. No-op if plat_wdt_stop_feeding() was called. */
void plat_wdt_feed(void);

/* Stops feeding on purpose, for testing that the watchdog actually
 * resets the SoC on starvation (see "plat wdt starve" shell command).
 */
void plat_wdt_stop_feeding(void);

#endif
