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

#ifndef PLAT_GPIO_H
#define PLAT_GPIO_H

/* Sets up interrupt-driven monitoring of the mon0 GPIO (SW2 on this EVK,
 * standing in for a real platform status signal - see plat_gpio.c).
 * Logs every state transition; current state is also queryable live via
 * the "plat gpio mon0" shell command.
 */
void plat_gpio_init(void);

/* Current debounced level of the monitored GPIO: 1 = asserted, 0 = not. */
int plat_gpio_mon0_get(void);

/* Called once per debounced mon0 (SW2) transition, from the system
 * workqueue. __weak no-op by default; override to react. */
void plat_gpio_mon0_changed(int level);

#endif
