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

#include <zephyr.h>

/* semaphore CPLD polling semaphore */
void plat_ragular_cpld_polling_sem_handler(struct k_timer *timer);
void plat_activate_cpld_polling_semaphore_timer();
void plat_wait_for_cpld_polling_trigger(void);
void plat_trigger_cpld_polling(void);

/* Timer for dc status checking */
void plat_update_ubc_status(void);
bool plat_get_ubc_status(void);

/* Timer for power sequence event handling */
void plat_handle_pwr_sequence_event(void);