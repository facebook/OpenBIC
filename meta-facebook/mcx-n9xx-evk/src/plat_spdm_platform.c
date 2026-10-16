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
 *
 * Zephyr implementations of the libspdm platform hooks that its
 * os_stub only ships as non-functional "arm sample" stubs
 * (rng_arm_sample.c returns true without filling the buffer;
 * time_sample.c asserts). These are compiled in place of those.
 */

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/kernel.h>
#include <zephyr/random/random.h>

/* rnglib.h */
bool libspdm_get_random_number_64(uint64_t *rand_data)
{
	if (rand_data == NULL) {
		return false;
	}
	sys_rand_get(rand_data, sizeof(*rand_data));
	return true;
}

/* platform_lib time */
void libspdm_sleep(uint64_t microseconds)
{
	while (microseconds > 0) {
		uint32_t chunk = (microseconds > 1000000ULL) ? 1000000U : (uint32_t)microseconds;

		k_busy_wait(chunk);
		microseconds -= chunk;
	}
}

/* responder watchdoglib.h - this responder has no per-request watchdog
 * (the SPDM state machine here is synchronous and short); the SoC-level
 * watchdog (hal_wdt) already covers a wedged firmware. No-ops. */
bool libspdm_start_watchdog(uint32_t session_id, uint16_t timeout)
{
	ARG_UNUSED(session_id);
	ARG_UNUSED(timeout);
	return true;
}

bool libspdm_stop_watchdog(uint32_t session_id)
{
	ARG_UNUSED(session_id);
	return true;
}

bool libspdm_reset_watchdog(uint32_t session_id)
{
	ARG_UNUSED(session_id);
	return true;
}
