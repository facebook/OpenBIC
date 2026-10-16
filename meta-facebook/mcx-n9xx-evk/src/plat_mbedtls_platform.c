/* SPDX-License-Identifier: Apache-2.0 */
/*
 * mbedTLS platform hooks for the libspdm build. Compiled inside the
 * libspdm zephyr_library (see libspdm.cmake) so it is built with
 * MBEDTLS_CONFIG_FILE / MBEDTLS_USER_CONFIG_FILE and the mbedtls
 * include dirs.
 *
 * MCXN947 EVK has no wired RTC: anchor wall-clock time to a fixed epoch
 * plus kernel uptime. Monotonic and good enough for x509 not-before /
 * not-after checks on the embedded device cert chain and for libspdm's
 * internal timing.
 */
#include <stdint.h>
#include <time.h>

#include <zephyr/kernel.h>

#include <mbedtls/build_info.h>
#include <mbedtls/platform_time.h>

/* 2026-10-01T00:00:00Z - sits inside the embedded device cert chain's
 * validity window (notBefore 2026-08-28, notAfter 2036-08-25), so x509
 * not-before / not-after checks pass on a board with no RTC. */
#define PLAT_EPOCH_BASE 1759276800LL

time_t plat_mbedtls_time(time_t *timer)
{
	time_t now = (time_t)(PLAT_EPOCH_BASE + (k_uptime_get() / 1000));

	if (timer != NULL) {
		*timer = now;
	}
	return now;
}

mbedtls_ms_time_t mbedtls_ms_time(void)
{
	return (mbedtls_ms_time_t)k_uptime_get();
}
