/* SPDX-License-Identifier: Apache-2.0 */
/*
 * mbedTLS user config for the libspdm build, layered on top of libspdm's
 * bundled include/mbedtls/libspdm_mbedtls_config.h (included last by
 * mbedtls/build_info.h).
 *
 * Zephyr + picolibc: mbedtls's bundled platform_util.c recognises no
 * POSIX clock path and falls through to
 *   #error "No mbedtls_ms_time available"
 * The MCXN947 EVK also has no wired RTC. Route both time hooks to a
 * fixed epoch plus kernel uptime (src/plat_mbedtls_platform.c) - enough
 * for deterministic x509 validity math on the device cert chain.
 */
#ifndef LIBSPDM_MBEDTLS_USER_CONFIG_H
#define LIBSPDM_MBEDTLS_USER_CONFIG_H

#include <time.h>

time_t plat_mbedtls_time(time_t *timer);

#define MBEDTLS_PLATFORM_MS_TIME_ALT
#define MBEDTLS_PLATFORM_TIME_MACRO  plat_mbedtls_time

#endif /* LIBSPDM_MBEDTLS_USER_CONFIG_H */
