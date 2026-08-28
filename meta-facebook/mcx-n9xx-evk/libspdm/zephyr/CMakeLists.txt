# SPDX-License-Identifier: Apache-2.0
#
# Zephyr module glue for DMTF libspdm (the west project at
# modules/lib/libspdm). libspdm ships no Zephyr module support, so its
# protocol libs + its bundled mbedTLS + the os_stub shims are compiled
# here as one zephyr_library. Attestation-only responder - see
# ../include/plat_libspdm_config.h for the capability/algorithm trim.
#
# This file runs during Zephyr module processing (kernel mode), so
# zephyr_library() registers into ZEPHYR_LIBS and links into every
# pass. (The earlier approach - include()'d from the app CMakeLists
# after find_package(Zephyr) - hit "called in application mode" and the
# archive was silently dropped from the link.)

# meta-facebook/mcx-n9xx-evk
set(PLAT_DIR ${CMAKE_CURRENT_LIST_DIR}/../..)

set(LIBSPDM_DIR ${ZEPHYR_BASE}/../modules/lib/libspdm)
set(MBEDTLS_SRC ${LIBSPDM_DIR}/os_stub/mbedtlslib/mbedtls)

if(NOT EXISTS ${MBEDTLS_SRC}/library/aes.c)
  message(FATAL_ERROR "libspdm's mbedTLS submodule is missing: run "
          "'git -C ${LIBSPDM_DIR} submodule update --init os_stub/mbedtlslib/mbedtls'")
endif()

zephyr_library_named(libspdm)

# LIBSPDM_CONFIG must be visible to every translation unit that pulls in
# a libspdm header (spdm_lib_config.h does #include LIBSPDM_CONFIG), so
# it goes on the whole app build - including src/plat_spdm.c.
zephyr_compile_definitions(LIBSPDM_CONFIG=<plat_libspdm_config.h>)
zephyr_include_directories(
  ${LIBSPDM_DIR}/include
  ${LIBSPDM_DIR}/include/hal
  ${PLAT_DIR}/include
)

# Third-party code: don't let its warnings fail our -Werror build, and
# feed it the mbedTLS config + arch hint. The user config layers our
# no-RTC time hooks on top of libspdm's bundled mbedtls config.
#   -w           : silence all warnings
#   -fpermissive : GCC 14 promotes -Wincompatible-pointer-types /
#                  -Wint-conversion / -Wimplicit-* to hard errors by
#                  default; libspdm 3.8.2 + its pinned mbedTLS trip
#                  incompatible-pointer-types (const-qualified asn1
#                  buffers). Downgrade those permerrors back to
#                  warnings, which -w then swallows.
zephyr_library_compile_options(-w -fpermissive)
zephyr_library_compile_definitions(
  MBEDTLS_CONFIG_FILE=<libspdm_mbedtls_config.h>
  MBEDTLS_USER_CONFIG_FILE=<libspdm_mbedtls_user_config.h>
  LIBSPDM_CPU_ARM
)

file(GLOB _mbedtls_c        ${MBEDTLS_SRC}/library/*.c)
file(GLOB _spdm_common      ${LIBSPDM_DIR}/library/spdm_common_lib/*.c)
file(GLOB _spdm_crypt       ${LIBSPDM_DIR}/library/spdm_crypt_lib/*.c)   # non-recursive: skips fips/
file(GLOB _spdm_secmsg      ${LIBSPDM_DIR}/library/spdm_secured_message_lib/*.c)
file(GLOB _spdm_responder   ${LIBSPDM_DIR}/library/spdm_responder_lib/*.c)
file(GLOB _spdm_transport   ${LIBSPDM_DIR}/library/spdm_transport_mctp_lib/*.c)
file(GLOB_RECURSE _cryptlib ${LIBSPDM_DIR}/os_stub/cryptlib_mbedtls/*.c)

set(_osstub
  ${LIBSPDM_DIR}/os_stub/memlib/compare_mem.c
  ${LIBSPDM_DIR}/os_stub/memlib/copy_mem.c
  ${LIBSPDM_DIR}/os_stub/memlib/set_mem.c
  ${LIBSPDM_DIR}/os_stub/memlib/zero_mem.c
  ${LIBSPDM_DIR}/os_stub/malloclib/malloclib.c
  ${LIBSPDM_DIR}/os_stub/debuglib_null/debuglib.c
  # device-secret hooks: the _null impl satisfies the link for the
  # signing/measurement/PSK/CSR/event API. CERT_CAP + GET_CERTIFICATE +
  # GET_DIGESTS work with just the app-provided cert chain
  # (libspdm_set_data); CHALLENGE and signed MEASUREMENTS need a real
  # sign hook, added later in plat_spdm_secret.c which then replaces
  # this file in the source list.
  ${LIBSPDM_DIR}/os_stub/spdm_device_secret_lib_null/lib.c
)

zephyr_library_sources(
  ${PLAT_DIR}/src/plat_mbedtls_platform.c
  ${_mbedtls_c}
  ${_spdm_common}
  ${_spdm_crypt}
  ${_spdm_secmsg}
  ${_spdm_responder}
  ${_spdm_transport}
  ${_cryptlib}
  ${_osstub}
)

zephyr_library_include_directories(
  ${LIBSPDM_DIR}/include
  ${LIBSPDM_DIR}/include/hal
  ${LIBSPDM_DIR}/os_stub/include
  ${LIBSPDM_DIR}/os_stub
  ${LIBSPDM_DIR}/os_stub/cryptlib_mbedtls
  ${LIBSPDM_DIR}/os_stub/mbedtlslib/include
  ${LIBSPDM_DIR}/os_stub/mbedtlslib/include/mbedtls
  ${MBEDTLS_SRC}/include
  ${MBEDTLS_SRC}/library
  ${PLAT_DIR}/include
)
