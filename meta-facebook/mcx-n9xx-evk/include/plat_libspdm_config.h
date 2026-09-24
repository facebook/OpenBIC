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
 * libspdm build configuration for the MCX-N9XX-EVK SPDM responder
 * (pointed at by -DLIBSPDM_CONFIG). Every knob in libspdm's
 * include/library/spdm_lib_config.h is #ifndef-guarded, so this file
 * only states the deltas from upstream and then #includes the upstream
 * file to fill in every default we don't care about (robust across
 * libspdm version bumps).
 *
 * Shape: attestation-only responder - identity (certificate),
 * CHALLENGE, and MEASUREMENTS. No secure sessions, key exchange, PSK,
 * mutual auth, chunking, CSR, or events. mbedTLS crypto backend, ECDSA
 * P-256/P-384 + SHA-256/SHA-384 only.
 */

#ifndef PLAT_LIBSPDM_CONFIG_H
#define PLAT_LIBSPDM_CONFIG_H

/* ---- responder capabilities: keep CERT + CHAL + MEAS, drop the rest */
#define LIBSPDM_ENABLE_CAPABILITY_CERT_CAP 1
#define LIBSPDM_ENABLE_CAPABILITY_CHAL_CAP 1
#define LIBSPDM_ENABLE_CAPABILITY_MEAS_CAP 1

#define LIBSPDM_ENABLE_CAPABILITY_KEY_EX_CAP 0
#define LIBSPDM_ENABLE_CAPABILITY_PSK_CAP 0
#define LIBSPDM_ENABLE_CAPABILITY_HBEAT_CAP 0
#define LIBSPDM_ENABLE_CAPABILITY_MUT_AUTH_CAP 0
#define LIBSPDM_ENABLE_CAPABILITY_ENCAP_CAP 0
#define LIBSPDM_ENABLE_CAPABILITY_CSR_CAP 0
#define LIBSPDM_ENABLE_CAPABILITY_CSR_CAP_EX 0
#define LIBSPDM_ENABLE_CAPABILITY_SET_CERT_CAP 0
#define LIBSPDM_ENABLE_CAPABILITY_CHUNK_CAP 0
#define LIBSPDM_ENABLE_CAPABILITY_MEL_CAP 0
#define LIBSPDM_ENABLE_CAPABILITY_EVENT_CAP 0
#define LIBSPDM_ENABLE_CAPABILITY_GET_KEY_PAIR_INFO_CAP 0
#define LIBSPDM_ENABLE_CAPABILITY_SET_KEY_PAIR_INFO_CAP 0
#define LIBSPDM_ENABLE_CAPABILITY_ENDPOINT_INFO_CAP 0
#define LIBSPDM_ENABLE_VENDOR_DEFINED_MESSAGES 0

/* requester-side sends: responder-only build */
#define LIBSPDM_SEND_GET_CERTIFICATE_SUPPORT 0
#define LIBSPDM_SEND_CHALLENGE_SUPPORT 0
#define LIBSPDM_SEND_GET_ENDPOINT_INFO_SUPPORT 0
#define LIBSPDM_EVENT_RECIPIENT_SUPPORT 0
#define LIBSPDM_RESPOND_IF_READY_SUPPORT 1

/* ---- crypto algorithm support ------------------------------------- */
#define LIBSPDM_RSA_SSA_SUPPORT 0
#define LIBSPDM_RSA_PSS_SUPPORT 0
#define LIBSPDM_ECDSA_SUPPORT 1
#define LIBSPDM_SM2_DSA_SUPPORT 0
#define LIBSPDM_EDDSA_ED25519_SUPPORT 0
#define LIBSPDM_EDDSA_ED448_SUPPORT 0

#define LIBSPDM_FFDHE_SUPPORT 0
#define LIBSPDM_ECDHE_SUPPORT 0
#define LIBSPDM_SM2_KEY_EXCHANGE_SUPPORT 0

#define LIBSPDM_AEAD_GCM_SUPPORT 0
#define LIBSPDM_AEAD_CHACHA20_POLY1305_SUPPORT 0
#define LIBSPDM_AEAD_SM4_SUPPORT 0

#define LIBSPDM_SHA256_SUPPORT 1
#define LIBSPDM_SHA384_SUPPORT 1
#define LIBSPDM_SHA512_SUPPORT 0
#define LIBSPDM_SHA3_256_SUPPORT 0
#define LIBSPDM_SHA3_384_SUPPORT 0
#define LIBSPDM_SHA3_512_SUPPORT 0
#define LIBSPDM_SM3_256_SUPPORT 0

/* ---- sizing (this is a 320 KB SRAM part) ------------------------- */
#define LIBSPDM_MAX_SPDM_MSG_SIZE 0x1200
#define LIBSPDM_MAX_CERT_CHAIN_SIZE 0x1000
#define LIBSPDM_MAX_MEASUREMENT_RECORD_SIZE 0x400

/* no secure sessions; context still sizes a few arrays off this */
#define LIBSPDM_MAX_SESSION_COUNT 1

/* ---- misc ------------------------------------------------------- */
#define LIBSPDM_DEBUG_ENABLE 0
#define LIBSPDM_FIPS_MODE 0

/* pull in every default we did not override above */
#include "library/spdm_lib_config.h"

#endif /* PLAT_LIBSPDM_CONFIG_H */
