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
 * libspdm responder device-secret hooks for the MCX-N9XX-EVK. Drop-in
 * replacement for os_stub/spdm_device_secret_lib_null/lib.c - provides
 * exactly the symbols this board's capability set
 * (include/plat_libspdm_config.h: CERT | CHAL | MEAS) pulls in:
 *
 *   libspdm_responder_data_sign               - REAL: ECDSA P-256 with
 *                                               the embedded dev key
 *   libspdm_measurement_collection            - REAL: one DMTF block,
 *                                               digest of a firmware
 *                                               region
 *   libspdm_generate_measurement_summary_hash - REAL: hash over that
 *                                               block (for CHALLENGE)
 *   libspdm_measurement_opaque_data           - none
 *   libspdm_challenge_opaque_data             - none
 *   libspdm_encap_challenge_opaque_data       - none
 *
 * Compiled inside the libspdm module library (see
 * libspdm/zephyr/CMakeLists.txt) so it gets LIBSPDM_CONFIG and the
 * libspdm/mbedTLS include paths.
 *
 * Key material is a dev/bring-up EC P-256 key in flash
 * (include/plat_spdm_certs.h) - this EVK has no secure key store.
 */

#include <stdint.h>
#include <string.h>

#include "hal/base.h"
#include "industry_standard/spdm.h"
#include "library/spdm_crypt_lib.h"
#include "library/spdm_return_status.h"
#include "hal/library/cryptlib.h"
#include "hal/library/responder/asymsignlib.h"
#include "hal/library/responder/measlib.h"

#include "plat_spdm_certs.h"

/* Firmware region measured by measurement index 1. A fixed window at
 * the start of the internal flash image (vector table + early boot
 * code) - deterministic and real, without needing a linker symbol for
 * the exact image extent. Not a whole-image hash; documented as such. */
#define PLAT_FW_MEAS_BASE ((const uint8_t *)0x10000000u)
#define PLAT_FW_MEAS_LEN  0x8000u

/* raw-bit-stream form of measurement index 1, when the requester asks
 * for raw instead of a digest. Fixed-size identifier blob. */
#define PLAT_MEAS_RAW_SIZE 32
static const uint8_t plat_meas_raw_id[PLAT_MEAS_RAW_SIZE] =
	"MCXN947 OpenBIC SPDM fw meas v1\0";

/* single measurement block: index 1 */
#define PLAT_MEAS_BLOCK_INDEX   1
#define PLAT_MEAS_BLOCK_COUNT   1

/* -------------------------------------------------------------------- */

static void *plat_new_dev_ec_context(uint32_t base_asym_algo)
{
	void *ec;

	if (base_asym_algo != SPDM_ALGORITHMS_BASE_ASYM_ALGO_TPM_ALG_ECDSA_ECC_NIST_P256) {
		/* only the embedded P-256 dev key is available */
		return NULL;
	}

	ec = libspdm_ec_new_by_nid(LIBSPDM_CRYPTO_NID_ECDSA_NIST_P256);
	if (ec == NULL) {
		return NULL;
	}
	if (!libspdm_ec_set_pub_key(ec, plat_spdm_dev_pub, plat_spdm_dev_pub_len) ||
	    !libspdm_ec_set_priv_key(ec, plat_spdm_dev_priv, plat_spdm_dev_priv_len)) {
		libspdm_ec_free(ec);
		return NULL;
	}
	return ec;
}

/*
 * Build measurement block index 1 into out (capacity out_cap).
 * digest form unless use_raw. Returns block size, or 0 on failure.
 */
static size_t plat_build_meas_block(uint32_t measurement_hash_algo, bool use_raw,
				    uint8_t *out, size_t out_cap, uint32_t base_hash_algo)
{
	spdm_measurement_block_dmtf_t *blk = (spdm_measurement_block_dmtf_t *)out;
	uint8_t *value = out + sizeof(spdm_measurement_block_dmtf_t);
	uint16_t value_size;
	uint8_t value_type;

	if (use_raw) {
		value_size = PLAT_MEAS_RAW_SIZE;
		value_type = SPDM_MEASUREMENT_BLOCK_MEASUREMENT_TYPE_IMMUTABLE_ROM |
			     SPDM_MEASUREMENT_BLOCK_MEASUREMENT_TYPE_RAW_BIT_STREAM;
	} else {
		uint32_t hs = libspdm_get_measurement_hash_size(measurement_hash_algo);

		if (hs == 0 || hs == 0xffffffffu || hs > 64) {
			return 0;
		}
		value_size = (uint16_t)hs;
		value_type = SPDM_MEASUREMENT_BLOCK_MEASUREMENT_TYPE_IMMUTABLE_ROM;
	}

	if (out_cap < sizeof(spdm_measurement_block_dmtf_t) + value_size) {
		return 0;
	}

	blk->measurement_block_common_header.index = PLAT_MEAS_BLOCK_INDEX;
	blk->measurement_block_common_header.measurement_specification =
		SPDM_MEASUREMENT_SPECIFICATION_DMTF;
	blk->measurement_block_common_header.measurement_size =
		(uint16_t)(sizeof(spdm_measurement_block_dmtf_header_t) + value_size);
	blk->measurement_block_dmtf_header.dmtf_spec_measurement_value_type = value_type;
	blk->measurement_block_dmtf_header.dmtf_spec_measurement_value_size = value_size;

	if (use_raw) {
		memcpy(value, plat_meas_raw_id, PLAT_MEAS_RAW_SIZE);
	} else if (!libspdm_hash_all(base_hash_algo, PLAT_FW_MEAS_BASE, PLAT_FW_MEAS_LEN, value)) {
		return 0;
	}

	return sizeof(spdm_measurement_block_dmtf_t) + value_size;
}

/* -------------------------------------------------------------------- */

bool libspdm_responder_data_sign(
#if LIBSPDM_HAL_PASS_SPDM_CONTEXT
	void *spdm_context,
#endif
	spdm_version_number_t spdm_version, uint8_t op_code,
	uint32_t base_asym_algo,
	uint32_t base_hash_algo, bool is_data_hash,
	const uint8_t *message, size_t message_size,
	uint8_t *signature, size_t *sig_size)
{
	void *ec;
	bool result;

	ec = plat_new_dev_ec_context(base_asym_algo);
	if (ec == NULL) {
		return false;
	}

	if (is_data_hash) {
		result = libspdm_asym_sign_hash(spdm_version, op_code, base_asym_algo, base_hash_algo,
					       ec, message, message_size, signature, sig_size);
	} else {
		result = libspdm_asym_sign(spdm_version, op_code, base_asym_algo, base_hash_algo,
					   ec, message, message_size, signature, sig_size);
	}

	libspdm_ec_free(ec);
	return result;
}

#if LIBSPDM_ENABLE_CAPABILITY_MEAS_CAP
libspdm_return_t libspdm_measurement_collection(
#if LIBSPDM_HAL_PASS_SPDM_CONTEXT
	void *spdm_context,
#endif
	spdm_version_number_t spdm_version,
	uint8_t measurement_specification,
	uint32_t measurement_hash_algo,
	uint8_t measurements_index,
	uint8_t request_attribute,
	uint8_t *content_changed,
	uint8_t *measurements_count,
	void *measurements,
	size_t *measurements_size)
{
	bool use_raw;
	size_t block_size;

	if (measurement_specification != SPDM_MEASUREMENT_SPECIFICATION_DMTF ||
	    measurement_hash_algo == 0) {
		return LIBSPDM_STATUS_UNSUPPORTED_CAP;
	}

	if (measurements_index ==
	    SPDM_GET_MEASUREMENTS_REQUEST_MEASUREMENT_OPERATION_TOTAL_NUMBER_OF_MEASUREMENTS) {
		*measurements_count = PLAT_MEAS_BLOCK_COUNT;
		goto done;
	}

	if (measurements_index !=
		    SPDM_GET_MEASUREMENTS_REQUEST_MEASUREMENT_OPERATION_ALL_MEASUREMENTS &&
	    measurements_index != PLAT_MEAS_BLOCK_INDEX) {
		*measurements_count = 0;
		return LIBSPDM_STATUS_MEAS_INVALID_INDEX;
	}

	use_raw = (measurement_hash_algo ==
		   SPDM_ALGORITHMS_MEASUREMENT_HASH_ALGO_RAW_BIT_STREAM_ONLY) ||
		  ((request_attribute &
		    SPDM_GET_MEASUREMENTS_REQUEST_ATTRIBUTES_RAW_BIT_STREAM_REQUESTED) != 0);

	/* base_hash_algo for the digest: measurement_hash_algo shares the
	 * SPDM hash-algo bit values, so it doubles as the base_hash_algo
	 * arg to libspdm_hash_all() for the non-raw case. */
	block_size = plat_build_meas_block(measurement_hash_algo, use_raw,
					  measurements, *measurements_size, measurement_hash_algo);
	if (block_size == 0) {
		return LIBSPDM_STATUS_BUFFER_TOO_SMALL;
	}

	*measurements_size = block_size;
	*measurements_count = 1;

done:
	if (content_changed != NULL &&
	    (spdm_version >> SPDM_VERSION_NUMBER_SHIFT_BIT) >= SPDM_MESSAGE_VERSION_12) {
		*content_changed = SPDM_MEASUREMENTS_RESPONSE_CONTENT_CHANGE_NO_DETECTION;
	}
	return LIBSPDM_STATUS_SUCCESS;
}

bool libspdm_measurement_opaque_data(
#if LIBSPDM_HAL_PASS_SPDM_CONTEXT
	void *spdm_context,
#endif
	spdm_version_number_t spdm_version,
	uint8_t measurement_specification,
	uint32_t measurement_hash_algo,
	uint8_t measurement_index,
	uint8_t request_attribute,
	void *opaque_data,
	size_t *opaque_data_size)
{
	*opaque_data_size = 0;
	return true;
}

bool libspdm_generate_measurement_summary_hash(
#if LIBSPDM_HAL_PASS_SPDM_CONTEXT
	void *spdm_context,
#endif
	spdm_version_number_t spdm_version,
	uint32_t base_hash_algo,
	uint8_t measurement_specification,
	uint32_t measurement_hash_algo,
	uint8_t measurement_summary_hash_type,
	uint8_t *measurement_summary_hash,
	uint32_t measurement_summary_hash_size)
{
	uint8_t block[sizeof(spdm_measurement_block_dmtf_t) + 64];
	size_t block_size;

	if (measurement_summary_hash_type == SPDM_REQUEST_NO_MEASUREMENT_SUMMARY_HASH) {
		return true;
	}
	if (measurement_summary_hash_type != SPDM_REQUEST_TCB_COMPONENT_MEASUREMENT_HASH &&
	    measurement_summary_hash_type != SPDM_REQUEST_ALL_MEASUREMENTS_HASH) {
		return false;
	}
	if (measurement_summary_hash_size != libspdm_get_hash_size(base_hash_algo)) {
		return false;
	}

	/* summary = base_hash over the concatenation of all measurement
	 * blocks; this board has exactly one. */
	block_size = plat_build_meas_block(measurement_hash_algo, false,
					  block, sizeof(block), base_hash_algo);
	if (block_size == 0) {
		return false;
	}
	return libspdm_hash_all(base_hash_algo, block, block_size, measurement_summary_hash);
}
#endif /* LIBSPDM_ENABLE_CAPABILITY_MEAS_CAP */

#if LIBSPDM_ENABLE_CAPABILITY_CHAL_CAP
bool libspdm_challenge_opaque_data(
#if LIBSPDM_HAL_PASS_SPDM_CONTEXT
	void *spdm_context,
#endif
	spdm_version_number_t spdm_version,
	uint8_t slot_id,
	uint8_t *measurement_summary_hash,
	size_t measurement_summary_hash_size,
	void *opaque_data,
	size_t *opaque_data_size)
{
	*opaque_data_size = 0;
	return true;
}

bool libspdm_encap_challenge_opaque_data(
#if LIBSPDM_HAL_PASS_SPDM_CONTEXT
	void *spdm_context,
#endif
	spdm_version_number_t spdm_version,
	uint8_t slot_id,
	uint8_t *measurement_summary_hash,
	size_t measurement_summary_hash_size,
	void *opaque_data,
	size_t *opaque_data_size)
{
	*opaque_data_size = 0;
	return true;
}
#endif /* LIBSPDM_ENABLE_CAPABILITY_CHAL_CAP */
