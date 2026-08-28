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
 * SPDM (DSP0274) responder over MCTP (DSP0275, message type 0x05),
 * backed by DMTF libspdm (modules/lib/libspdm, built via libspdm.cmake)
 * with the mbedTLS crypto backend.
 *
 * The board's MCTP endpoint is callback-driven: plat_mctp_msg_recv()
 * hands us one fully reassembled SPDM message and expects a synchronous
 * response. libspdm's responder model is pull/push
 * (libspdm_responder_dispatch_message() -> receive cb -> process ->
 * send cb), so this file bridges the two: stash the inbound message +
 * its MCTP return address, run one dispatch, and the send callback
 * forwards libspdm's transport-encoded reply back out over MCTP.
 *
 * Capabilities: CERT_CAP + CHAL_CAP + MEAS_CAP(SIG). Identity is an
 * embedded EC P-256 self-signed dev cert (plat_spdm_certs.h). The
 * responder device-secret hooks (ECDSA sign for CHALLENGE_AUTH /
 * signed MEASUREMENTS, and the firmware-region measurement itself)
 * live in plat_spdm_secret.c.
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "hal/base.h"
#include "library/spdm_common_lib.h"
#include "library/spdm_crypt_lib.h"
#include "library/spdm_responder_lib.h"
#include "library/spdm_transport_mctp_lib.h"
#include "industry_standard/spdm.h"

#include "mctp.h"
#include "plat_spdm.h"
#include "plat_spdm_certs.h"

LOG_MODULE_REGISTER(plat_spdm);

#define SPDM_BUF_SIZE (LIBSPDM_MAX_SPDM_MSG_SIZE + \
		       LIBSPDM_MCTP_TRANSPORT_HEADER_SIZE + LIBSPDM_MCTP_TRANSPORT_TAIL_SIZE)

static void *spdm_context;
static void *spdm_scratch;
static bool spdm_ready;

/* This responder negotiates a single base hash (SHA-384) so the
 * DSP0274 certificate-chain buffer below can carry one fixed root
 * hash. A requester that only offers SHA-256 for the base hash will
 * fail NEGOTIATE_ALGORITHMS - acceptable for this bring-up responder. */
#define PLAT_SPDM_BASE_HASH_ALGO  SPDM_ALGORITHMS_BASE_HASH_ALGO_TPM_ALG_SHA_384
#define PLAT_SPDM_ROOT_HASH_SIZE  48

/* Full DSP0274 cert chain handed to LIBSPDM_DATA_LOCAL_PUBLIC_CERT_CHAIN:
 * libspdm stores this pointer verbatim and slices it for
 * GET_CERTIFICATE, so it must already be
 *   length(2 LE) | reserved(2) | root_hash[H] | DER cert(s)
 * not raw DER. Built once in plat_spdm_init(); must stay resident. */
#define PLAT_SPDM_CERT_CHAIN_SIZE (sizeof(spdm_cert_chain_t) + PLAT_SPDM_ROOT_HASH_SIZE + \
				   sizeof(plat_spdm_dev_cert_der))
static uint8_t spdm_cert_chain[PLAT_SPDM_CERT_CHAIN_SIZE] __aligned(4);

static uint8_t sender_buf[SPDM_BUF_SIZE] __aligned(8);
static uint8_t receiver_buf[SPDM_BUF_SIZE] __aligned(8);
static bool sender_taken;
static bool receiver_taken;

/* one in-flight request from plat_mctp_msg_recv() */
static const uint8_t *in_req;
static uint32_t in_req_len;
static bool in_req_pending;
static mctp *in_mctp;
static mctp_ext_params in_ext;

static uint8_t last_request_code;
static bool seen_request;

/* ---- device buffer callbacks ------------------------------------- */

static libspdm_return_t acquire_sender_buffer(void *ctx, void **buf)
{
	ARG_UNUSED(ctx);
	if (sender_taken) {
		return LIBSPDM_STATUS_ACQUIRE_FAIL;
	}
	sender_taken = true;
	*buf = sender_buf;
	return LIBSPDM_STATUS_SUCCESS;
}

static void release_sender_buffer(void *ctx, const void *buf)
{
	ARG_UNUSED(ctx);
	ARG_UNUSED(buf);
	sender_taken = false;
}

static libspdm_return_t acquire_receiver_buffer(void *ctx, void **buf)
{
	ARG_UNUSED(ctx);
	if (receiver_taken) {
		return LIBSPDM_STATUS_ACQUIRE_FAIL;
	}
	receiver_taken = true;
	*buf = receiver_buf;
	return LIBSPDM_STATUS_SUCCESS;
}

static void release_receiver_buffer(void *ctx, const void *buf)
{
	ARG_UNUSED(ctx);
	ARG_UNUSED(buf);
	receiver_taken = false;
}

/* ---- device IO callbacks --------------------------------------------
 * "receive" hands libspdm the message we stashed from MCTP; "send"
 * forwards libspdm's transport-encoded reply back out over MCTP.
 */

static libspdm_return_t device_receive_message(void *ctx, size_t *msg_size, void **msg,
					      uint64_t timeout)
{
	ARG_UNUSED(ctx);
	ARG_UNUSED(timeout);

	if (!in_req_pending) {
		return LIBSPDM_STATUS_RECEIVE_FAIL;
	}
	if (in_req_len > SPDM_BUF_SIZE) {
		in_req_pending = false;
		return LIBSPDM_STATUS_RECEIVE_FAIL;
	}

	memcpy(*msg, in_req, in_req_len);
	*msg_size = in_req_len;
	in_req_pending = false;
	return LIBSPDM_STATUS_SUCCESS;
}

static libspdm_return_t device_send_message(void *ctx, size_t msg_size, const void *msg,
					   uint64_t timeout)
{
	ARG_UNUSED(ctx);
	ARG_UNUSED(timeout);

	if (!in_mctp) {
		return LIBSPDM_STATUS_SEND_FAIL;
	}

	/* msg already carries the MCTP message-type byte (0x05) from the
	 * mctp transport encoder - hand it straight to mctp_send_msg. */
	if (mctp_send_msg(in_mctp, (uint8_t *)msg, (uint16_t)msg_size, in_ext) != MCTP_SUCCESS) {
		LOG_ERR("SPDM response send failed");
		return LIBSPDM_STATUS_SEND_FAIL;
	}
	return LIBSPDM_STATUS_SUCCESS;
}

/* ---- init ------------------------------------------------------------ */

static bool set_u32(libspdm_data_type_t type, uint32_t v)
{
	libspdm_data_parameter_t p = { 0 };

	p.location = LIBSPDM_DATA_LOCATION_LOCAL;
	return libspdm_set_data(spdm_context, type, &p, &v, sizeof(v)) == LIBSPDM_STATUS_SUCCESS;
}

static bool set_u8(libspdm_data_type_t type, uint8_t v)
{
	libspdm_data_parameter_t p = { 0 };

	p.location = LIBSPDM_DATA_LOCATION_LOCAL;
	return libspdm_set_data(spdm_context, type, &p, &v, sizeof(v)) == LIBSPDM_STATUS_SUCCESS;
}

void plat_spdm_init(void)
{
	spdm_context = malloc(libspdm_get_context_size());
	if (!spdm_context) {
		LOG_ERR("spdm context alloc failed");
		return;
	}

	if (libspdm_init_context(spdm_context) != LIBSPDM_STATUS_SUCCESS) {
		LOG_ERR("libspdm_init_context failed");
		return;
	}

	libspdm_register_device_io_func(spdm_context, device_send_message, device_receive_message);
	libspdm_register_transport_layer_func(spdm_context, LIBSPDM_MAX_SPDM_MSG_SIZE,
					     LIBSPDM_MCTP_TRANSPORT_HEADER_SIZE,
					     LIBSPDM_MCTP_TRANSPORT_TAIL_SIZE,
					     libspdm_transport_mctp_encode_message,
					     libspdm_transport_mctp_decode_message);
	libspdm_register_device_buffer_func(spdm_context, SPDM_BUF_SIZE, SPDM_BUF_SIZE,
					   acquire_sender_buffer, release_sender_buffer,
					   acquire_receiver_buffer, release_receiver_buffer);

	size_t scratch_size = libspdm_get_sizeof_required_scratch_buffer(spdm_context);

	spdm_scratch = malloc(scratch_size);
	if (!spdm_scratch) {
		LOG_ERR("spdm scratch alloc (%zu) failed", scratch_size);
		return;
	}
	libspdm_set_scratch_buffer(spdm_context, spdm_scratch, scratch_size);

	/* capabilities: certificate + challenge + signed measurements */
	set_u8(LIBSPDM_DATA_CAPABILITY_CT_EXPONENT, 0);
	set_u32(LIBSPDM_DATA_CAPABILITY_FLAGS,
		SPDM_GET_CAPABILITIES_RESPONSE_FLAGS_CERT_CAP |
			SPDM_GET_CAPABILITIES_RESPONSE_FLAGS_CHAL_CAP |
			SPDM_GET_CAPABILITIES_RESPONSE_FLAGS_MEAS_CAP_SIG);

	/* algorithms: DMTF measurements; ECDSA P-256 only (that is the
	 * embedded dev key - advertising P-384 too made libspdm negotiate
	 * a curve we have no key for, so CHALLENGE/signed MEASUREMENTS
	 * failed); SHA-384 base hash only (see PLAT_SPDM_BASE_HASH_ALGO);
	 * measurement digests may be SHA-256 or SHA-384. */
	set_u8(LIBSPDM_DATA_MEASUREMENT_SPEC, SPDM_MEASUREMENT_SPECIFICATION_DMTF);
	set_u32(LIBSPDM_DATA_MEASUREMENT_HASH_ALGO,
		SPDM_ALGORITHMS_MEASUREMENT_HASH_ALGO_TPM_ALG_SHA_384 |
			SPDM_ALGORITHMS_MEASUREMENT_HASH_ALGO_TPM_ALG_SHA_256);
	set_u32(LIBSPDM_DATA_BASE_ASYM_ALGO,
		SPDM_ALGORITHMS_BASE_ASYM_ALGO_TPM_ALG_ECDSA_ECC_NIST_P256);
	set_u32(LIBSPDM_DATA_BASE_HASH_ALGO, PLAT_SPDM_BASE_HASH_ALGO);

	/* identity: build the DSP0274 cert chain (header + root hash + DER)
	 * around the embedded EC P-256 self-signed dev cert, slot 0. */
	spdm_cert_chain_t *cc = (spdm_cert_chain_t *)spdm_cert_chain;

	cc->length = (uint16_t)PLAT_SPDM_CERT_CHAIN_SIZE;
	cc->reserved = 0;
	if (!libspdm_hash_all(PLAT_SPDM_BASE_HASH_ALGO, plat_spdm_dev_cert_der,
			      sizeof(plat_spdm_dev_cert_der),
			      spdm_cert_chain + sizeof(spdm_cert_chain_t))) {
		LOG_ERR("SPDM cert-chain root hash failed");
		return;
	}
	memcpy(spdm_cert_chain + sizeof(spdm_cert_chain_t) + PLAT_SPDM_ROOT_HASH_SIZE,
	       plat_spdm_dev_cert_der, sizeof(plat_spdm_dev_cert_der));

	libspdm_data_parameter_t cp = { 0 };

	cp.location = LIBSPDM_DATA_LOCATION_LOCAL;
	cp.additional_data[0] = 0; /* slot id */
	if (libspdm_set_data(spdm_context, LIBSPDM_DATA_LOCAL_PUBLIC_CERT_CHAIN, &cp,
			     spdm_cert_chain,
			     PLAT_SPDM_CERT_CHAIN_SIZE) != LIBSPDM_STATUS_SUCCESS) {
		LOG_WRN("SPDM cert chain set failed - CERT_CAP will be degraded");
	}

	spdm_ready = true;
	LOG_INF("SPDM responder up (libspdm): CERT+CHAL+MEAS, ECDSA P-256, SHA-384");
}

/* ---- inbound message from plat_mctp_msg_recv() -------------------- */

void plat_spdm_handler(void *mctp_p, uint8_t *buf, uint32_t len, mctp_ext_params ext_params)
{
	if (!spdm_ready || !mctp_p || !buf || len < 4) {
		return;
	}

	/* buf[0] = MCTP type byte (0x05); buf[2] = SPDM request code */
	last_request_code = buf[2];
	seen_request = true;

	in_req = buf;
	in_req_len = len;
	in_req_pending = true;
	in_mctp = (mctp *)mctp_p;
	in_ext = ext_params;

	libspdm_return_t st = libspdm_responder_dispatch_message(spdm_context);

	if (LIBSPDM_STATUS_IS_ERROR(st) && st != LIBSPDM_STATUS_SEND_FAIL) {
		LOG_WRN("SPDM dispatch (req 0x%02x) status 0x%x", last_request_code, (uint32_t)st);
	}

	in_req_pending = false;
	in_mctp = NULL;
}

bool plat_spdm_last_request(uint8_t *code)
{
	if (code) {
		*code = last_request_code;
	}
	return seen_request;
}
