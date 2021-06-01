/*
 * Copyright (c) Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _CRYPTO_ASPEED_H_
#define _CRYPTO_ASPEED_H_
#include "objects.h"

enum aspeed_crypto_op {
	CRYPTO_DES_CBC = 1,
	CRYPTO_3DES_CBC = 2,
	CRYPTO_AES_CBC = 3,
	CRYPTO_AES_CTR = 4,
	CRYPTO_AES_ECB = 5,
	CRYPTO_AES_GCM = 6,
	CRYPTO_SHA1_HMAC = 10,
	CRYPTO_SHA2_224_HMAC = 11,
	CRYPTO_SHA2_256_HMAC = 12,
	CRYPTO_SHA2_384_HMAC = 13,
	CRYPTO_SHA2_512_HMAC = 14,
	CRYPTO_SHA1 = 100,
	CRYPTO_SHA2_224 = 101,
	CRYPTO_SHA2_256 = 102,
	CRYPTO_SHA2_384 = 103,
	CRYPTO_SHA2_512 = 104,
	CRYPTO_ALGORITHM_ALL
};

struct aspeed_sg_list {
	uint32_t len;
	void *phy_addr;
};

struct sha_context {
	uint8_t	digest[64] __aligned(64);
	uint64_t digcnt[2];
	size_t digsize;
	size_t block_size;
	size_t bufcnt;
	uint8_t buffer[256];
};

struct hash_data {
	uint32_t cipher;
	uint32_t cmd;
	uint32_t offset;
	void *src_addr;
	size_t src_len;
	struct sha_context sctx;
	struct aspeed_sg_list src_list[2];
};

struct rsa_key {
	char *m;
	char *e;
	char *d;
	uint32_t m_bits;
	uint32_t e_bits;
	uint32_t d_bits;
};

int aspeed_rsa_enc(char *data, uint32_t data_len, char *dst, struct rsa_key *rsa_key);
int aspeed_rsa_dec(char *data, uint32_t data_len, char *dst, struct rsa_key *rsa_key);
void aspeed_rsa_init(rsa_t *sec);

#endif /* #ifndef _CRYPTO_ASPEED_H_ */