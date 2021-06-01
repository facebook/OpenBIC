/*
 * Copyright (c) Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include "log.h"
#include "hal_def.h"
#include "aspeed_crypto_reg.h"
#include "crypto_aspeed.h"

static const uint32_t sha1_iv[8] = {
	0x01234567UL, 0x89abcdefUL, 0xfedcba98UL, 0x76543210UL,
	0xf0e1d2c3UL, 0, 0, 0
};

static const uint32_t sha224_iv[8] = {
	0xd89e05c1UL, 0x07d57c36UL, 0x17dd7030UL, 0x39590ef7UL,
	0x310bc0ffUL, 0x11155868UL, 0xa78ff964UL, 0xa44ffabeUL
};

static const uint32_t sha256_iv[8] = {
	0x67e6096aUL, 0x85ae67bbUL, 0x72f36e3cUL, 0x3af54fa5UL,
	0x7f520e51UL, 0x8c68059bUL, 0xabd9831fUL, 0x19cde05bUL
};

static const uint32_t sha384_iv[16] = {
	0x5d9dbbcbUL, 0xd89e05c1UL, 0x2a299a62UL, 0x07d57c36UL,
	0x5a015991UL, 0x17dd7030UL, 0xd8ec2f15UL, 0x39590ef7UL,
	0x67263367UL, 0x310bc0ffUL, 0x874ab48eUL, 0x11155868UL,
	0x0d2e0cdbUL, 0xa78ff964UL, 0x1d48b547UL, 0xa44ffabeUL
};

static const uint32_t sha512_iv[16] = {
	0x67e6096aUL, 0x08c9bcf3UL, 0x85ae67bbUL, 0x3ba7ca84UL,
	0x72f36e3cUL, 0x2bf894feUL, 0x3af54fa5UL, 0xf1361d5fUL,
	0x7f520e51UL, 0xd182e6adUL, 0x8c68059bUL, 0x1f6c3e2bUL,
	0xabd9831fUL, 0x6bbd41fbUL, 0x19cde05bUL, 0x79217e13UL
};

static void aspeed_ahash_iv(struct hash_data *ctx)
{
	struct sha_context *sctx = &ctx->sctx;

	switch (ctx->cipher) {
	case CRYPTO_SHA1:
		memcpy(sctx->digest, sha1_iv, 32);
		break;
	case CRYPTO_SHA2_224:
		memcpy(sctx->digest, sha224_iv, 32);
		break;
	case CRYPTO_SHA2_256:
		memcpy(sctx->digest, sha256_iv, 32);
		break;
	case CRYPTO_SHA2_384:
		memcpy(sctx->digest, sha384_iv, 64);
		break;
	case CRYPTO_SHA2_512:
		memcpy(sctx->digest, sha512_iv, 64);
		break;
	}
}

static void aspeed_hash_fill_padding(struct hash_data *ctx)
{
	struct sha_context *sctx = &ctx->sctx;
	unsigned int index, padlen;
	uint64_t bits[2];

	switch (ctx->cipher) {
	case CRYPTO_SHA1:
	case CRYPTO_SHA2_224:
	case CRYPTO_SHA2_256:
		bits[0] = bswap_64(sctx->digcnt[0] << 3);
		index = sctx->bufcnt & 0x3f;
		padlen = (index < 56) ? (56 - index) : ((64 + 56) - index);
		*(sctx->buffer + sctx->bufcnt) = 0x80;
		memset(sctx->buffer + sctx->bufcnt + 1, 0, padlen - 1);
		memcpy(sctx->buffer + sctx->bufcnt + padlen, bits, 8);
		sctx->bufcnt += padlen + 8;
		break;
	case CRYPTO_SHA2_384:
	case CRYPTO_SHA2_512:
		bits[1] = bswap_64(sctx->digcnt[0] << 3);
		bits[0] = bswap_64(sctx->digcnt[1] << 3 | sctx->digcnt[0] >> 61);
		index = sctx->bufcnt & 0x7f;
		padlen = (index < 112) ? (112 - index) : ((128 + 112) - index);
		*(sctx->buffer + sctx->bufcnt) = 0x80;
		memset(sctx->buffer + sctx->bufcnt + 1, 0, padlen - 1);
		memcpy(sctx->buffer + sctx->bufcnt + padlen, bits, 16);
		sctx->bufcnt += padlen + 16;
		break;
	}
}

static void aspeed_hash_trigger(struct hash_data *ctx)
{
	struct sha_context *sctx = &ctx->sctx;
	uint32_t sts;

	HACE_WR(ASPEED_HACE_HASH_SRC, ctx->src_addr);
	HACE_WR(ASPEED_HACE_HASH_DIGEST_BUFF, sctx->digest);
	HACE_WR(ASPEED_HACE_HASH_KEY_BUFF, sctx->digest);
	HACE_WR(ASPEED_HACE_HASH_DATA_LEN, ctx->src_len);
	HACE_WR(ASPEED_HACE_HASH_CMD, ctx->cmd);

	do {
		sts = HACE_RD(ASPEED_HACE_STS);
	} while (sts & HACE_HASH_BUSY);
	HACE_WR(ASPEED_HACE_STS, sts);
}

int aspeed_hash_init(struct hash_data *ctx, uint32_t cipher)
{
	struct sha_context *sctx = &ctx->sctx;

	ctx->cmd = HASH_CMD_ACC_MODE;
	ctx->cipher = cipher;

	switch (cipher) {
	case CRYPTO_SHA1:
		ctx->cmd |= HASH_CMD_SHA1 | HASH_CMD_SHA_SWAP;
		sctx->digsize = 20;
		sctx->block_size = 64;
		break;
	default:
		log_error("cipher:%d not support\n", cipher);
		return HAL_ERROR;
	}
	aspeed_ahash_iv(ctx);

	sctx->bufcnt = 0;
	sctx->digcnt[0] = 0;
	sctx->digcnt[1] = 0;

	return 0;
}

int aspeed_hash_update(struct hash_data *ctx, const uint8_t *input, uint32_t in_len)
{
	struct sha_context *sctx = &ctx->sctx;
	uint32_t remaining;
	uint32_t offset;
	uint32_t index = 0;
	uint32_t pre_len;

	sctx->digcnt[0] += in_len;
	if (sctx->digcnt[0] < in_len)
		sctx->digcnt[1]++;

	if (in_len + sctx->bufcnt < sctx->block_size) {
		memcpy(sctx->buffer + sctx->bufcnt, input, in_len);
		sctx->bufcnt += in_len;
		return 0;
	}

	remaining = (in_len + sctx->bufcnt) % sctx->block_size;
	pre_len = in_len + sctx->bufcnt - remaining;
	offset = sctx->bufcnt - remaining;
	ctx->src_len = pre_len;

	if (sctx->bufcnt != 0) {
		ctx->src_list[0].phy_addr = sctx->buffer;
		ctx->src_list[0].len = sctx->bufcnt;
		pre_len -= sctx->bufcnt;
		if (pre_len == 0)
			ctx->src_list[0].len |= BIT(31);
		index = 1;
	}

	if (pre_len != 0) {
		ctx->src_list[index].phy_addr = input;
		ctx->src_list[index].len = pre_len | BIT(31);
	}
	ctx->src_addr = ctx->src_list;
	ctx->cmd |= HASH_CMD_HASH_SRC_SG_CTRL;

	aspeed_hash_trigger(ctx);

	memcpy(sctx->buffer, input + offset, remaining);
	sctx->bufcnt = remaining;

	return 0;
}

int aspeed_hash_final(struct hash_data *ctx)
{
	struct sha_context *sctx = &ctx->sctx;

	aspeed_hash_fill_padding(ctx);
	ctx->src_addr = sctx->buffer;
	ctx->src_len = sctx->bufcnt;

	ctx->cmd &= ~HASH_CMD_HASH_SRC_SG_CTRL;
	aspeed_hash_trigger(ctx);
	sctx->bufcnt = 0;

	return 0;
}



