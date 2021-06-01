/*
 * Copyright (c) Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _ASPEED_CRYPTO_REG_H_
#define _ASPEED_CRYPTO_REG_H_

#define ASPEED_HACE_SRC					0x00
#define ASPEED_HACE_DEST				0x04
#define ASPEED_HACE_CONTEXT				0x08	/* 8 byte aligned */
#define ASPEED_HACE_DATA_LEN			0x0C
#define ASPEED_HACE_CMD					0x10
#define  HACE_CMD_AES_KEY_FROM_OTP		BIT(24)
#define  HACE_CMD_GHASH_TAG_XOR_EN		BIT(23)
#define  HACE_CMD_GHASH_PAD_LEN_INV		BIT(22)
#define  HACE_CMD_GCM_TAG_ADDR_SEL		BIT(21)
#define  HACE_CMD_MBUS_REQ_SYNC_EN		BIT(20)
#define  HACE_CMD_DES_SG_CTRL			BIT(19)
#define  HACE_CMD_SRC_SG_CTRL			BIT(18)
#define  HACE_CMD_SINGLE_DES			0
#define  HACE_CMD_TRIPLE_DES			BIT(17)
#define  HACE_CMD_AES_SELECT			0
#define  HACE_CMD_DES_SELECT			BIT(16)
#define  HACE_CMD_CTR_IV_AES_128		0
#define  HACE_CMD_CTR_IV_DES_64			0
#define  HACE_CMD_CTR_IV_AES_96			(0x1 << 14)
#define  HACE_CMD_CTR_IV_DES_32			(0x1 << 14)
#define  HACE_CMD_CTR_IV_AES_64			(0x2 << 14)
#define  HACE_CMD_CTR_IV_AES_32			(0x3 << 14)
#define  HACE_CMD_AES_KEY_HW_EXP		BIT(13)
#define  HACE_CMD_ISR_EN				BIT(12)
#define  HACE_CMD_CONTEXT_SAVE_ENABLE	(0)
#define  HACE_CMD_CONTEXT_SAVE_DISABLE	BIT(9)
#define  HACE_CMD_AES					(0)
#define  HACE_CMD_DES					(0)
#define  HACE_CMD_RC4					BIT(8)
#define  HACE_CMD_DECRYPT				(0)
#define  HACE_CMD_ENCRYPT				BIT(7)
#define  HACE_CMD_ECB					(0)
#define  HACE_CMD_CBC					(0x1 << 4)
#define  HACE_CMD_CFB					(0x2 << 4)
#define  HACE_CMD_OFB					(0x3 << 4)
#define  HACE_CMD_CTR					(0x4 << 4)
#define  HACE_CMD_GCM					(0x5 << 4)
#define  HACE_CMD_AES128				(0)
#define  HACE_CMD_AES192				(0x1 << 2)
#define  HACE_CMD_AES256				(0x2 << 2)
#define  HACE_CMD_OP_CASCADE			(0x3)
#define  HACE_CMD_OP_INDEPENDENT		(0x1)
#define ASPEED_HACE_GCM_ADD_LEN			0x14
#define ASPEED_HACE_TAG					0x18
#define ASPEED_HACE_GCM_TAG_BASE_ADDR	0x18
#define ASPEED_HACE_STS					0x1C
#define  HACE_RSA_ISR					BIT(13)
#define  HACE_CRYPTO_ISR				BIT(12)
#define  HACE_HASH_ISR					BIT(9)
#define  HACE_RSA_BUSY					BIT(2)
#define  HACE_CRYPTO_BUSY				BIT(1)
#define  HACE_HASH_BUSY					BIT(0)
#define ASPEED_HACE_HASH_SRC			0x20
#define ASPEED_HACE_HASH_DIGEST_BUFF	0x24
#define ASPEED_HACE_HASH_KEY_BUFF		0x28	/* 16 byte aligned */
#define ASPEED_HACE_HASH_DATA_LEN		0x2C
#define ASPEED_HACE_HASH_CMD			0x30
#define  HASH_CMD_MBUS_REQ_SYNC_EN		BIT(20)
#define  HASH_CMD_HASH_SRC_SG_CTRL		BIT(18)
#define  HASH_CMD_ACC_LAST_BLOCK		BIT(14)
#define  HASH_CMD_ACC_FIRST_BLOCK		BIT(13)
#define  HASH_CMD_SHA512_224			(0x3 << 10)
#define  HASH_CMD_SHA512_256			(0x2 << 10)
#define  HASH_CMD_SHA384				(0x1 << 10)
#define  HASH_CMD_SHA512				(0)
#define  HASH_CMD_INT_ENABLE			BIT(9)
#define  HASH_CMD_INT_DISABLE			(0)
#define  HASH_CMD_HMAC					(0x1 << 7)
#define  HASH_CMD_ACC_MODE				(0x2 << 7)
#define  HASH_CMD_HMAC_KEY				(0x3 << 7)
#define  HASH_CMD_WITHOUT_HMAC			(0)
#define  HASH_CMD_MD5					(0)
#define  HASH_CMD_SHA1					(0x2 << 4)
#define  HASH_CMD_SHA224				(0x4 << 4)
#define  HASH_CMD_SHA256				(0x5 << 4)
#define  HASH_CMD_SHA512_SER			(0x6 << 4)
#define  HASH_CMD_MD5_SWAP				(0x1 << 2)
#define  HASH_CMD_SHA_SWAP				(0x2 << 2)
#define  HASH_CMD_CASCADED_CRYPTO_FIRST	(0x2)
#define  HASH_CMD_CASCADED_HASH_FIRST	(0x3)
#define ASPEED_HACE_HASH_DATA_PAD_LEN	0x34
#define ASPEED_HACE_RSA_MD_EXP_BIT		0x40

#define ASPEED_SEC_STS					0x14
#define ASPEED_SEC_MCU_MEMORY_MODE		0x5c
#define ASPEED_SEC_RSA_KEY_LEN			0xb0
#define ASPEED_SEC_RSA_TRIG				0xbc

extern uint32_t hace_base;
extern uint32_t sec_base;

#define HACE_RD(offs)		REG_RD(hace_base + offs)
#define HACE_WR(offs, val)	REG_WR(hace_base + offs, val)

#define SEC_RD(offs)		REG_RD(sec_base + offs)
#define SEC_WR(offs, val)	REG_WR(sec_base + offs, val)

#endif
