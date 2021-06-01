/*
 * Copyright (c) Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include "reset_aspeed.h"
#include "clk_aspeed.h"
#include "wait.h"
#include "log.h"
#include "hal_def.h"
#include "aspeed_crypto_reg.h"
#include "crypto_aspeed.h"

#define MCU_SRAM_BASE 0x79000000
#define RSA_MAX_LEN 0x400

uint32_t sec_base;
char *sram;
bool busy;

static int aspeed_rsa_trigger(char *data, uint32_t data_len, char *dst,
							  char *m, char *e,
							  uint32_t m_bits, uint32_t e_bits)
{
	uint32_t m_len = (m_bits + 7) / 8;
	uint32_t e_len = (e_bits + 7) / 8;
	uint32_t sts;
	char *s;
	int leading_zero;
	int i, j;

	if (data_len > 512) {
		log_error("the maxium rsa input data length is 4096\n");
		return HAL_ERROR;
	}

	memset(sram, 0, 0x1800);

	for (i = 0; i < e_len; i++) {
		sram[i] = e[e_len - i - 1];
	}

	s = sram + 0x400;
	for (i = 0; i < m_len; i++) {
		s[i] = m[m_len - i - 1];
	}

	s = sram + 0x800;
	for (i = 0; i < data_len; i++) {
		s[i] = data[data_len - i - 1];
	}

	SEC_WR(ASPEED_SEC_RSA_KEY_LEN, e_bits << 16 | m_bits);
	SEC_WR(ASPEED_SEC_RSA_TRIG, 1);
	SEC_WR(ASPEED_SEC_RSA_TRIG, 0);
	do {
		osDelay(10);
		sts = SEC_RD(ASPEED_SEC_STS);
	} while (!(sts & BIT(4)));

	s = sram + 0x1400;
	i = 0;
	leading_zero = 1;
	for (j = RSA_MAX_LEN - 1; j >= 0; j--) {
		if (s[j] != 0 || !leading_zero) {
			leading_zero = 0;
			dst[i] = s[j];
			i++;
		}
	}

	memset(sram, 0, 0x1800);

	return HAL_OK;
}

int aspeed_rsa_enc(char *data, uint32_t data_len, char *dst, struct rsa_key *rsa_key)
{
	int ret;

	if (busy) {
		return HAL_BUSY;
	} else {
		busy = 1;
		ret = aspeed_rsa_trigger(data, data_len, dst, rsa_key->m, rsa_key->e,
								 rsa_key->m_bits, rsa_key->e_bits);
		busy = 0;
		return ret;
	}
}
int aspeed_rsa_dec(char *data, uint32_t data_len, char *dst, struct rsa_key *rsa_key)
{
	int ret;

	if (busy) {
		return HAL_BUSY;
	} else {
		busy = 1;
		ret = aspeed_rsa_trigger(data, data_len, dst, rsa_key->m, rsa_key->d,
								 rsa_key->m_bits, rsa_key->d_bits);
		busy = 0;
		return ret;
	}
}

void aspeed_rsa_init(rsa_t *rsa)
{
	aspeed_device_t *device = rsa->device;

	sram = (char *)MCU_SRAM_BASE;
	sec_base = device->base;

	if (device->init) {
		return;
	}

	busy = 0;
	memset(sram, 0, 0x1800);
	aspeed_clk_enable(device);

	device->init = 1;
}