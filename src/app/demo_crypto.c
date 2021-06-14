/*
 * Copyright (c) Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include "stdlib.h"
#include "cmsis_os.h"
#include "board_device.h"
#include "objects.h"
#include "crypto_aspeed.h"
#include "getopt.h"
#include "FreeRTOS_CLI.h"
#include "log.h"

hace_t hace;
extern aspeed_device_t hace_dev;
static const CLI_Command_Definition_t crypto_cmd;

static void hash_test()
{
	struct hash_data hd;
	uint8_t *data = (uint8_t *)"asdfasdfasdf";
	int i;

	aspeed_hash_init(&hd, CRYPTO_SHA2_256);
	aspeed_hash_update(&hd, data, 13);
	aspeed_hash_final(&hd);

	for (i = 0; i < hd.sctx.digsize; i++) {
		printf("%02x", hd.sctx.digest[i]);
		if ((i + 1) % 32 == 0)
			printf("\n");
	}
	printf("\n");
	printf("finish\n");
}

static void crypto_cmd_handler(int argc, char *argv[])
{
	char option;
	bool test = 0;

	optind = 0;
	while ((option = getopt(argc, argv, "ht")) != (char) -1) {
		switch (option) {
		case 'h':
		case '?':
			printf("%s", crypto_cmd.pcHelpString);
			return;
		case 't':
			test = 1;
			break;
		default:
			log_warn("unknown option -%c", option);
			break;
		}
	}

	if (test) {
		hash_test();
	}

}

CLI_FUNC_DECL(crypto, crypto_cmd_handler);

static const CLI_Command_Definition_t crypto_cmd = {
	"crypto",
	"\r\ncrypto:\n \
	usage:\r\n \
		[-h]: Show this message\r\n \
		[-t]: test crypto\r\n",
	CLI_FUNC_SYM(crypto),
	-1
};

void demo_crypto_init(void)
{
	hace.device = &hace_dev;
	aspeed_hace_init(&hace);

	FreeRTOS_CLIRegisterCommand(&crypto_cmd);
}
