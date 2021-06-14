/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "board_device.h"
#include "cmsis_os.h"
#include "flash_api.h"
#include "FreeRTOS_CLI.h"
#include "getopt.h"
#include "log.h"
#include "objects.h"
#include <stdio.h>
#include <string.h>

static const CLI_Command_Definition_t sf_cmd;
extern spi_t fmc_spi[];

/* for stress test */
static osThreadId_t tid_fmc_spi_task;
static osThreadAttr_t tattr_fmc_spi_task;

static void aspeed_fmc_spi_dump_buf(uint8_t *buf, uint32_t len)
{
	uint32_t i;

	if (buf == NULL) {
		return;
	}

	printf("buf: ");
	for (i = 0; i < len; i++) {
		printf("%02x ", buf[i]);
		if (i % 16 == 15) {
			printf("\n");
			printf("     ");
		}
	}
	printf("\r\n");
	return;
}

static void aspeed_fmc_spi_test(uint32_t spi, uint32_t cs)
{
	uint32_t i;
	uint32_t ret = 0;
	uint8_t *golden_buf = NULL;
	uint8_t *read_buf = NULL;
	uint32_t test_len = 0x100;
	uint32_t fail_count = 0;
	fmc_spi_priv_t *priv;
	const char *fmc_spi_name[3] = {"fmc", "spi1", "spi2"};

	if (spi > 3) {
		log_error("The maximum SPI number is 2 (from 0 to 2)(%d)\n", spi);
		goto end;
	}

	if (cs > 3) {
		log_error("The maximum CS number is 2 (from 0 to 2) (%d)\n", cs);
		goto end;
	}

	aspeed_flash_probe(&fmc_spi[spi], cs);

	golden_buf = (uint8_t *)pvPortMalloc(test_len);
	read_buf = (uint8_t *)pvPortMalloc(test_len);

	if (!golden_buf) {
		log_error("read buf is NULL\n");
		goto end;
	}

	if (!read_buf) {
		log_error("demo buf is NULL\n");
		goto end;
	}

	memset(read_buf, 0x0, test_len);

	for (i = 0; i < test_len; i++)
		golden_buf[i] = 'a' + (i % 26);

	priv = (fmc_spi_priv_t *)fmc_spi[spi].device->private;
	log_info("reading %dB from %s cs %d... \n", test_len, priv->name, cs);
	ret = aspeed_read_from_flash(fmc_spi[spi], cs,
				read_buf, 0x80000, test_len);
	if (ret) {
		log_info("%s %d, %s, ret = 0x%x\n",
			__func__, __LINE__, fmc_spi_name[spi], ret);
		fail_count++;
		goto end;
	}

	if (memcmp(read_buf, golden_buf, test_len)) {
		aspeed_fmc_spi_dump_buf(read_buf, test_len);
		log_info("write golden data to %s %d\n", priv->name, cs);
		ret = aspeed_update_flash(fmc_spi[spi], cs,
				golden_buf, 0x80000, test_len, false);
		if (ret) {
			log_info("%s %d, %s, ret = 0x%x\n",
				__func__, __LINE__, fmc_spi_name[spi], ret);
			fail_count++;
			goto end;
		}

		memset(read_buf, 0x0, test_len);
		ret = aspeed_read_from_flash(fmc_spi[spi], cs,
				read_buf, 0x80000, test_len);
		if (ret) {
			log_info("%s %d, %s, ret = 0x%x\n",
				__func__, __LINE__, fmc_spi_name[spi], ret);
			fail_count++;
			goto end;
		}

		if (memcmp(read_buf, golden_buf, test_len)) {
			log_error("fail to update golden data to %s %d\n", priv->name, cs);
			fail_count++;
			aspeed_fmc_spi_dump_buf(read_buf, test_len);
			goto end;
		}
	}

end:
	log_info("%s cs %d test %s.\n",
			fmc_spi_name[spi], cs, fail_count ? "fail" : "pass");

	if (golden_buf)
		vPortFree(golden_buf);
	if (read_buf)
		vPortFree(read_buf);

	return;
}

static void fmc_spi_stress(void *argv)
{
	uint32_t i = 0;
	uint32_t ret = 0;
	uint8_t *golden_buf = NULL;
	uint8_t *read_buf = NULL;
	uint32_t test_len = 0x100;
	uint32_t count = 0;
	const char *fmc_spi_name[3] = {"fmc", "spi1", "spi2"};

	for (i = 0; i < 3; i++)
		aspeed_fmc_spi_test(i, 0);

	golden_buf = (uint8_t *)pvPortMalloc(test_len);
	read_buf = (uint8_t *)pvPortMalloc(test_len);

	if (!golden_buf) {
		log_error("read buf is NULL\n");
		goto end;
	}

	if (!read_buf) {
		log_error("demo buf is NULL\n");
		goto end;
	}

	for (i = 0; i < test_len; i++)
		golden_buf[i] = 'a' + (i % 26);

	while(1) {
		log_debug("count = %d\n", count);
		for (i = 0; i < 3; i++) {
			memset(read_buf, 0x0, test_len);
			ret = aspeed_read_from_flash(fmc_spi[i], 0,
						read_buf, 0x80000, test_len);
			if (ret) {
				log_error("%s %d, %s, ret = 0x%x\n",
					__func__, __LINE__, fmc_spi_name[i], ret);
				goto end;
			}

			if (memcmp(read_buf, golden_buf, test_len)) {
				log_error("incorrect read data %s %d, %s", __func__, __LINE__, fmc_spi_name[i]);
				aspeed_fmc_spi_dump_buf(read_buf, test_len);
				goto end;
			}
		}

		if (count % 10 == 0)
			log_info("fmc_spi pass, count = %d\n", count);

		osDelay(1000);
		count++;
	}

end:
	if (golden_buf)
		vPortFree(golden_buf);
	if (read_buf)
		vPortFree(read_buf);

	return;
}

static void sf_cmd_handler(int argc, char *argv[])
{
	char option;
	uint32_t spi = 0;
	uint32_t cs = 0;
	bool test = false;
	bool stress = false;

	optind = 0;
	while ((option = getopt(argc, argv, "hts:c:r")) != (char)-1) {
		switch (option) {
			case 'h':
				printf("%s", sf_cmd.pcHelpString);
				return;
			case 't':
				test = true;
				break;
			case 's':
				spi = atoi(optarg);
				break;
			case 'c':
				cs = atoi(optarg);
				break;
			case 'r':
				stress = true;
				break;
			default:
				printf("unknown option -%c", option);
				break;
			}
	}

	if (stress) {
		tattr_fmc_spi_task.name = "fmc_spi_stress";
		tattr_fmc_spi_task.priority = osPriorityBelowNormal;
		tattr_fmc_spi_task.stack_size = 0x1000;

		tid_fmc_spi_task = osThreadNew(fmc_spi_stress, NULL, &tattr_fmc_spi_task);
		return;
	}

	if (test)
		aspeed_fmc_spi_test(spi, cs);
}

CLI_FUNC_DECL(sf, sf_cmd_handler);

static const CLI_Command_Definition_t sf_cmd =
{
	"sf",
	"\r\nsf:\n \
	usage: \r\n \
		[-t]: fmc_spi test for <spi> <cs>\r\n \
		[-s]: spi type: '0' for fmc, '1' for SPI1 and '2' for SPI2\r\n \
		[-c]: chip select\r\n",
	CLI_FUNC_SYM(sf),
	-1
};

void demo_fmc_spi_flash_init(void)
{
	FreeRTOS_CLIRegisterCommand(&sf_cmd);
}

