/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "flash.h"
#include "log.h"
#include "objects.h"
#include <stdlib.h>
#include <string.h>

#define INFO_NAME(_name) .name = _name,

#define INFO(_name, _jedec_id, _total_sz, _sector_sz, _flag) \
	INFO_NAME(_name) \
	.id = _jedec_id, \
	.total_sz = _total_sz, \
	.sector_sz = (_sector_sz), \
	.flag = (_flag)


const flash_info_t flash_ids[] = {
	{ INFO("mx25l25635f",  0xc22019, 32 * 1024, 64 * 1024, 0) },
	{ INFO("mx25l51245g",  0xc2201a, 64 * 1024, 64 * 1024, 0) },
	{ INFO("w25q512jvfq",  0xef4020, 64 * 1024, 64 * 1024, 0) },
	{ INFO("w25q80dv",  0xef4014, 1 * 1024, 4 * 1024, SPI_NOR_QUAD_READ) },
	{ },
};


