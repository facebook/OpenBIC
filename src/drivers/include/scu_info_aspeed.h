/*
 * Copyright (c) Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ASPEED_SCU_INFO_API_H
#define ASPEED_SCU_INFO_API_H

#include <stdint.h>
#include "objects.h"
#include "pinmap.h"
#include "buffer.h"
#include "hal_def.h"

void aspeed_print_soc_id(void);
void aspeed_print_sysrst_info(void);
void aspeed_print_security_info(void);
void aspeed_print_2nd_wdt_mode(void);
void aspeed_print_fmc_aux_ctrl(void);
void aspeed_print_spi1_abr_mode(void);
void aspeed_print_spi1_aux_ctrl(void);
void aspeed_print_spi_strap_mode(void);
void aspeed_print_espi_mode(void);

#endif
