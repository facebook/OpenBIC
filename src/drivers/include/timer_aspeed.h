/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdint.h>
int aspeed_timer_init(ast_timer_t *obj);
int aspeed_timer_wait_us(ast_timer_t *obj, uint32_t us);
int aspeed_timer_wait_ms(ast_timer_t *obj, uint32_t ms);