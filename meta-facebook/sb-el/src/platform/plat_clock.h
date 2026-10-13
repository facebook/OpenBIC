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
 */
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#define CLK_BUF_U85_ADDR (0xCE >> 1)
#define CLK_BUF_U690_ADDR (0xD8 >> 1)
#define CLK_BUF_U88_ADDR (0xDE >> 1)
#define CLK_GEN_100M_U86_ADDR (0x12 >> 1)
#define CLK_GEN_312_5M_U618_ADDR (0x10 >> 1)

/* for error indexing  */
#define CLK_100MHZ_ERR_IDX 0x1
#define CLK_312_5MHZ_ERR_IDX 0x2
#define CLK_BUF0_100M_LOSB_PLD 0x3
#define CLK_BUF1_100M_LOSB_PLD 0x4
#define CLK_BUF2_100M_LOSB_PLD 0x5
// #define CLK_312_5MHZ_REINIT_ERR_IDX 0x6

#define CLOCK_APLL_UNLOCK_EVENT 0x5B
#define CLK_312_5M_APLL_UNLOCK_EVENT 0x5C
#define CLK_BUF0_100M_LOSB_PLD_EVENT 0x5D
#define CLK_BUF1_100M_LOSB_PLD_EVENT 0x5E
#define CLK_BUF2_100M_LOSB_PLD_EVENT 0x5F

uint8_t clk_100mhz_get_lock_status_u86(void);
uint8_t clk_312_5mhz_get_lock_status_u618(void);
void check_clk_buf_loss_status(void);
bool clock_get_error_data(uint16_t error_code, uint8_t *data);