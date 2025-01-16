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

#ifndef C2H_NPCM_H
#define C2H_NPCM_H

typedef enum {
	CR07_LDN = 0x07,
	CR15_L_GIO = 0x15,
	CR1A_L_PWD = 0x1A,
	CR1B_L_FUN = 0x1B,
	CR20_SID_H = 0x20,
	CR21_SID_L = 0x21,
	CR24_CHPREV = 0x24,
	CR25_SIOCF5 = 0x25,
	CR26_SIOCF6 = 0x26,
	CR28_SIOGPS = 0x28,
	CR29_SIOCF9 = 0x29,
	CR2A_DEVICE_ID = 0x2A,
	CR2D_CFD = 0x2D,
	CR2F_SIOSFR = 0x2F,
} CR_REG_Enum;

void c2h_npcm_init(void);
int get_chip_id(void);
int get_chip_rev(void);
int get_device_id(void);

#endif
