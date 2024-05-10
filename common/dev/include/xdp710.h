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

#ifndef XDP710_H
#define XDP710_H

#include <stdint.h>

enum XDP710_VTLM_RNG {
	VTLM_RNG_88,
	VTLM_RNG_44,
	VTLM_RNG_22,
	VTLM_RNG_RESERVED,
};

enum XDP710_VSNS_CS {
	VSNS_CS_12_5,
	VSNS_CS_25,
	VSNS_CS_50,
	VSNS_CS_100,
	VSNS_CS_MAX,
};

typedef struct {
	bool mbr_init;
	uint16_t m;
	uint16_t b;
	uint16_t r; /* multiples of 10, r = -2 -> val = 100 */
	float r_sense; /* mohm */
} xdp710_priv;

#endif // XDP710_H
