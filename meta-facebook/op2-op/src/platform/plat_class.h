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

#ifndef PLAT_CLASS_H
#define PLAT_CLASS_H

#include <stdbool.h>
#include <stdint.h>

enum CARD_POSITION {
	CARD_POSITION_1OU,
	CARD_POSITION_2OU,
	CARD_POSITION_3OU,
	CARD_POSITION_4OU,
	CARD_POSITION_UNKNOWN,
};

enum CARD_TYPE {
	CARD_TYPE_OPA,
	CARD_TYPE_OPB,
	CARD_TYPE_UNKNOWN,
};

int init_platform_config();
uint8_t get_card_type();
uint8_t get_card_position();

#endif
