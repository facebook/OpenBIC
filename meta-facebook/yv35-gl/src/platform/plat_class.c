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

#include "plat_class.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hal_gpio.h"
#include "hal_i2c.h"
#include "libutil.h"
#include "plat_gpio.h"
#include "plat_i2c.h"

static uint8_t system_class = SYS_CLASS_1;
static uint8_t board_revision = 0x0F;
static CARD_STATUS _1ou_status = { false, TYPE_1OU_UNKNOWN };

uint8_t get_system_class()
{
	return system_class;
}

CARD_STATUS get_1ou_status()
{
	return _1ou_status;
}

uint8_t get_board_revision()
{
	return board_revision;
}
