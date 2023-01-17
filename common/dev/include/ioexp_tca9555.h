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

#ifndef IOEXP_TCA9555_H
#define IOEXP_TCA9555_H

enum TCA9555_PORT {
	TCA9555_PORT_0 = BIT(0),
	TCA9555_PORT_1 = BIT(1),
	TCA9555_PORT_2 = BIT(2),
	TCA9555_PORT_3 = BIT(3),
	TCA9555_PORT_4 = BIT(4),
	TCA9555_PORT_5 = BIT(5),
	TCA9555_PORT_6 = BIT(6),
	TCA9555_PORT_7 = BIT(7),
};

enum TCA9555_REGISTER {
	TCA9555_INPUT_PORT_REG_0,
	TCA9555_INPUT_PORT_REG_1,
	TCA9555_OUTPUT_PORT_REG_0,
	TCA9555_OUTPUT_PORT_REG_1,
	TCA9555_POLARITY_INVERSION_REG_0,
	TCA9555_POLARITY_INVERSION_REG_1,
	TCA9555_CONFIG_REG_0,
	TCA9555_CONFIG_REG_1,
};

#endif
