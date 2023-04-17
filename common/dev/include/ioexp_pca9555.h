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

#ifndef IOEXP_PCA9555_H
#define IOEXP_PCA9555_H

enum PCA9555_REGISTER {
	PCA9555_INPUT_PORT_REG_0,
	PCA9555_INPUT_PORT_REG_1,
	PCA9555_OUTPUT_PORT_REG_0,
	PCA9555_OUTPUT_PORT_REG_1,
	PCA9555_POLARITY_INVERSION_REG_0,
	PCA9555_POLARITY_INVERSION_REG_1,
	PCA9555_CONFIG_REG_0,
	PCA9555_CONFIG_REG_1,
};

#endif
