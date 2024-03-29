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

#ifndef I2C_MUX_TCA9543A_H
#define I2C_MUX_TCA9543A_H

#define TCA9543A_DEAULT_SETTING 0

enum TCA9543A_CHANNEL {
	TCA9543A_CHANNEL_0 = BIT(0),
	TCA9543A_CHANNEL_1 = BIT(1),
};

#endif
