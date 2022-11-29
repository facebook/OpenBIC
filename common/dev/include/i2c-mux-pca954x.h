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

#ifndef I2C_MUX_PCA9548A_H
#define I2C_MUX_PCA9548A_H

enum PCA9548A_CHANNEL {
	PCA9548A_CHANNEL_0 = BIT(0),
	PCA9548A_CHANNEL_1 = BIT(1),
	PCA9548A_CHANNEL_2 = BIT(2),
	PCA9548A_CHANNEL_3 = BIT(3),
	PCA9548A_CHANNEL_4 = BIT(4),
	PCA9548A_CHANNEL_5 = BIT(5),
	PCA9548A_CHANNEL_6 = BIT(6),
	PCA9548A_CHANNEL_7 = BIT(7),
};

enum PCA9546A_CHANNEL {
	PCA9546A_CHANNEL_0 = BIT(0),
	PCA9546A_CHANNEL_1 = BIT(1),
	PCA9546A_CHANNEL_2 = BIT(2),
	PCA9546A_CHANNEL_3 = BIT(3),
};

#endif
