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

#ifndef I2C_MUX_PCA984X_H
#define I2C_MUX_PCA984X_H

#define PCA9846_MUTEX_LOCK_MS 1000
#define PCA9846_DEFAULT_CHANNEL 0

enum PCA9846_CHANNEL {
	PCA9846_CHANNEL_0 = BIT(0),
	PCA9846_CHANNEL_1 = BIT(1),
	PCA9846_CHANNEL_2 = BIT(2),
	PCA9846_CHANNEL_3 = BIT(3),
};

bool set_pca9846_channel_and_transfer(uint8_t bus, uint8_t mux_addr, uint8_t mux_channel,
				      uint8_t tran_type, I2C_MSG *msg);

#endif
