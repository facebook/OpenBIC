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

#ifndef I2C_MUX_PI4MSD5V9542_H
#define I2C_MUX_PI4MSD5V9542_H

#define PI4MSD5V9542_DEFAULT_SETTING 0
#define PI4MSD5V9542_ENABLE_BIT BIT(2)
#define PI4MSD5V9542_SELECT_CHANNEL_0 0
#define PI4MSD5V9542_SELECT_CHANNEL_1 1

enum PI4MSD5V9542_CHANNEL {
	PI4MSD5V9542_CHANNEL_0 = (PI4MSD5V9542_ENABLE_BIT | PI4MSD5V9542_SELECT_CHANNEL_0),
	PI4MSD5V9542_CHANNEL_1 = (PI4MSD5V9542_ENABLE_BIT | PI4MSD5V9542_SELECT_CHANNEL_1),
};

#endif
