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

#ifndef HDC1080_H
#define HDC1080_H

enum HDC1080_MODE {
	TEMP_OR_HUM = 0, // Temperature or Humidity is acquired.
	TEMP_THEN_HUM, // Temperature and Humidity are acquired in sequence, Temperature first
};

enum HDC1080_TEMP_RESOLUTION {
	TEMP_14_BIT = 0,
	TEMP_11_BIT,
};

enum HDC1080_HUM_RESOLUTION {
	HUM_14_BIT = 0,
	HUM_11_BIT,
	HUM_8_BIT,
};

typedef struct {
	uint8_t idx; // Create index based on init variable
	uint8_t mode;
	uint8_t tres; // Temperature Measurement Resolution
	uint8_t hres; //Humidity Measurement Resolution
	sys_snode_t node; // linked list node
} hdc1080_data;

#define HDC1080_TEMP_OFFSET 0x00
#define HDC1080_HUM_OFFSET 0x01
#define HDC1080_CONFIGURE_OFFSET 0x02

#endif //HDC1080_H